"""Finite-torque arm simulation with a geometry-gated thread surrogate.

The motion is designed from observed task phases, not recovered robot telemetry.
Free objects are retained by switchable grasp welds; arm qpos is never assigned
after initialization. Screw velocity comes from measured driver angular velocity.
"""
from __future__ import annotations

import math
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .kinematics import ArmIK, HOME, frame, quat_wxyz
from .scene import build_model, INSERTION, PITCH, TURNS, DRIVER_TIP_Z

STROKES = 36
DRIVE_END = 10.0+STROKES
RELEASE_TIME = DRIVE_END+4.5
DURATION = RELEASE_TIME+3.5
WORK_CENTER = np.array([.37, -.015, .24])
WORK_ROTATION = Rotation.from_euler("y", -60, degrees=True).as_matrix()


def smooth(u):
    u = np.clip(u,0,1)
    return u*u*(3-2*u)


def blend_pose(a,b,u):
    u = float(smooth(u))
    return a[0]*(1-u)+b[0]*u, Slerp([0,1],Rotation.from_matrix([a[1],b[1]]))([u]).as_matrix()[0]


def engagement(tip,head,tool_axis,normal,turning):
    """Geometric engagement gate, independently testable without simulation."""
    delta = np.asarray(tip)-head
    axial = float(np.dot(delta,normal))
    lateral = float(np.linalg.norm(delta-axial*np.asarray(normal)))
    alignment = float(np.dot(tool_axis,-np.asarray(normal)))
    engaged = bool(turning and lateral < .004 and abs(axial) < .006 and alignment > .985)
    return engaged, lateral, axial, alignment


class TwinSimulation:
    def __init__(self):
        self.model = build_model()
        self.data = mujoco.MjData(self.model)
        self.ik = {s:ArmIK(self.model,s) for s in ["left","right"]}
        self.thread_q = self.model.joint("screw_angle").qposadr[0]
        self.depth_q = self.model.joint("screw_depth").qposadr[0]
        self.thread_ctrl = self.model.actuator("thread_drive").id
        for side,ik in self.ik.items():
            self.data.qpos[ik.qadr] = HOME
            self.data.ctrl[[self.model.actuator(f"{side}_servo{i}").id for i in range(1,7)]] = HOME
        mujoco.mj_forward(self.model,self.data)
        self.home = {s:self.site_pose(f"{s}_grasp_site") for s in self.ik}
        self.held = False
        self.released = False
        self.phase = "SETTLE"
        self.turning = False
        self.events = []
        self.records = []
        self.max_tip_error = 0.
        self.engaged_steps = 0
        self.turn_steps = 0
        self.max_arm_tracking_error = 0.
        self.last_gate = (False,0.,0.,0.)
        self.last_command = {s:HOME.copy() for s in self.ik}
        self.last_velocity = {s:np.zeros(6) for s in self.ik}
        # Only initialization sets qpos. Objects then settle onto pickup rests.
        for _ in range(250):
            self._physics_step()
        self.data.time = 0.
        self.starts = {"plate":self.body_pose("plate"),"screwdriver":self.body_pose("screwdriver")}
        left_pick = self.starts["plate"][0]+self.starts["plate"][1]@np.array([0,.043,0])
        self.pick = {"left":(left_pick,frame([0,-1,0],[0,0,-1])),
                     "right":self.starts["screwdriver"]}
        # Fail early if the chosen pickup cannot be reached with real limits.
        for side in self.ik:
            self.ik[side].solve(*self.pick[side],thorough=True)
            self.ik[side].q = HOME.copy()

    def body_pose(self,name):
        idx = self.model.body(name).id
        return self.data.xpos[idx].copy(), self.data.xmat[idx].reshape(3,3).copy()

    def site_pose(self,name):
        idx = self.model.site(name).id
        return self.data.site_xpos[idx].copy(),self.data.site_xmat[idx].reshape(3,3).copy()

    def _weld(self,name,a,b):
        idx = self.model.equality(name).id
        ap,ar = self.body_pose(a); bp,br = self.body_pose(b)
        self.model.eq_data[idx,3:6] = ar.T@(bp-ap)
        self.model.eq_data[idx,6:10] = quat_wxyz(ar.T@br)
        self.data.eq_active[idx] = True

    def _grasp(self,t):
        self._weld("plate_grip","left_link_6","plate")
        self._weld("driver_grip","right_link_6","screwdriver")
        self.object_offsets = {}
        for side,obj in [("left","plate"),("right","screwdriver")]:
            sp,sr = self.site_pose(f"{side}_grasp_site")
            op,orr = self.body_pose(obj)
            self.object_offsets[side] = (sr.T@(op-sp),sr.T@orr)
        self.held = True
        relative = self.object_offsets["right"][1]
        self.tool_roll_offset = math.atan2(relative[1,0],relative[0,0])
        self.events.append({"time":t,"event":"grasp_welds_enabled"})

    def object_to_grasp(self,side,p,r):
        offset,relative = self.object_offsets[side]
        rr = r@relative.T
        return p-rr@offset,rr

    def _targets(self,t):
        self.turning = False
        if t < 1:
            self.phase = "APPROACH"
            return self.home
        if t < 4:
            self.phase = "PICK UP PLATE + DRIVER"
            if t < 2.8:
                above = {s:(p+[0,0,.08],r) for s,(p,r) in self.pick.items()}
                return {s:blend_pose(self.home[s],above[s],(t-1)/1.8) for s in self.ik}
            return {s:blend_pose((p+[0,0,.08],r),(p,r),(t-2.8)/1.2) for s,(p,r) in self.pick.items()}
        if t < 5:
            self.phase = "GRASP"
            if t >= 4.6 and not self.held:
                self._grasp(t)
            return self.pick
        if t < 9:
            self.phase = "LIFT + PRESENT PLATE"
            u = smooth((t-5)/4)
            board = blend_pose(self.starts["plate"],(WORK_CENTER,WORK_ROTATION),u)
            left = self.object_to_grasp("left",*board)
            n = WORK_ROTATION[:,2]
            right_final_r = frame(-n,[0,0,-1])@Rotation.from_rotvec([0,0,-1.6+self.tool_roll_offset]).as_matrix()
            head = WORK_CENTER+n*(.0134+.0021)
            tool = blend_pose(self.starts["screwdriver"],(head+n*(DRIVER_TIP_Z+.035),right_final_r),u)
            return {"left":left,"right":self.object_to_grasp("right",*tool)}
        if t < DRIVE_END:
            left = self.object_to_grasp("left",WORK_CENTER,WORK_ROTATION)
            plate_p,plate_r = self.body_pose("plate")
            n = plate_r[:,2]
            head,_ = self.site_pose("screw_head_site")
            clearance = .035*(1-smooth(t-9)) if t < 10 else 0.
            roll = -1.6
            if t < 10:
                self.phase = "ENGAGE PHILLIPS TIP"
            else:
                cycle = (t-10)%1.
                if cycle < .58:
                    self.phase = "TURN CLOCKWISE"
                    self.turning = True
                    roll = -1.6+2.55*smooth(cycle/.58)
                elif cycle < .72:
                    self.phase = "LIFT TIP"
                    roll = .95
                    clearance = .014*smooth((cycle-.58)/.14)
                elif cycle < .92:
                    self.phase = "RESET WRIST"
                    roll = .95-2.55*smooth((cycle-.72)/.20)
                    clearance = .014
                else:
                    self.phase = "RE-ENGAGE"
                    clearance = .014*(1-smooth((cycle-.92)/.08))
            tool_r = frame(-n,[0,0,-1])@Rotation.from_rotvec([0,0,roll+self.tool_roll_offset]).as_matrix()
            tool_p = head+n*(DRIVER_TIP_Z+clearance)
            return {"left":left,"right":self.object_to_grasp("right",tool_p,tool_r)}
        if t < RELEASE_TIME:
            self.phase = "WITHDRAW + RETURN OBJECTS"
            if not hasattr(self,"return_poses"):
                self.return_poses = {"plate":self.body_pose("plate"),"screwdriver":self.body_pose("screwdriver")}
                # First lift off the screw head before returning through space.
                self.return_poses["screwdriver"] = (self.return_poses["screwdriver"][0]+WORK_ROTATION[:,2]*.035,self.return_poses["screwdriver"][1])
            u = (t-DRIVE_END)/4.5
            return {side:self.object_to_grasp(side,*blend_pose(self.return_poses[obj],self.starts[obj],u))
                    for side,obj in [("left","plate"),("right","screwdriver")]}
        self.phase = "RELEASE + HOME"
        if not self.released:
            for name in ["plate_grip","driver_grip"]:
                self.data.eq_active[self.model.equality(name).id] = False
            self.released = True
            self.release_poses = {s:self.site_pose(f"{s}_grasp_site") for s in self.ik}
            self.events.append({"time":t,"event":"grasp_welds_disabled"})
        if t < RELEASE_TIME+1:
            return {s:(p+np.array([0,0,.10])*smooth(t-RELEASE_TIME),r) for s,(p,r) in self.release_poses.items()}
        return {s:blend_pose((self.release_poses[s][0]+[0,0,.10],self.release_poses[s][1]),self.home[s],(t-RELEASE_TIME-1)/2.5) for s in self.ik}

    def command(self,t):
        for side,(p,r) in self._targets(t).items():
            q = self.ik[side].solve(p,r)
            self.last_velocity[side] = np.clip((q-self.last_command[side])/.02,-10,10)
            for i,val in enumerate(q,1):
                self.data.ctrl[self.model.actuator(f"{side}_servo{i}").id] = val
            self.last_command[side] = q
            closed = t >= 4 and t < RELEASE_TIME
            self.data.ctrl[self.model.actuator(f"{side}_grip").id] = (.033 if side=="left" else .026) if closed else .002

    def _physics_step(self):
        # Add inverse-dynamics bias compensation inside the position-actuator
        # command, so the complete PD + compensation force is torque limited.
        self.data.qfrc_applied[:] = 0.
        for side,ik in self.ik.items():
            for i,(dadr,q) in enumerate(zip(ik.dadr,self.last_command[side]),1):
                kp = 450. if i <= 3 else 100.
                kv = 22. if i <= 3 else 1.5
                self.data.ctrl[self.model.actuator(f"{side}_servo{i}").id] = q+(self.data.qfrc_bias[dadr]+kv*self.last_velocity[side][i-1])/kp
        tip,tool_r = self.site_pose("driver_tip")
        head,_ = self.site_pose("screw_head_site")
        _,board_r = self.body_pose("plate")
        self.last_gate = engagement(tip,head,tool_r[:,2],board_r[:,2],self.turning)
        engaged = self.last_gate[0]
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_BODY,self.model.body("screwdriver").id,vel,0)
        board_vel = np.zeros(6)
        mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_BODY,self.model.body("plate").id,board_vel,0)
        clockwise = max(0.,float(np.dot(vel[:3]-board_vel[:3],-board_r[:,2])))
        seated = self.data.qpos[self.thread_q] >= TURNS*2*math.pi-.015
        self.data.ctrl[self.thread_ctrl] = min(clockwise,12.) if engaged and not seated else 0.
        if self.turning:
            self.turn_steps += 1
            self.engaged_steps += int(engaged)
            self.max_tip_error = max(self.max_tip_error,float(np.linalg.norm(tip-head)))
        mujoco.mj_step(self.model,self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError("Non-finite physics state")
        if self.data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number:
            raise RuntimeError("MuJoCo reports unstable accelerations")

    def advance(self,until):
        while self.data.time < until-1e-8:
            t = float(self.data.time)
            # Cartesian control at 50 Hz; physics at 500 Hz.
            if round(t/.002)%10 == 0:
                self.command(t)
            self._physics_step()
        self.record()

    def record(self):
        for side,ik in self.ik.items():
            err = float(np.max(np.abs(self.data.qpos[ik.qadr]-self.last_command[side])))
            self.max_arm_tracking_error = max(self.max_arm_tracking_error,err)
        self.records.append({"time":float(self.data.time),"phase":self.phase,
                             "insertion_mm":float(-self.data.qpos[self.depth_q]*1000),
                             "turns":float(self.data.qpos[self.thread_q]/(2*math.pi)),
                             "engaged":bool(self.last_gate[0]),"lateral_error_mm":self.last_gate[1]*1000,
                             "axial_error_mm":self.last_gate[2]*1000})

    def report(self):
        angle = float(self.data.qpos[self.thread_q])
        depth = float(-self.data.qpos[self.depth_q])
        helix_error = abs(depth-angle*PITCH/(2*math.pi))
        success = bool(depth >= INSERTION-.00015 and helix_error < .0001 and self.held and self.released)
        return {"success":success,"simulation_time_s":float(self.data.time),"clockwise_turns":angle/(2*math.pi),
                "insertion_mm":depth*1000,"target_insertion_mm":INSERTION*1000,
                "helix_constraint_error_mm":helix_error*1000,
                "engagement_fraction_during_turns":self.engaged_steps/max(self.turn_steps,1),
                "max_turn_tip_error_mm":self.max_tip_error*1000,
                "max_arm_joint_tracking_error_rad":self.max_arm_tracking_error,
                "ik_max_position_error_mm":{s:ik.max_position_error*1000 for s,ik in self.ik.items()},
                "ik_max_orientation_error_rad":{s:ik.max_orientation_error for s,ik in self.ik.items()},
                "events":self.events,"warnings":{str(i):int(w.number) for i,w in enumerate(self.data.warning) if w.number},
                "object_return_error_mm":{obj:float(np.linalg.norm(self.body_pose(obj)[0]-self.starts[obj][0]))*1000 for obj in self.starts},
                "model_scope":"Video-informed motion; finite-torque arm dynamics, switched grasp welds, geometry-gated analytic M4 thread. No inferred torque telemetry, frictional grasp validation, tooth contacts, or trained policy."}
