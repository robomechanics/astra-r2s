"""Plot closed da69 original array evidence; no simulation or force replay."""
import hashlib
import json
import os
from pathlib import Path
import numpy as np
os.environ.setdefault('MPLCONFIGDIR','/tmp/m8-original-failure-plot-mpl')
os.environ.setdefault('XDG_CACHE_HOME','/tmp/m8-original-failure-plot-fonts')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):
            h.update(b)
    return h.hexdigest()


here=Path(__file__).resolve().parent
run=here.with_name('full_c2_inertia_v2')
audits=here.with_name('full_c2_inertia_v2_audits')
binding=audits/'independent_audit_binding.json'
assert digest(binding)=='4be80e698c07137d6182dffa297586af18084569d594b4dc49f47ef187f9e605'
frozen_inputs=json.loads(binding.read_text())['original_run_files_sha256']
for filename in ('insertion_validation.json','insertion_trace.npz','native_feedback_force_history.npz','robot_inertia_command_history.npz'):
    assert digest(run/filename)==frozen_inputs[filename]
supplement=audits/'supplemental_open_reset_contact_audit.json'
assert digest(supplement)=='caa7ad3714640ab099bd23aa3fddfc91dc8806228dfe4603a7ed9d42a436a15d'
report=json.loads(supplement.read_text())
metadata=json.loads((run/'insertion_validation.json').read_text())
assert metadata['passed'] is False and metadata['aborted']['phase']=='reset_open_search_2'
dt=float(metadata['scene_config']['base']['thread']['timestep'])
with np.load(run/'native_feedback_force_history.npz',allow_pickle=False) as z:
    labels=json.loads(str(z['phase_labels_json']))
    feedback={k:z[k] for k in ('time','phase_index','right_robot_bolt_contact_count',
        'right_actual_aperture_postintegration_m','right_pad_normal_force_N')}
mask=feedback['phase_index']==labels.index('reset_open_search_2')
ids=np.flatnonzero(mask)
assert len(ids)==21686
with np.load(run/'robot_inertia_command_history.npz',allow_pickle=False) as z:
    post=z['time'][ids]
    command_time=z['command_time_s'][ids]
    pd_force=z['pd_wrench_world_N_Nm'][ids,:3]
    applied_force=z['capped_wrench_world_N_Nm'][ids,:3]
    clipped=z['cartesian_force_clipped'][ids]
    assert not np.any(z['enabled'][ids])
assert np.array_equal(post,feedback['time'][ids])
with np.load(run/'insertion_trace.npz',allow_pickle=False) as z:
    rows=json.loads(str(z['info_json']))
selected=[row for row in rows if row['phase']=='reset_open_search_2']
error_post=np.array([row['time'] for row in selected])
error_retained=np.array([row['inertia_control']['retained_native_state_time_s'] for row in selected])
error_norm=np.array([np.linalg.norm(row['right_position_error_m']) for row in selected])
assert np.allclose(error_retained,error_post-2*dt,rtol=0.,atol=1e-10)
native_time=feedback['time'][ids]-dt
contacts=feedback['right_robot_bolt_contact_count'][ids]
cap_index=int(np.flatnonzero(clipped)[0])
contact_index=int(np.flatnonzero(contacts>0)[0])
assert np.sum(clipped)==4683 and contact_index==len(ids)-1
first_cap_label=float(post[cap_index])
first_contact_label=float(post[contact_index])
cap_actual=float(command_time[cap_index])
contact_actual=float(native_time[contact_index])
assert np.isclose(first_cap_label,report['first_reset_force_cap_post_label_time_s'])
assert np.isclose(first_contact_label,report['first_fully_open_contact_post_label_time_s'])
pd_norm=np.linalg.norm(pd_force,axis=1)
applied_norm=np.linalg.norm(applied_force,axis=1)
np.savez_compressed(here/'original_reset_plot_data.npz',
    original_native_indices=ids,post_label_time_s=post,robot_command_time_s=command_time,
    original_uncapped_PD_force_N=pd_force,original_applied_force_N=applied_force,
    original_uncapped_PD_force_norm_N=pd_norm,original_applied_force_norm_N=applied_norm,
    original_cartesian_force_clipped=clipped,original_native_contact_time_s=native_time,
    original_right_robot_bolt_contact_count=contacts,
    original_actual_postintegration_jaw_aperture_m=feedback['right_actual_aperture_postintegration_m'][ids],
    saved_error_post_label_time_s=error_post,saved_error_retained_pose_time_s=error_retained,
    original_saved_retained_position_error_norm_m=error_norm)

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.titlesize':13,'axes.labelsize':11,
    'figure.facecolor':'#f5f7fb','axes.facecolor':'white','axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(3,1,figsize=(13.2,9.7),sharex=True,gridspec_kw={'height_ratios':[2.2,1.5,1.0]})
fig.subplots_adjust(left=.085,right=.97,top=.88,bottom=.15,hspace=.25)
fig.suptitle('M8 open reset: tracking lag and pad recontact',x=.085,y=.965,ha='left',fontsize=19,fontweight='bold',color='#192d47')
fig.text(.085,.925,'Original failed fresh attempt  |  21,686 executed reset steps  |  No regrasp or full capture',color='#536174',fontsize=11.3)
ax=axes[0]
ax.plot(command_time,pd_norm,color='#d56c27',lw=2,label='Original uncapped PD force')
ax.plot(command_time,applied_norm,color='#2466a8',lw=2,label='Applied finite Cartesian force')
ax.axhline(8,color='#778394',lw=1,ls='--',label='8 N Cartesian cap')
ax.set_ylabel('Robot command force (N)')
ax.set_ylim(-3,165)
ax.legend(loc='upper left',frameon=False,ncol=1)
ax.text(.02,.43,'4,683 reset steps clipped at 8 N\nNo torque or native motor clipping',transform=ax.transAxes,color='#536174')
ax.annotate(f'First force clip\npost label {first_cap_label:.5f} s',xy=(cap_actual,8),xytext=(cap_actual-.23,78),
    color='#784116',fontsize=10,arrowprops={'arrowstyle':'->','color':'#d56c27'},bbox={'boxstyle':'round,pad=.35','fc':'#fff5ea','ec':'none'})
ax.annotate(f'{pd_norm[-1]:.2f} N requested\n8 N applied',xy=(command_time[-1],pd_norm[-1]),xytext=(command_time[-1]-.23,129),
    fontsize=10,color='#784116',arrowprops={'arrowstyle':'->','color':'#d56c27'})
ax=axes[1]
ax.plot(error_retained,error_norm*1000,color='#6c4ea2',lw=1.7,marker='.',ms=3,label='Original saved retained-position error')
ax.set_ylabel('Tool position error (mm)')
ax.set_ylim(-.2,7.5)
ax.legend(loc='upper left',frameon=False)
ax.text(.02,.50,f'{len(selected)} original samples (about 5 ms apart)\nPosition error belongs to retained command pose at t − 2dt',transform=ax.transAxes,color='#536174',fontsize=10)
ax.annotate(f'{error_norm[-1]*1000:.3f} mm',xy=(error_retained[-1],error_norm[-1]*1000),xytext=(error_retained[-1]-.18,6.5),
    color='#6c4ea2',fontsize=10,arrowprops={'arrowstyle':'->','color':'#6c4ea2'})
ax=axes[2]
ax.step(native_time,contacts,where='post',color='#b34948',lw=2,label='Original whole-right-robot / bolt contacts')
ax.scatter([contact_actual],[contacts[-1]],color='#b34948',s=44,zorder=5)
ax.set_ylabel('Native contact count')
ax.set_yticks([0,1]);ax.set_ylim(-.1,1.35)
ax.text(.02,.68,f'Contact-free until final solved step\nJaw gap held near 30 mm; final {feedback["right_actual_aperture_postintegration_m"][ids[-1]]*1000:.5f} mm',transform=ax.transAxes,color='#536174',fontsize=10)
ax.annotate(f'First pad recontact\npost label {first_contact_label:.5f} s\n1.487 N original pad normal',
    xy=(contact_actual,1),xytext=(contact_actual-.25,.58),fontsize=9.6,color='#943a3a',
    arrowprops={'arrowstyle':'->','color':'#b34948'},bbox={'boxstyle':'round,pad=.28','fc':'#fff0ef','ec':'none'})
for ax in axes:
    ax.axvline(cap_actual,color='#d56c27',ls=':',lw=1,alpha=.65)
    ax.axvline(contact_actual,color='#b34948',ls=':',lw=1,alpha=.65)
    ax.grid(axis='y',color='#dce1e8',lw=.6)
axes[-1].set_xlim(command_time[0]-.012,command_time[-1]+.015)
axes[-1].set_xlabel('Original command / native force-state time (s)')
fig.text(.085,.088,'Timing: robot commands and native contact geometry are at recorded post label t − dt; retained error samples are at t − 2dt.',fontsize=9.5,color='#536174')
fig.text(.085,.059,'dt = 50 µs. Callouts retain original post labels; vertical markers use their actual command / solved-contact times.',fontsize=9.5,color='#536174')
fig.text(.085,.031,'Closed original arrays only. No simulation, force reconstruction or successful slower-reset prediction.',fontsize=9.5,color='#536174')
fig.savefig(here/'open_reset_tracking_failure.png',dpi=150)
plt.close(fig)

inputs={str(p.relative_to(run.parent)):digest(p) for p in (run/'insertion_validation.json',run/'insertion_trace.npz',
    run/'native_feedback_force_history.npz',run/'robot_inertia_command_history.npz',binding,supplement)}
outputs={name:digest(here/name) for name in ('plot_original_open_reset_failure.py','original_reset_plot_data.npz','open_reset_tracking_failure.png')}
manifest={'kind':'Closed original open-reset control failure plot','closed':True,'producer_commit':report['producer_commit'],
    'original_native_child_exit_code':1,'original_native_passed':False,'native_trajectory_qualification':False,
    'inputs_sha256':inputs,'outputs_sha256':outputs,'reset_native_rows':21686,'saved_retained_error_samples':len(selected),
    'total_original_native_rows':441657,'first_force_cap_post_label_s':first_cap_label,'first_force_cap_command_time_s':cap_actual,
    'first_contact_post_label_s':first_contact_label,'first_contact_native_force_time_s':contact_actual,
    'first_contact_original_pad_normal_N':float(feedback['right_pad_normal_force_N'][ids[-1],0]),
    'plot_scope':'Every original executed reset control row and native contact count; original sparse retained-error samples only. No interpolation, new native model/force solve/integration, future run read, original audit/source/runtime mutation or new trajectory success claim.',
    'timestamp_scope':'Command force plotted at authoritative command_time_s; contact count plotted at original post label minusdt; position error plotted at authoritative retained_native_state_time_s. Callouts separately preserve original post labels.'}
(here/'plot_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
print(json.dumps({'plot':str(here/'open_reset_tracking_failure.png'),'plot_sha256':outputs['open_reset_tracking_failure.png'],
    'manifest_sha256':digest(here/'plot_manifest.json'),'native_reset_rows':21686,'saved_error_samples':len(selected)}))
