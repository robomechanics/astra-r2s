# M8 contact integration in the dual-YAM environment

Work in progress: the passive M8 model now compiles in the real dual-YAM
workcell. Pads are attached to the native sliding finger bodies. The nut stays
free, and the robot's joint motors supply all manipulation forces. The bolt is
clamped to a slender bench support; the left arm is in standby.

![Actual YAM fingers around the M8 nut](../media/yam_m8_progress.png)

This is a recorded exploratory turn, not an accepted demo. A three-stroke
120° schedule fits the native wrist limits and preserves hex-flat alignment
when the fingers open, reset, and regrasp. The first grasp transmits about
5.94 N per pad and the commanded 0.05 N axial feed. The first turn fails the
strict per-stroke lead check: 26 µm residual versus an 8.3 µm limit, accompanied
by nut rocking and arm orientation tracking error. Controller work is underway;
the thread geometry, friction, contact parameters and gates remain unchanged.

The source, runnable full demo, recorded video and complete evidence will be
published after the arm-driven validation run. The original independent M8
[qualification limits](m8_status.md) remain in effect.
