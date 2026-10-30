"""EV3 program generator (Stage 5.1).

Reads the inferred kinematic model and emits a robot-specific control
package under ``generated/<robot>/``: constants, synchronized motion,
homing and safety guards. The output carries abstract kinematics only
(joint axes, limits, coupling, brick/port wiring): public.
"""
