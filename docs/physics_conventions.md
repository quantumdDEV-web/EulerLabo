# Physics conventions

## Units and vectors

The simulator uses a right-handed Cartesian world frame and SI units: position in metres, time in seconds, mass in kilograms, and angles in radians internally. Vectors are column vectors. Coordinates are written with a frame subscript, such as `p_B` for a point expressed in body coordinates.

## Orientation

Orientation is an active rotation: it rotates a body's local vectors into the fixed world frame. `R_WB` maps body-frame components to world-frame components. The body orientation is parameterized by intrinsic yaw, pitch, roll about the current Z, Y, then X axes. Equivalently, for column vectors the same orientation is the extrinsic fixed-axis X, then Y, then Z sequence:

`R_WB = Rz(yaw) Ry(pitch) Rx(roll)`.

Thus `p_W = R_WB p_B + t_WB`, and `p_B = R_WB^T (p_W - t_WB)`. A free vector is rotated without translation. The transpose is the inverse because `R_WB` is orthonormal. Positive angles follow the right-hand rule. The displayed roll, pitch, yaw are the angles of this parameterization, not three independent spatial rotation freedoms. At pitch `+/- pi/2`, roll and yaw cannot be uniquely recovered (gimbal lock); the body still has three physical rotational degrees of freedom.

## Angular velocity and display

Angular velocity is a physical vector, shown in world coordinates in the kinematics view. It is not the vector of Euler angle rates. For this convention, with body components `omega_B` and Euler rates `(rollDot,pitchDot,yawDot)`, `omega_B = [rollDot - yawDot sin(pitch), pitchDot cos(roll) + yawDot sin(roll) cos(pitch), -pitchDot sin(roll) + yawDot cos(roll) cos(pitch)]`. This map is singular at gimbal lock. The interface may display degrees, but conversion occurs only at the display/input boundary; calculations remain in radians.

## Scope

The initial laboratory is kinematic: it prescribes pose and demonstrates coordinate transforms. It does not claim to integrate forces, torques, collisions, or a dynamic rigid body. These require a separately validated dynamics implementation.

## References

The rotation-sequence convention follows the standard composition treatment in J. B. Kuipers, *Quaternions and Rotation Sequences*, introductory chapters on rotations and Euler angles. Rotation matrices and rigid-body coordinate transformations are standard rigid-body kinematics; see H. Goldstein, C. Poole, and J. Safko, *Classical Mechanics*, chapters on rigid-body motion. These references are suggested background; this implementation does not rely on page-specific claims.
