# Rigid Body Laboratory

An interactive, browser-based 6-DOF **kinematics** laboratory. The initial scope is deliberately prescribed-pose kinematics; it does not present a fake force-driven dynamics mode.

## Run locally

Requires Node.js 20 or later.

```sh
npm install
npm run dev
```

Use the six inputs to set position `(x,y,z)` and Euler orientation `(roll,pitch,yaw)`. Drag in the viewport to orbit; camera motion does not affect the body pose. The geometry menu, visualization toggles, angle display, reset buttons, matrix output, Euler composition frames, and body point transformation all use the same pose state and math engine.

## Verify and build

```sh
npm test
npm run build
```

## Architecture

- `src/physics/math.ts`: rendering-independent vectors, matrix and quaternion operations, pose composition, and coordinate maps.
- `src/main.ts`: Three.js scene and interface wiring; the scene consumes the physics math rather than defining it.
- `docs/physics_conventions.md`: explicit frames, units, active Z-Y-X convention, angular velocity mapping, and scope.
- `tests/math.test.ts`: deterministic tests for rotation, transforms, quaternion equivalence, and singularity handling.

## Scope and limitations

The displayed object is a visual model; no mass properties, force integrator, contact, or dynamic response are implemented. The asymmetric example is a low-poly illustrative mesh rather than a validated mass model. Euler angles become singular at pitch ±90°; quaternion output remains a nonsingular orientation representation, but direct pose controls remain Euler-based. Grid extent is 20 m. References and the exact implementation convention are in the physics conventions document.
