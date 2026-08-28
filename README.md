# Joint Controller and Understanding mjModel / mjData

A from-scratch torque controller for the 7-DoF Franka Emika Panda in MuJoCo.

The controller is thirty lines. The project targets the layer beneath it: how MuJoCo lays out state across `qpos`, `qvel` and `ctrl`, which index maps to which joint, and where an external torque enters the equation of motion. Getting that wrong is the reason most first MuJoCo controllers fail, so this notebook makes each mapping explicit before writing any control law.

## What the notebook covers

- Reading `nq`, `nv` and `nu`, and why the Panda has nine joints but eight actuators
- Mapping joints to `qpos` slots and actuators to their joint or tendon targets
- Disabling the built-in position servos so `qfrc_applied` becomes the only driving force
- Gravity compensation from `qfrc_bias`, cross-checked against `mj_inverse`
- PD, PD with gravity compensation, and computed torque with an inertial feedforward term
- Torque clamping against the Franka datasheet limits
- Control-rate decimation, and the gain retuning it forces

## Results

Tracking a 0.25 Hz sine of amplitude 0.4 rad on all seven joints, 500 Hz control:

| Controller | max error (rad) | rms error (rad) |
|---|---|---|
| PD | 0.0714 | 0.0237 |
| PD + gravity compensation | 0.0120 | 0.0039 |
| PD + gravity + inertial feedforward | 0.0119 | 0.0034 |

Dropping the control loop to 100 Hz with the same gains destabilises the arm. Scaling `KP` by 0.35 recovers stability at roughly three times the tracking error, which is the accuracy-for-robustness trade every hardware controller has to make.

## Setup

```bash
pip install mujoco numpy matplotlib
git clone https://github.com/google-deepmind/mujoco_menagerie
jupyter notebook notebook.ipynb
```

Run from the directory containing `mujoco_menagerie/`. Tested on MuJoCo 3.12.

## Notes

The notebook neutralises the Menagerie position actuators by zeroing their gain and bias parameters, then drives the arm through `qfrc_applied`. For a project that needs actuator saturation modelled, copy `panda.xml` and replace the `<position>` block with `<motor>` entries carrying the datasheet `ctrlrange`.

`mj_mulM` handles the mass-matrix product instead of `mj_fullM`, since the internal storage of `qM` changed across MuJoCo 3.x releases.

## Next

Task-space control: end-effector Jacobian from `mj_jacSite`, damped least squares inverse kinematics, and a mocap target you drag in the viewer.