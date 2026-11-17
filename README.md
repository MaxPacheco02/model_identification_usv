# model_identification_usv

ROS 2 (Humble) package for the system identification of a differential-thrust USV: open-loop test
program, logging, and a least-squares fit of the 3-DOF manoeuvring model. It runs against Gazebo
(gz-sim 8, Harmonic, with the `gz-waves` hydrodynamics of `asv_wave_sim`) or against the real vessel,
since it only talks through topics. It is a package, not a workspace: clone it into the `src/` of
the workspace that uses it (`asv_nav`, `vanttec_usv`).

## Model

About the body origin, body frame x forward, y left, z up (the coefficients are the same in
Fossen's x forward, y right, z down):

```
M nu_dot + C(nu) nu + D(nu) nu = tau,      nu = [u, v, r],   tau = [X, Y, N]

M = [m11 0 0; 0 m22 m23; 0 m23 m33]        C(nu) from M (Fossen 2011, eq. 6.43)
X: d_u1 u + d_u2 |u| u
Y: d_v1 v + d_v2 |v| v + d_vr r
N: d_r1 r + d_r2 |r| r + d_rv1 v + d_rv2 |v| v
```

Fossen's coefficients are the negatives (`X_u = -d_u1`, `Y_v = -d_v1`, ...). The rigid-body part
(`m`, `x_g`, `I_z`) is an input; the fit returns the inertia per axis (rigid body plus added mass)
and the damping. See `include/model_identification_usv/model3dof.hpp`.

## Nodes

- `id_experiment`: runs the test program and logs. Blocks, in order:
  - `surge`: equal thrust on both sides, hold until steady, release (coast-down);
  - `yaw`: opposite thrust (pure moment), hold until steady, release (spin-down);
  - `sway`: external lateral force at the centre of gravity, heading held with differential
    thrust (the moment needed gives the yaw moment due to sway), release. In Gazebo the force comes
    from the `BodyWrench` system; on the water it is a thruster rotated to push sideways, driven
    from the same topic;
  - `turn`: straight run to steady speed, constant differential thrust until the turn is steady,
    release;
  - `zigzag`: heading-triggered differential thrust at +-10 and +-20 deg (validation);
  - `multisine`: random-phase multisines on the common and the differential thrust (validation).

  Segments advance on the odometry time stamps, so the program is the same at any real-time
  factor. Output: `<output_dir>/<run_name>.csv` (one row per odometry message with the inputs then
  active and the generalized force) and `<run_name>_segments.csv`.
- `id_fit`: offline fit from one or more logs. Steady points give the damping per axis, in three
  forms (linear + quadratic, linear only, quadratic only) so the residuals show which holds; the
  transients (step responses and decays, integrated over short windows to avoid numerical
  derivatives) give the inertia per axis and the damping at low speed; the turning circles give
  `d_vr`. The model is then simulated on the validation runs, with the fit of each state in percent
  (normalised RMSE). Output: a YAML file plus one CSV per validation run.
- `BodyWrench` (gz-sim system, `libmodel_identification_usv_body_wrench.so`): applies a body-fixed
  wrench (topic `gz.msgs.Wrench`) to a link at every physics step. Built only when gz-sim 8 is found.

## Gazebo

```
colcon build --packages-select model_identification_usv
ros2 launch model_identification_usv gz_identification.launch.xml wind_speed:=0 run_name:=calm
ros2 run model_identification_usv id_fit --ros-args \
  --params-file $(ros2 pkg prefix model_identification_usv)/share/model_identification_usv/config/gz_usv.yaml \
  -p "logs:=['$HOME/model_identification_logs/calm.csv']" -p output:=$HOME/model_identification_logs/calm_fit.yaml
```

The worlds `id_wind{0,2,4,6}.sdf` hold the USV of `asv_nav` (`model://usv`) on the `gz-waves` wave
field with that wind speed (0 = flat water). `GZ_SIM_RESOURCE_PATH` must contain the `asv_nav`
models and the `asv_wave_sim` models, as for `asv_nav`. Launch arguments: `gui:=true` for the GUI,
`update_rate` (physics steps per wall-clock second, 0 = as fast as possible), `params`,
`output_dir`.

`config/gz_usv.yaml` has the rigid body of `asv_description/models/usv/model.sdf` (base link plus
thruster housings and propellers: `m = 48.4 kg`, `x_g = 0.162 m`, `I_z = 12.59 kg m^2` about the
origin) and the test levels. The `gz-waves` hydrodynamics has no added mass, so the fitted inertia
should come back close to the rigid body; it is a check of the method.

## On the vessel

Point `odom_topic`, `left_thrust_topic`, `right_thrust_topic` (`std_msgs/Float64`, thrust in N) and
`wrench_topic` (`geometry_msgs/Wrench`, body frame; the vessel side turns `force.y` into the
command of the rotated thruster) at the vessel's topics, set the rigid body and the levels in a
copy of the config, and leave out the blocks that cannot run (`sway` without a rotated thruster).
The odometry twist must be in the body frame.

## License

MIT, see `LICENSE`.
