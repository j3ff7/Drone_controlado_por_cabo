# PX4 SITL + MuJoCo proof of concept

Independent experimental physics backend for the tethered UAV project. It does
not replace Gazebo, does not require ROS 2, and does not modify the local PX4
checkout. Runtime airframe files are installed into ignored
`experiments/mujoco_px4/runtime/`.

## Environment

Validated on 2026-10-05:

```text
OS                 Ubuntu 22.04.5 LTS, kernel 5.19.0-45
Python             3.10.12
MuJoCo             3.14.0 (official PyPI package)
PX4                 v1.14.4, commit 1555f2bd2229544c43966ab5f94879c41d8e1e01
ROS 2              Humble (not used by this experiment)
CMake / GCC         3.22.1 / 11.4.0
```

The bridge is derived from `WKoishi/mujoco_px4_sitl` at commit
`67630ff462c0bf84239b0315126d44415cef6a33` under BSD-3-Clause. Provenance and
license are in `vendor/mujoco_px4_sitl/`.

## Architecture

```text
MuJoCo 3.14 (ENU world, FLU body)
  HIL_SENSOR + HIL_STATE_QUATERNION, TCP 4560
                    |
                    v
PX4 v1.14.4 simulator_mavlink (strict lockstep)
                    |
                    v
  HIL_ACTUATOR_CONTROLS, four independent motors
```

The simulator publishes ideal IMU at 250 Hz and ground truth every IMU frame.
PX4's `sensor_baro_sim`, `sensor_mag_sim`, and `sensor_gps_sim` synthesize the
remaining sensors. Each physics frame waits for the PX4 actuator answer stamped
with simulation time. Physics runs at 1000 Hz (`dt=0.001 s`).

The minimal X500 uses the current Gazebo SDF values:

```text
aggregate mass       2.064307692 kg
aggregate inertia    (0.02383948, 0.02394241, 0.04399995) kg.m2
rotor xy             (+/-0.174, +/-0.174) m
spin                 CCW, CCW, CW, CW
c_t                  8.54858e-06 N/(rad/s)^2
k_m                  0.016 m
omega_max            1000 rad/s
IMU                   base_link origin, body-aligned
```

The tether uses native MuJoCo bodies and ball joints rather than the legacy
`composite` macro. This gives explicit endpoint bodies, contact geoms and
instrumentation. A fixed world endpoint anchors the chain; a MuJoCo equality
`connect` constraint joins the other endpoint to the X500 attachment site.
MuJoCo supports this loop constraint without giving a body a second parent
joint. Baseline tether: `L=2.5 m`, `rho=0.06 kg/m`, mass `0.15 kg`.

The side channel on UDP 14650 records ground truth, motor commands, endpoint
error, equality force/tension, anchor drift, and endpoint tangent. Tether angles
use the project convention in the UAV body frame:

```text
azimuth   = atan2(y, x)
elevation = atan2(-z, hypot(x, y))
```

## Setup

From the repository root:

```bash
cd /home/lima/codes/ic/drone-cabo
bash experiments/mujoco_px4/scripts/prepare.sh
```

This creates `.venv`, installs the pinned `requirements.txt`, checks the PX4
binary, and prepares an experiment-private PX4 rootfs. To run unit tests:

```bash
PYTHONPATH=experiments/mujoco_px4/bridge \
  experiments/mujoco_px4/.venv/bin/pytest -q experiments/mujoco_px4/tests
```

## Run

M0 X500 interactively, with the MuJoCo viewer:

```bash
bash experiments/mujoco_px4/scripts/run_sitl.sh --viewer
```

In another terminal, use PX4 commands through the daemon rootfs:

```bash
experiments/mujoco_px4/scripts/px4_cmd.sh commander status
experiments/mujoco_px4/scripts/px4_cmd.sh commander arm
experiments/mujoco_px4/scripts/px4_cmd.sh commander takeoff
experiments/mujoco_px4/scripts/px4_cmd.sh commander land
```

Automated, timestamped gates:

```bash
bash experiments/mujoco_px4/scripts/run_m0.sh
bash experiments/mujoco_px4/scripts/run_m1_n30.sh
bash experiments/mujoco_px4/scripts/run_m3_n70.sh
bash experiments/mujoco_px4/scripts/run_m4_n70.sh
bash experiments/mujoco_px4/scripts/run_rtf_contact_n70.sh
bash experiments/mujoco_px4/scripts/run_rtf_full_contact_sweep_n70.sh
```

Generate and test static tether discretizations:

```bash
EXP=experiments/mujoco_px4
$EXP/.venv/bin/python $EXP/scripts/generate_tether_model.py \
  --links 70 --tether-only --output $EXP/models/tether_static_n70.xml
$EXP/.venv/bin/python $EXP/scripts/test_tether_static.py \
  $EXP/models/tether_static_n70.xml --duration 5 \
  --output $EXP/results/$(date +%Y%m%d_%H%M%S)_m2_static_n70
```

Results contain `metadata.txt`, `metrics.json`, `ground_truth.csv`, PX4 command
output and SITL logs where applicable. See `results/README.md`.

## Results

| Gate | Result | Main observation |
| --- | --- | --- |
| M0 X500 | PASS | Arm/takeoff/hover/land; RTF 1.000; RMS XY/Z 0.058/0.065 m |
| M1 N=30 | PASS | Tension max 1.267 N; endpoint error max 0.50 mm; RTF 0.429 |
| M2 N=70 | PASS static | RTF 0.0470; no NaN/crash; anchor drift 0 |
| M2 N=100 | PASS static | RTF 0.0247; no NaN/crash; anchor drift 0 |
| M3 N=70 | PASS | Vertical flight; roll/pitch 1.50/0.60 deg; corrected elevation mean 81.0 deg; RTF 0.0578 |
| M4 N=70 | PASS | 0.433 m achieved for 0.5 m command; return error 0.040 m |
| RTF/contact N=70 | PASS | Descending from 2.00 to 0.66 m increased floor-contacting links from 15.6 to 53.2; contact/RTF correlation -0.812 |

Gazebo X500 baseline has RTF about 0.992 and sampled hover attitude about
0.3/0.2 deg. MuJoCo M0 has similarly stable basic flight and RTF 1.0. With a
contacting N=70 tether, MuJoCo remains numerically stable but falls to RTF
0.05-0.06; this is the dominant limitation of the present chain model.

### Floor-contact RTF diagnostic

`run_rtf_contact_n70.sh` takes off to 2 m and then commands five descending
hover plateaus while preserving the same N=70 model, 1000 Hz physics and PX4
controller. It records wall and simulation time independently, counts both
tether-floor contact points and unique tether links touching the floor, and
computes a local RTF for each settled plateau.

The validated run `20261005_131644_rtf_contact_n70` found a strong negative
correlation (-0.812) between floor-contacting links and RTF. From the highest
to the lowest plateau, mean contacting links increased from 15.6 to 53.2 and
RTF decreased from 0.111 to 0.078. The minimum intermediate RTF was 0.067.
The last plateau was slightly faster than the preceding one, so contact count
does not explain all runtime variation, but floor collision handling is a
major cost. Zero floor contact was intentionally not attempted: at 2 m, a
2.5 m anchored tether retains slack and 15-16 links on the floor.

The extended `run_rtf_full_contact_sweep_n70.sh` profile conditions the X500 at
about 2.6 m before measurement, then descends through eight settled plateaus.
Its validated `20261005_162508_rtf_full_contact_sweep_n70` run covers exactly
zero contacting links through a mean 63.8 of 70 (maximum 66). Across the full
profile, RTF/contact correlation is -0.879 and RTF reaches 0.038 near the
floor. The zero-contact point has RTF 0.102 while the next point reaches 0.155,
so the relationship is not monotonic at the nearly taut initial condition;
after contact is established, RTF consistently falls as contact grows. The
result directory includes `contact_rtf_summary.csv` and `contact_rtf.svg`.
The settled zero-contact plateau has mean tether tension 1.47 N. A 15.50 N
peak occurred briefly during the excluded takeoff/conditioning transition at
2.63 m, when the 2.5 m tether was pulled nearly taut; it is not a floor-contact
cost and must not be used as a steady-state operating point.

## Troubleshooting

- `Address already in use` on 4560/14650: stop an older `run_sitl.sh`, PX4 or
  bridge process before retrying.
- `run scripts/prepare.sh first`: create the venv/private runtime with the setup
  command above.
- Viewer errors over SSH/headless sessions: omit `--viewer`; physics and tests
  do not need OpenGL.
- PX4 never becomes ready: inspect the timestamped `sitl.log`; verify local PX4
  is exactly v1.14.4 and the private airframe is present under `runtime/etc`.
- N=70 at 500 Hz is unstable. Keep `--physics-rate 1000`; do not trade solver
  stability for wall-clock speed.
- N=70/N=100 are intentionally slow with floor contact. Use N=30 for rapid
  iteration and reserve larger chains for offline validation.

## Rover recommendation

Start with a directly actuated MuJoCo rover in the same world. It is the
simplest deterministic option and keeps the first coupled tests inside one
clock/solver. Add a thin ROS 2 command/state bridge when experiment orchestration
requires ROS. A second PX4 SITL should be deferred until the rover specifically
needs autopilot behavior; it adds another estimator, transport and clock domain
without helping the initial tether physics question.

## References

- [PX4 v1.14 simulation](https://docs.px4.io/v1.14/en/simulation/)
- [MuJoCo Python bindings](https://mujoco.readthedocs.io/en/latest/python.html)
- [MuJoCo XML reference](https://mujoco.readthedocs.io/en/latest/XMLreference.html)
- [Bridge implementation studied](https://github.com/WKoishi/mujoco_px4_sitl)
