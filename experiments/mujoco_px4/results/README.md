# MuJoCo experiment results

Results are timestamped and never overwritten. Large CSV and SITL logs remain
local; this index preserves the reproducible headline results.

| Run | Configuration | Result | Key metrics |
| --- | --- | --- | --- |
| `20261005_110100_m0_x500` | X500, no tether | PASS | RTF 1.000; RMS XY/Z 0.058/0.065 m; roll/pitch 1.42/0.46 deg |
| `20261005_110805_m1_static_n30` | tether-only, N=30 | PASS | RTF 0.617; anchor drift 0; no NaN |
| `20261005_111741_m1_tether_n30` | X500 + anchored tether, N=30 | PASS | RTF 0.429; tension max 1.267 N; connection error max 0.50 mm |
| `20261005_111952_m2_static_n70` | tether-only, N=70 | PASS | RTF 0.0470; anchor drift 0; no NaN |
| `20261005_112138_m2_static_n100` | tether-only, N=100 | PASS | RTF 0.0247; anchor drift 0; no NaN |
| `20261005_112924_m3_vertical_n70` | vertical flight, N=70 | PASS | RTF 0.0578; tension max 0.544 N; roll/pitch 1.50/0.60 deg |
| `20261005_114210_m4_horizontal_n70` | dx=0.5 m and return, N=70 | PASS | achieved 0.433 m; target/return error 0.069/0.040 m; RTF 0.0623 |
| `20261005_131644_rtf_contact_n70` | staged descent, N=70 | PASS | 15.6 to 53.2 links contacting floor; RTF 0.111 to 0.078; correlation -0.812 |
| `20261005_162508_rtf_full_contact_sweep_n70` | airborne-to-floor sweep, N=70 | PASS | 0 to 63.8 mean contacting links; RTF 0.102 to 0.038; correlation -0.879 |

Diagnostic failures retained locally:

- `20261005_105855_m0_x500`: evaluator expected the wrong PX4 arm log string.
- `20261005_110646_m1_static_n30`: false anchor drift because the site position
  was sampled before the initial `mj_forward`; the physics itself stayed finite.
- `20261005_111337_m1_tether_n30`: `connect.anchor` was incorrectly supplied
  in world coordinates instead of the last tether body's local frame. The
  corrected run reduced endpoint error from 166.7 mm to 0.50 mm.
- A non-persistent N=70 isolation probe at 500 Hz became unstable at 0.022 s.
  N=70 flight therefore uses the validated 1000 Hz physics rate.
- `20261005_130955_rtf_contact_n70` ended after takeoff because PX4 exited
  before the first measurement plateau. It produced no valid profile and was
  repeated without changing the model or parameters.

The M3 CSV was recorded before correcting the sign of MuJoCo capsule's
`fromto` longitudinal axis. Corrected offline, its settled tether elevation is
81.0 deg on average (74.6-88.7 deg). Current code and the M4 CSV use the fixed
sign. Azimuth is poorly conditioned in these near-vertical configurations.
