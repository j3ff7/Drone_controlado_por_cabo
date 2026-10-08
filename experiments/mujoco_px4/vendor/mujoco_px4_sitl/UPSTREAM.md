# Vendored bridge provenance

- Upstream: `https://github.com/WKoishi/mujoco_px4_sitl`
- Commit: `67630ff462c0bf84239b0315126d44415cef6a33`
- License: BSD-3-Clause (see `LICENSE`)
- Local scope: `bridge/mujoco_px4_sitl/`

The bridge was vendored to keep this proof-of-concept reproducible and isolated.
The local runtime uses PX4 v1.14.4 and an experiment-private airframe/rootfs;
the upstream project targets a newer PX4 release. Compatibility is verified by
the tests and recorded M0 execution, not assumed from the upstream README.
