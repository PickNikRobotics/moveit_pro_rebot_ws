# Boston static gravity fit

The six inertial-origin vectors in `config/gravity/identified_first_moments.yaml`
are retained from the integrated source
`7d7761965e8082d169be39817ab3545c924e446c`. They are a **gravity-equivalent model**
for this empty-jaw, camera/cable assembly, not separately identified physical
centers of mass, link masses or a dynamic mass matrix. In particular link4's
x≈0.257m origin lies outside its physical link.

The 2026-11-01 source fit used24poses,48opposing-approach windows of100samples
following1.5ssettling. Every fifth pose (5,10,15,20) was held out before fitting:
20training,4held-out. Static rank10, training condition29.39. Nominal masses and
rotational inertia tensors were retained. Observable first-moment combinations
were fitted; unobservable components remained nominal. Do not use this model
for MuJoCo or other dynamic computations.

Held-out measured-torque RMSE, Nm:

| Joint | Nominal | Fitted |
| --- | ---: | ---: |
| J1 | 0.0732 | 0.0732 |
| J2 | 1.0313 | 0.1820 |
| J3 | 1.5250 | 0.4161 |
| J4 | 0.5031 | 0.0813 |
| J5 | 0.0881 | 0.0963 |
| J6 | 0.0796 | 0.0791 |

These are historical source results, not a new measurement during migration.
J1's model gravity is zero; measured bias/friction is not a gravity parameter.
J2–J4 improve, J5 gets slightly worse and J6 is essentially unchanged. Friction,
hysteresis and torque-report bias remain. OLS covariance assumes independent,
equal-variance residuals and does not bound those effects.

The source controller configuration uses gains `[0,1,1,1,0,0]` and zero payload.
The effective model may already include jaw, camera and cables; do not add the
stock-finger payload again. Historical moving gains `[0,.25,1.25,1.5,0,0]` are
not part of the fit and must not be reapplied as if the model were nominal.
The later48single-approach collection has different data semantics and has not
been refitted into these vectors. A collision-free pose or increased configured
J2 feedforward cap does not establish continuous thermal/load capability.

No raw recordings, unit telemetry, images or generated fit artifacts are shipped.
Only the preserved source values and offline tooling are migrated. Any future
collection, refit or model adoption is separately authorized work.
