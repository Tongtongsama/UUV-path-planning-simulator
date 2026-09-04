# Physics v0.4 Vertical-Plane Derivation

**Status:** Approved implementation derivation  
**Applies to:** diagonal 3-DOF surge-heave-pitch reference model  
**Last verified:** 2026-09-03

## 1. Frames and reduced variables

Core position uses ENU world coordinates (`x` East, `z` up). Body velocity
uses SNAME axes (`u` forward, `w` down, `q` nose-up). Therefore:

\[
\dot{x}=u\cos\theta+w\sin\theta,\qquad
\dot{z}=u\sin\theta-w\cos\theta,\qquad
\dot{\theta}=q.
\]

The negative `w*cos(theta)` term is required by the opposed world/body z-axis
signs.

## 2. Inertia convention

The reduced matrices are:

\[
M_{RB}=\operatorname{diag}(m,m,I_y),\qquad
M_A=\operatorname{diag}(A_u,A_w,A_q),\qquad M=M_{RB}+M_A.
\]

All canonical entries are non-negative inertia contributions. Literature
hydrodynamic derivatives use the opposite convention, hence
`A_u=-X_udot`, `A_w=-Z_wdot`, and `A_q=-M_qdot` before storage.

## 3. Rigid-body Coriolis reduction

At the centre of gravity, with translational velocity `[u,0,w]` and angular
velocity `[0,q,0]`, the linear-velocity-independent rigid-body
parametrization reduces to:

\[
C_{RB}(\nu)=
\begin{bmatrix}0&m q&0\\-m q&0&0\\0&0&0\end{bmatrix}.
\]

Thus `C_RB(nu) nu = [m*q*w, -m*q*u, 0]^T`, matching the reduced Newton-Euler
cross product `m*(omega cross v)`.

## 4. Added-mass Coriolis reduction

Reducing the general 6-DOF `m2c` construction to indices surge, heave, and
pitch gives:

\[
C_A(\nu_r)=
\begin{bmatrix}
0&0&A_w w_r\\
0&0&-A_u u_r\\
-A_w w_r&A_u u_r&0
\end{bmatrix}.
\]

Its generalized-force product is
`[A_w*w_r*q, -A_u*u_r*q, (A_u-A_w)*u_r*w_r]^T`.
Both reduced Coriolis matrices are skew-symmetric; consequently their
quadratic power terms vanish.

## 5. Uniform-current correction

For constant world current `[v_cx,v_cz]`, its body components `[u_c,w_c]`
change as the vehicle pitches:

\[
\dot{\nu}_c=[-q w_c,q u_c,0]^T.
\]

Starting from the relative-motion added-mass term
`M_A * d(nu_r)/dt`, with `nu_r = nu - nu_c`, produces the absolute-state
equation:

\[
M\dot{\nu}=B_\tau\tau-C_{RB}(\nu)\nu-C_A(\nu_r)\nu_r
-D(\nu_r)\nu_r-g(\eta)+M_A\dot{\nu}_c.
\]

Omitting the final term is valid only for zero current or zero pitch rate; it
is not the general constant-world-current model.

## 6. Evidence and implementation boundary

The reduction was checked against Fossen's published 6-DOF matrix structure
and the `m2c` construction in Python Vehicle Simulator commit
`6073346c86180087d3bca608b1aa2bc08be59708`. Files inspected:

- `src/python_vehicle_simulator/lib/gnc.py`: `m2c`;
- `src/python_vehicle_simulator/vehicles/remus100.py`: mass construction and
  Coriolis call sites.

No upstream code or parameters were copied. Project implementation uses this
independent reduction and its own ENU/SNAME contracts. Tests MUST check matrix
entries, skew symmetry, zero quadratic power, current correction, and the
expanded scalar dynamics.
