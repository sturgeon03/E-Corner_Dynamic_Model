# Plant Differences vs `SeohanModel_ver8`

This note compares the plant used in the current workspace (`E-Corner_Dynamic_Model`)
against the plant used by:

- `C:/Users/sym89/Desktop/SeohanModel_ver8/SeohanModel/vehicle_sim/scenarios/yaw_rate_study/run_yaw_rate_study.py`

The goal is to make later checks easier. This file is plant-focused only.

## Reference Files

Current workspace:

- `vehicle_sim/models/params/vehicle_standard.yaml`
- `vehicle_sim/models/vehicle_body/vehicle_body.py`

External study repo:

- `C:/Users/sym89/Desktop/SeohanModel_ver8/SeohanModel/vehicle_sim/models/params/vehicle_standard.yaml`
- `C:/Users/sym89/Desktop/SeohanModel_ver8/SeohanModel/vehicle_sim/models/vehicle_body/vehicle_body.py`

## 1. Vehicle Parameter Differences

Geometry:

- Current workspace: `L_wheelbase = 2.97`, asymmetric `corner_offsets`
  - `FL/FR x = +1.155`
  - `RL/RR x = -1.815`
- External study repo: `L_wheelbase = 2.106`, symmetric `corner_offsets`
  - `FL/FR x = +1.053`
  - `RL/RR x = -1.053`
- `L_track = 1.634` is the same in both.

Mass and unsprung definition:

- Current workspace:
  - `vehicle_body.m = 1806.8`
  - `unsprung.m_u_front = 74.120`
  - `unsprung.m_u_rear = 54.995`
- External study repo:
  - `vehicle_body.m = 2457.3`
  - `unsprung.m_u = 65.4`

Wheel and tire:

- Current workspace:
  - `R_eff = 0.321`
  - `J_wheel_front = 1.781`
  - `J_wheel_rear = 1.781`
  - `C_alpha = 115000`
  - `trail = 0.02635`
- External study repo:
  - `R_eff = 0.327`
  - `J_wheel = 1.6`
  - `B_wheel = 0.08`
  - `C_alpha = 111850`
  - `trail = 0.025`

Steering:

- `J_cq = 0.0738` and `B_cq = 85.61` are the same in both YAML files.
- The steering-angle limits are effectively the same family, but the surrounding plant is not.

Suspension / load split:

- Current workspace has front/rear load split fields:
  - `front_load_ratio = 0.611`
  - `rear_load_ratio = 0.389`
- External study repo does not use that same front/rear split definition in its YAML.
- Suspension tuning is materially different between the two repos.

## 2. VehicleBody Implementation Differences

Current workspace `vehicle_body.py`:

- Builds `m_total = m_sprung + m_unsprung_total`
- Uses actual `corner_offsets` to derive:
  - `a = abs(FL.x)`
  - `b = abs(RL.x)`
- Integrates body x/y acceleration using `m_total`

External study repo `vehicle_body.py`:

- Does not build or use `m_total`
- Reads `a` and `b` directly from YAML keys `geometry.a`, `geometry.b`
- Because those keys are absent in the external YAML, the code falls back to:
  - `a = 1.4`
  - `b = 1.4`
- Integrates body linear acceleration using `m`

Practical implication:

- Even before controller differences, the two repos are not running the same plant.
- The current workspace uses a heavier effective body mass in x/y integration than its controller model.
- The external study repo uses a simpler mass treatment and default `a/b` handling that does not match the YAML `corner_offsets`.

## 3. What To Check If You Want To Reproduce the Plant Gap

Check these items side by side:

1. `vehicle_standard.yaml`
   - wheelbase
   - `corner_offsets`
   - mass
   - unsprung mass definition
   - tire `C_alpha`
   - tire `trail`
2. `vehicle_body.py`
   - whether `m_total` exists
   - whether x/y acceleration uses `m` or `m_total`
   - how `a` and `b` are derived
3. Any code that assumes wheel positions from `L_wheelbase / 2` and `L_track / 2`
   - this will differ from a plant that uses explicit `corner_offsets`

## 4. Why This Matters For Controller Comparison

If a controller looks better in the external study repo than in the current notebook,
that does not automatically mean the controller logic is better there.

It can also mean the controller is being evaluated on:

- a different wheelbase and CG layout
- a different mass model
- a different tire/suspension combination
- a different x/y acceleration model inside `VehicleBody`

This file is only the plant side. Controller and input-side findings should be read separately.
