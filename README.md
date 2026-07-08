# CARLA–Simulink Co-Simulation Bridge
## Autonomous CAT 797F Mining Dump Truck — Lateral Control

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [File Reference](#2-file-reference)
3. [Python Bridge — `carla_bridge.py`](#3-python-bridge--carla_bridgepy)
4. [MATLAB System Block — `CarlaSimulinkBridge.m`](#4-matlab-system-block--carlasimulinkbridgem)
5. [Differential Flatness Control Law](#5-differential-flatness-control-law)
6. [Controller Gains — Selection Rationale](#6-controller-gains--selection-rationale)
7. [Sample Rate and Timing](#7-sample-rate-and-timing)
8. [Coordinate Systems and Sign Conventions](#8-coordinate-systems-and-sign-conventions)
9. [Simulink Model Architecture](#9-simulink-model-architecture)
10. [Data Dictionary — `CAT_797F.sldd`](#10-data-dictionary--cat_797fsldd)
11. [Reference Path Generation](#11-reference-path-generation)
12. [Data Logging](#12-data-logging)
13. [Known Limitations and Design Notes](#13-known-limitations-and-design-notes)

---

## 1. Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────┐
│  MATLAB / Simulink (R2025a)                                          │
│                                                                      │
│  ┌─────────────────┐   steer, throttle, brake                        │
│  │ Flatness        │──────────────────────────────────────────────┐  │
│  │ Controller      │                                              │  │
│  │ (flatness_ctrl) │◄─── x, y, ψ, vx, Xd, Yd, ψd, ψ̇d, ψ̈d,         │  │
│  └─────────────────┘     ψ̇, Ẏ  (11 signals)                       │  │
│           ▲                                                       │  │
│           │                                                       ▼  │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │  CarlaSimulinkBridge.m  (MATLAB System Block)                   │ │
│  │  Calls Python CarlaBridge.step() via pyrun / py interface       │ │
│  └──────────────────────────────┬──────────────────────────────────┘ │
└─────────────────────────────────│────────────────────────────────────┘
                                  │ Python 3.12 (in-process)
                                  ▼
┌──────────────────────────────────────────────────────────────────────┐
│  carla_bridge.py — CarlaBridge class                                 │
│  • Connects to CARLA server (localhost:2000)                         │
│  • Spawns CAT 797F mining truck on Mine_01 map                       │
│  • Ticks CARLA world (synchronous mode)                              │
│  • Computes reference path and look-ahead tracking                   │
│  • Returns 11 state signals per step                                 │
└────────────────────────────┬─────────────────────────────────────────┘
                             │ CARLA API (TCP)
                             ▼
┌──────────────────────────────────────────────────────────────────────┐
│  CARLA 0.10.x — Mine_01 map                                           │
│  CAT 797F mining truck physics                                       │
└──────────────────────────────────────────────────────────────────────┘
```

**Execution flow per simulation tick:**

1. Simulink solver fires the `CarlaSimulinkBridge` System block at `t = k·Ts`.
2. The block calls `CarlaBridge.step(steer, throttle, brake)`.
3. Python applies the control to the CARLA vehicle, calls `world.tick()` (advancing physics by `Ts`), then reads back the new vehicle state.
4. The 11 state signals are returned to Simulink.
5. The flatness controller computes the next control commands, which are delivered on tick `k+1`.

---

## 2. File Reference

| File | Location | Purpose |
|------|----------|---------|
| `carla_bridge.py` | `src/plant_model/`, `src/sim/`, root | Python CARLA interface (keep all three in sync) |
| `CarlaSimulinkBridge.m` | `src/plant_model/`, `src/sim/` | MATLAB System block wrapping the Python bridge |
| `CAT797_lateral_dynamics_differential_flatness_control_control_loop.slx` | `src/plant_model/` | Main Simulink model |
| `CAT_797F.sldd` | `src/plant_model/` | Simulink Data Dictionary — all tunable parameters |
| `bridge_log.csv` | root | Per-tick telemetry log written on simulation stop |
| `steering_characterization.csv` | root | Steering characterisation test output |
| `src/sim/characterize_steering.py` | `src/sim/` | Standalone CARLA steering characterisation script |

> **File synchronisation**: `carla_bridge.py` exists in three locations. Always edit `src/plant_model/carla_bridge.py` (canonical), then copy to the other two locations before running, or use the sync command:
> ```powershell
> $src = "G:\Control_Research\Autonomy_Mining_Dump_Truck\src\plant_model\carla_bridge.py"
> Copy-Item $src "G:\Control_Research\Autonomy_Mining_Dump_Truck\carla_bridge.py" -Force
> Copy-Item $src "G:\Control_Research\Autonomy_Mining_Dump_Truck\src\sim\carla_bridge.py" -Force
> ```

---

## 3. Python Bridge — `carla_bridge.py`

### 3.1 Class: `CarlaBridge`

#### Constructor `__init__()`

| Action | Detail |
|--------|--------|
| Connect to CARLA server | `localhost:2000`, 30 s timeout |
| Load world | `/Game/Carla/Maps/Mine_01` |
| Set synchronous mode | `fixed_delta_seconds = 0.01` s (100 Hz) |
| Destroy existing vehicles | Prevents duplicate actors across runs |
| Spawn mining truck | Blueprint: `vehicle.miningtruck*`; spawn on road waypoint near `x=−11.57, y=95.74` |
| No yaw flip | Vehicle faces road travel direction (yaw ≈ 181°, heading west-southwest) |
| Zero lateral offset | `offset = 0` → spawn exactly on road centreline, initial `y = 0` |
| Build reference path | 500 m straight line from spawn in spawn-heading direction, 2 m spacing |

**Persistent state initialised in constructor:**

```python
self._psi0        = None   # spawn heading captured at first step() call
self._psi_unwrap  = None   # continuous (unwrapped) yaw for error computation
self._psi_d_unwrap= None   # continuous reference heading
self._ref_x0      = None   # reference origin (first step), so Xd(0)=Yd(0)=0
self._ref_y0      = None
self._Xd_frozen   = None   # frozen Xd/Yd after lane-change completes
self._Yd_frozen   = None
self._fp_idx      = 0      # closest-point search start index (forward-only)
self.sim_time     = 0.0    # local counter (not CARLA server uptime)
self._dt          = 0.01   # must match fixed_delta_seconds
```

#### Method `step(steer, throttle, brake)`

Inputs are the three CARLA `VehicleControl` scalars (all floats, `steer ∈ [−1, 1]`).

Returns a Python list of **11 floats** (in order):

| Index | Signal | Units | Description |
|-------|--------|-------|-------------|
| 0 | `x` | m | Longitudinal position (origin = spawn) |
| 1 | `y` | m | Lateral position (origin = spawn) |
| 2 | `psi_rel` | rad | Heading relative to spawn heading |
| 3 | `vx_safe` | m/s | Longitudinal speed, clamped to `VX_MIN = 3.0` |
| 4 | `Xd` | m | Reference longitudinal position |
| 5 | `Yd` | m | Reference lateral position |
| 6 | `psi_d_rel` | rad | Reference heading (relative to spawn) |
| 7 | `psi_dot_d` | rad/s | Reference yaw rate |
| 8 | `psi_dd_d` | rad/s² | Reference yaw acceleration |
| 9 | `psi_dot` | rad/s | Measured yaw rate (`omega.z` in CARLA) |
| 10 | `Y_dot` | m/s | Global lateral velocity (`vel.y` in CARLA) |

**Key implementation details:**

- **`vx_safe` / `VX_MIN = 3.0`**: The A and B feedforward terms in the control law
  contain `1/vx`. Clamping at 3.0 m/s prevents division-by-zero **and** is a
  necessary constraint — see [Section 6.3](#63-a-feedforward-stability-constraint).

- **Yaw unwrapping**: Raw CARLA `rotation.yaw` wraps at ±180°. The bridge
  integrates the angular increment each step to produce a continuous `psi` signal,
  essential for heading error terms that cross the ±π boundary.

- **Relative heading (`psi_rel`)**: `psi0` (spawn heading) is subtracted so that
  `psi_rel(0) = 0` and the `vx·ψ` small-angle approximation is valid regardless
  of the absolute road direction.

- **`sim_time`**: Incremented locally by `_dt` each step. Independent of CARLA
  server uptime to ensure reference signals always start at `t = 0`.

#### Method `_lane_change_reference()`

Implements a closest-point + look-ahead (20 m) tracker on `full_path_pts`.

Reference lateral offset uses a smoothstep profile:

```
d_lat(τ) = yf · (3τ² − 2τ³),  τ = (t − t_start) / T
```

Parameters (all tuneable in source):

| Parameter | Value | Description |
|-----------|-------|-------------|
| `yf` | `0.0` (lane-change: `6.0`) | Lateral offset target [m] |
| `t_start` | `15.0` | Lane-change start time [s] |
| `T` | `10.0` | Lane-change duration [s] |
| `LOOK_AHEAD` | `20.0` | Look-ahead distance [m] |

> **Current mode**: `yf = 0.0` → straight-line tracking. Set `yf = 6.0` to
> enable a 6 m lane change.

**Yd drift fix**: After the lane change completes (`t > t_start + T`), `Xd` and
`Yd` are frozen to the value at completion. This prevents the reference from
drifting as the look-ahead point advances along a path with non-zero y-component.

#### Method `close()`

Destroys the vehicle actor and saves the telemetry log. Uses a `.tmp` file +
atomic `os.replace()` to prevent CSV header corruption from MATLAB stdout
capture.

---

## 4. MATLAB System Block — `CarlaSimulinkBridge.m`

### 4.1 Class hierarchy

```
matlab.System
└── CarlaSimulinkBridge
    Properties (Nontunable): PythonFolder
    Properties (Private):    bridge  (Python CarlaBridge object)
```

### 4.2 Key method overrides

#### `setupImpl()`

Loads `carla_bridge.py` by **absolute path** using `importlib.util.spec_from_file_location`:

```matlab
pyrun("import importlib.util, sys; " + ...
      "sys.modules.pop('carla_bridge', None); " + ...   % purge stale cache
      "spec = importlib.util.spec_from_file_location('carla_bridge', '<path>'); " + ...
      "m = importlib.util.module_from_spec(spec); " + ...
      "sys.modules['carla_bridge'] = m; " + ...
      "spec.loader.exec_module(m)");
```

This bypasses `sys.path` and stale `__pycache__` bytecode — critical because
MATLAB's Python runtime does not reload modules between simulation runs unless
explicitly flushed.

> **Always run `clear classes` in MATLAB before restarting the simulation**
> after any change to `carla_bridge.py`.

#### `stepImpl()`

Calls `bridge.step()`, receives the Python list, and converts via
`double(py.array.array('d', out))`. The 11 outputs are unpacked in order.

#### `isInputDirectFeedthroughImpl()`

Returns `false`. Marks that the output at time `k` does not depend on the
input at time `k` (the bridge applies `steer[k]`, ticks CARLA, and returns
`state[k+1]`). This **breaks the algebraic loop** that would otherwise form
between the controller and the bridge.

#### `getSampleTimeImpl()`

```matlab
sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', 0.01);
```

Locks the System block to the same `0.01 s` tick as CARLA. Without this,
Simulink's default solver could call `step()` at a different rate, causing the
CARLA world to advance out of step with the controller.

#### `releaseImpl()`

Calls `bridge.close()`, which destroys the CARLA vehicle and writes
`bridge_log.csv`.

---

## 5. Differential Flatness Control Law

The lateral dynamics of the vehicle are modelled by the linear bicycle model
with the flat output `Y` (global lateral position):

$$\ddot{\psi} = A(v_x)\dot{\psi} + B(v_x)(\dot{Y} - v_x \psi) + C \delta_f$$

where

$$A(v_x) = -\frac{C_f L_f^2 + C_r L_r^2}{I_z v_x}, \qquad
  B(v_x) = -\frac{C_f L_f - C_r L_r}{I_z v_x}, \qquad
  C = \frac{C_f L_f}{I_z}$$

The control law implements **feedback linearisation** plus outer-loop state
feedback:


```math
\delta_f =
\frac{1}{C}
\left(
\ddot{\psi}_d
-k_1(\dot{\psi}-\dot{\psi}_d)
-k_0(\psi-\psi_d)
-k_y(Y-Y_d)
-k_i\eta
-A\dot{\psi}
-B(\dot{Y}-v_x\psi)
\right)
```

where

```math
e_\psi = \psi-\psi_d
```

```math
e_y = Y-Y_d
```

```math
\eta = \int e_y\,dt
```
- v_x\psi)
\Bigr)$$

| Symbol | Description |
|--------|-------------|
| $\delta_f$ | Front wheel steering angle command [rad] (passed to CARLA as normalised steer after gain `N`) |
| $\psi_d, \dot{\psi}_d, \ddot{\psi}_d$ | Reference heading, yaw rate, yaw acceleration (from look-ahead tracker) |
| $Y_d$ | Reference lateral position [m] |
| $\eta = \int (Y - Y_d)\,dt$ | Integral of lateral error |
| $k_0, k_1, k_y, k_i$ | State feedback gains (in `CAT_797F.sldd`) |
| $A\dot\psi$ | Feedforward: cancels natural yaw damping |
| $B(\dot Y - v_x\psi)$ | Feedforward: cancels lateral-yaw coupling |

**Steer sign convention**:

```
steer_CARLA = −N · δf
```

CARLA's steer input is **normalised** (`+1 = full right`). The minus sign
converts from the paper convention (positive `δf` = left turn) to CARLA
convention (positive steer = right turn). The gain `N` (normalization) maps
the computed angle in radians to the normalised CARLA range.

---

## 6. Controller Gains — Selection Rationale

### 6.1 Vehicle parameters

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Front cornering stiffness (per axle) | $C_f$ | 5.047 × 10⁶ N/rad | Model (see §6.4) |
| Rear cornering stiffness (per axle) | $C_r$ | 5.047 × 10⁶ N/rad | Symmetric |
| Yaw inertia | $I_z$ | 7.19 × 10⁶ kg·m² | Box approximation for 280 t truck |
| Mass | $m$ | 280 000 kg | Empty-weight CAT 797F |
| CG to front axle | $L_f$ | 2.5 m | |
| CG to rear axle | $L_r$ | 2.3 m | |
| Wheelbase | $L$ | 4.8 m | |
| Control gain | $C = C_f L_f / I_z$ | 1.755 rad/s² per rad | Computed |

### 6.2 Steering normalisation gain `N`

The CARLA vehicle model's maximum physical wheel angle is approximately
**0.09–0.13 rad (5–7.5°)** at `steer_CARLA = 1.0` (characterised using
`src/sim/characterize_steering.py`). The feedback-linearisation control law
computes `δf` in radians assuming a direct mapping, so a normalisation factor
is needed:

```
steer_CARLA = −N · δf
```

The effective control authority ratio is:

$$\frac{C_{eff} \cdot N}{C_{model}} \approx \frac{0.17 \times N}{1.0}$$

where `C_eff ≈ 0.17 · C_model` was measured empirically. With `N = 2`, the
feedforward cancels approximately 34% of the plant, with the vehicle's natural
yaw damping providing the remainder. This avoids the **over-cancellation**
problem (N = 10 resulted in 170% of the plant being cancelled, which fights
the k₁ feedback).

> **Current value: N = 2** (Simulink Gain block after `flatness_ctrl_00`).

**Saturation limits with N = 2, k₁ = 5:**

| Term | Saturates steer at |
|------|--------------------|
| k₁ · ψ̇ | ψ̇ > 9.0°/s (0.157 rad/s) |
| k₀ · ψ | ψ > 22.6° (0.395 rad) |
| kᵧ · y | y > 10.5 m |

### 6.3 A-feedforward stability constraint

The A feedforward term in the control law has magnitude:

$$|A(v_x)| = \frac{C_f(L_f^2 + L_r^2)}{I_z \cdot v_x} = \frac{8.10}{v_x}$$

For the **net yaw-rate feedback to remain negative** (stable), `k₁` must
satisfy:

$$k_1 > |A(v_{x,\min})| = \frac{8.10}{v_{x,\min}}$$

With `VX_MIN = 3.0` (the clamping floor in the bridge):

$$k_1 > \frac{8.10}{3.0} = 2.70$$

**Current k₁ = 5.0 satisfies this with a 1.85× safety margin.**

> This is why reducing k₁ below 2.7 causes divergence: the A feedforward
> overcomes the k₁ damping, creating positive feedback on ψ̇.

### 6.4 Closed-loop stability — Routh–Hurwitz criterion

The lateral closed-loop (after linearisation) is approximately 3rd order in `y`:

$$s^3 + k_1 s^2 + k_0 s + v_x k_y = 0$$

**Routh stability condition:** $k_0 k_1 > v_x k_y$

With `k₀ = 2.0`, `k₁ = 5.0`, `kᵧ = 0.05`:

| vx (m/s) | k₀·k₁ | vx·kᵧ | Status |
|----------|--------|--------|--------|
| 3 | 10.0 | 0.15 | ✓ stable (67×) |
| 5 | 10.0 | 0.25 | ✓ stable (40×) |
| 8 | 10.0 | 0.40 | ✓ stable (25×) |

### 6.5 Current gain values in `CAT_797F.sldd`

| Gain | Value | Role |
|------|-------|------|
| `k0` | 2.0 | Heading proportional — large value avoids complex poles |
| `k1` | 5.0 | Yaw-rate damping — **must be > 2.70** (see §6.3) |
| `ky` | 0.05 | Lateral position — small to avoid Routh violation at high speed |
| `ki` | 0.02 | Integral correction — slow, prevents windup at road camber disturbance |

**Closed-loop poles with N = 2 (effective gains ≈ 0.34× of above) at vx = 3 m/s:**

```
s ≈ −4.19,   −0.105 ± j·0.065   (ζ = 0.85 — well-damped, period ≈ 97 s)
```

No oscillation visible in practice; the dominant mode decays smoothly.

---

## 7. Sample Rate and Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| CARLA `fixed_delta_seconds` | **0.01 s** (100 Hz) | Set in `carla_bridge.py` constructor |
| Simulink sample time | **0.01 s** | `getSampleTimeImpl` in `CarlaSimulinkBridge.m` |
| Simulink solver step | **0.01 s** | Must match in Configuration Parameters → Solver |
| `VX_MIN` clamp | **3.0 m/s** | Lower bound on `vx_safe` for A, B terms |
| Lateral enable threshold | **4.0 m/s** | Relay hysteresis block (on=4.5, off=3.5 m/s) |

**Why 100 Hz (0.01 s)?**

The oscillation timescale of interest is ~seconds (poles at ±0.1 rad/s).
100 Hz gives 100× oversampling, improving measurement accuracy of `psi_dot`
(from `omega.z`) and reducing zero-order-hold phase lag.

> Performance note: CARLA synchronous mode at 100 Hz runs slower than real
> time on typical hardware. For longer runs, consider reverting to 0.05 s
> (20 Hz) — the closed-loop dynamics at the gain values above are not affected
> by this change, and the previous 0.05 s results showed comparable or better
> tracking quality.

---

## 8. Coordinate Systems and Sign Conventions

### 8.1 CARLA world frame (Unreal Engine, left-handed)

| Axis | Positive direction |
|------|--------------------|
| X | East (approximately) |
| Y | South (right when facing east) |
| Z | Up |
| Yaw | Clockwise from +X when viewed from above |

In Mine_01: the spawn waypoint is on Road 0 near `(x=−12, y=96)`, heading
approximately **west** (yaw ≈ 181°).

### 8.2 Bridge output signals

All position signals are **zeroed at spawn**:

```python
x = x_raw - origin_x      # longitudinal (positive: west of spawn)
y = y_raw - origin_y      # lateral      (positive: south of spawn in Mine_01)
psi_rel = psi - psi0      # heading      (positive: clockwise turn from spawn heading)
```

With the road heading west:
- **y < 0** → vehicle is north of centreline
- **y > 0** → vehicle is south of centreline
- Positive steer_CARLA → right turn → vehicle moves south (+y)

### 8.3 Steer sign chain

```
δf (rad, paper convention: + = left)
  → steer_CARLA = −N · δf   (sign flip: left in paper = negative in CARLA)
  → VehicleControl.steer    (CARLA: + = right)
```

---

## 9. Simulink Model Architecture

**Model:** `CAT797_lateral_dynamics_differential_flatness_control_control_loop.slx`

```
CarlaSimulinkBridge (System block)
├── Outputs: x, y, ψ, vx, Xd, Yd, ψd, ψ̇d, ψ̈d, ψ̇, Ẏ  (11 ports)
│
├── Speed controller: Discrete PID (anti-windup)
│   └── throttle/brake → bridge
│
└── Lateral controller: flatness_ctrl_00 subsystem
    ├── Inputs: ψ, vx, Xd, Yd, ψd, ψ̇d, ψ̈d, ψ̇, Ẏ
    ├── Gains: k0, k1, ky, ki  (from CAT_797F.sldd)
    ├── A, B terms: computed from Cf, Cr, Lf, Lr, Iz, vx
    ├── Output: δf [rad]
    │
    ├── Normalization Gain block:  δf → N · δf  (N = 2)
    ├── Rate Limiter: ±3.0 rad/s  (limits rate of change of N·δf)
    ├── Saturation: ±0.9          (CARLA steer limit)
    └── Sign negate: steer_CARLA = −(N · δf)  → bridge
```

**Lateral controller enable:** A Relay block with hysteresis enables
`flatness_ctrl_00` only when `vx ≥ 4.0 m/s` (on) / `vx ≤ 3.5 m/s` (off).
This prevents premature activation during the initial acceleration phase.

---

## 10. Data Dictionary — `CAT_797F.sldd`

All parameters are defined in `src/plant_model/CAT_797F.sldd`. Edit via:
**MATLAB → Model Explorer → CAT_797F.sldd → Design Data**.

| Variable | Value | Unit | Description |
|----------|-------|------|-------------|
| `Cf` | 5.047e6 | N/rad | Front axle cornering stiffness |
| `Cr` | 5.047e6 | N/rad | Rear axle cornering stiffness |
| `Lf` | 2.5 | m | CG to front axle |
| `Lr` | 2.3 | m | CG to rear axle |
| `Iz` | 7.19e6 | kg·m² | Yaw moment of inertia |
| `Machine_Mass` | 280 | tonne | Empty vehicle mass |
| `k0` | 2.0 | — | Heading proportional gain |
| `k1` | 5.0 | — | Yaw-rate damping gain |
| `ky` | 0.05 | — | Lateral position gain |
| `ki` | 0.02 | — | Lateral integral gain |

---

## 11. Reference Path Generation

The reference path is built once in `CarlaBridge.__init__()` as a **straight
line** from the spawn point:

```python
LINE_LEN  = 500.0  # m
LINE_STEP =   2.0  # m

for i in range(n_pts):
    s = i * LINE_STEP
    full_path_pts.append((
        sx + s * cos(syaw),
        sy + s * sin(syaw),
        sz
    ))
```

The look-ahead tracker (`_lane_change_reference`) searches forward from the
previous closest index (`_fp_idx`, monotonically increasing) to find the
closest point, then returns the point `LOOK_AHEAD = 20 m` ahead as the
reference `(Xd, Yd)`.

**Lane-change mode** (set `yf = 6.0` in `_lane_change_reference()`): adds a
perpendicular offset `d_lat(t)` to the look-ahead point using a cubic
smoothstep profile starting at `t_start = 15 s` over duration `T = 10 s`.
After the lane change completes, `Xd/Yd` are frozen to prevent drift caused
by the path's slight non-zero y-component as the look-ahead advances.

---

## 12. Data Logging

Every call to `step()` appends one row to `self._log`. On `close()`, the log
is written atomically to `bridge_log.csv`:

| Column | Description |
|--------|-------------|
| `t` | Simulation time [s] |
| `steer_in` | CARLA steer command sent this tick |
| `throttle`, `brake` | Speed controller outputs |
| `x`, `y` | Vehicle position (origin-zeroed) |
| `psi` | `psi_rel` — heading relative to spawn |
| `vx` | `vx_safe` (clamped) |
| `Xd`, `Yd` | Reference position |
| `psi_d` | `psi_d_rel` — reference heading relative to spawn |
| `psi_dot_d`, `psi_dd_d` | Reference yaw rate and acceleration |
| `psi_dot` | Measured yaw rate |
| `Y_dot` | Measured global lateral velocity |
| `psi_abs` | Absolute (unwrapped) yaw [debug] |
| `vx_raw` | Unclamped longitudinal speed [debug] |

**Quick analysis** of a log file (PowerShell):
```powershell
python -c "
import csv, math
rows = []; 
with open('bridge_log.csv') as f: rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(f)]
late=[r for r in rows if r['t']>rows[-1]['t']*0.5 and abs(r['vx_raw'])>2]
print('y mean=%.3f  std=%.3f m' % (sum(r['y'] for r in late)/len(late), math.sqrt(sum((r['y']-sum(r['y'] for r in late)/len(late))**2 for r in late)/len(late))))
"
```

---

## 13. Known Limitations and Design Notes

### 13.1 VX_MIN clamping

`VX_MIN = 3.0` in `carla_bridge.py` sets the floor for `vx_safe`. This is
**not** just a numerical safeguard — it enforces the constraint from §6.3
(`k₁ > |A(vx_min)|`). Lowering `VX_MIN` below the speed where `|A| ≥ k₁`
will cause the closed-loop to destabilise.

### 13.2 Parameter mismatch

The model parameters `Cf`, `Cr`, `Iz` describe the CARLA vehicle's behaviour
only approximately. Steering characterisation (`characterize_steering.py`)
measured an effective gain `C_eff ≈ 0.17 · C_model`, meaning the actual
vehicle responds at ~17% of the modelled rate per unit steer. The normalisation
gain `N` was designed to compensate.

**Consequence**: the A and B feedforward terms computed from `Cf` are also
mismatched. With `N = 2`, the feedforward provides partial (≈34%) cancellation;
the remainder is stabilised by the vehicle's natural yaw damping and the state
feedback gains.

### 13.3 Road camber disturbance

Mine_01 roads have lateral slopes (camber) that cause a persistent northward
drift (y → negative). The small integral gain `ki = 0.02` slowly eliminates
this without causing integral windup. The Simulink PID integrator
upper/lower limits should be set to at least `±5` to allow sufficient
accumulation.

### 13.4 Speed during lateral control

Large steer commands cause tire side-loads that decelerate the vehicle. With
`N = 2`, steer commands are gentler and the speed controller (targeting
4.5 m/s) can maintain forward motion. With higher `N`, the vehicle can slow
to near-zero, causing the feedback-linearisation to break down (the kinematic
`Ẏ = vx·ψ` approximation fails and `VX_MIN` clamping introduces model error).

### 13.5 Module caching (MATLAB/Python)

MATLAB caches Python modules. After **any** change to `carla_bridge.py`,
always run in MATLAB:
```matlab
clear classes
```
before restarting the simulation. Failure to do so will run the old version.

### 13.6 CSV header corruption

The `close()` method uses an atomic temp-file rename (`os.replace`) to prevent
MATLAB's stdout capture from prepending text to the CSV header. If the log is
still corrupted, the analysis scripts handle it with:
```python
if 'csvt,' in h: h = 't,' + h[h.find('csvt,')+5:]
```

---

*Last updated: 2026-07-07*
