# GFM Droop Control with EN 50549-1/2 LVRT Current Saturation (CCS)

MATLAB Function blocks for a grid-forming (GFM), droop-based converter
control chain, with a Current Command Saturation (CCS) stage inserted
between the outer voltage controller and the inner current controller
to provide Low/High Voltage Ride-Through (LVRT/HVRT) capability
compliant in *structure* with EN 50549-1 (LV networks, <=1 kV) and
EN 50549-2 (MV networks, 1-110 kV).

## Signal chain

```
                 Vdq, Idq (measured)
                        |
        +---------------+---------------+
        |                               |
        v                               v
   APC (P-f droop)                RPC (Q-V droop)
   apc_active_power_control.m     rpc_reactive_power_control.m
        |                               |
        v                               v
     w, theta                  Vd_ref, Vq_ref = 1 pu
        |                               |
        |                               v
        |                     Voltage controller (existing)
        |                     -> Id_ref, Iq_ref (unsaturated)
        |                               |
        |                               v
        |                     *** CCS (this deliverable) ***
        |                     ccs_current_limiter.m
        |                     -> Id_ref_lim, Iq_ref_lim
        |                               |
        +---------------+---------------+
                        v
              Current controller (existing)
              -> modulating voltage / PWM reference,
                 using theta from APC for dq<->abc transforms
```

`theta` from the APC is the common angle used to build every dq frame
in the chain (voltage measurement, RPC, voltage controller, CCS,
current controller) - it is the "virtual rotor angle" of the GFM unit.

## Files

| File | Role |
|---|---|
| `apc_active_power_control.m` | P-f droop -> `w`, `theta` |
| `rpc_reactive_power_control.m` | Q-V droop -> `Vd_ref`, `Vq_ref` (fixed 1 pu) |
| `ccs_current_limiter.m` | **CCS**: saturates the voltage controller's `Id_ref`/`Iq_ref` with reactive-current-priority LVRT/HVRT support and an FRT boundary flag |

## Using the CCS block in Simulink

1. Drop a **MATLAB Function** block between the voltage controller and
   the current controller.
2. Paste the contents of `ccs_current_limiter.m` into it (or point the
   block at the file).
3. Wire the ports:
   - `Id_ref`, `Iq_ref`   <- voltage controller outputs
   - `Vd_meas`, `Vq_meas` <- measured terminal/PCC voltage, dq (same
     frame as the rest of the control, i.e. built from APC's `theta`)
   - `t`                  <- a `Clock` block (simulation time)
   - `Id_ref_lim`, `Iq_ref_lim` -> current controller inputs
   - `lvrt_active`, `must_stay_connected`, `Vpu` -> scope/logic as
     needed (e.g. drive a protection state machine or a scope to
     verify the ride-through behavior)
4. Set the block's sample time to match `Ts`, and set it to run at a
   **discrete** sample time consistent with the rest of the control
   (the block uses `persistent` state for the fast-engage/ramped-exit
   behavior, so it must execute at a fixed `Ts`).

### Suggested starting parameters

| Parameter | Meaning | Typical starting value |
|---|---|---|
| `Imax` | Converter current limit | 1.0 - 1.2 pu |
| `Vdb_lo` / `Vdb_hi` | Voltage dead-band edges | 0.90 / 1.10 pu |
| `k_lvrt` | Reactive current support gain, dIq/dV | 2.0 pu/pu |
| `Tramp` | Ramp-out time after voltage recovers | 0.2 - 1.0 s |
| `Ts` | Block sample time | match plant discretization, e.g. 1e-4 - 2e-4 s |

**Important:** EN 50549-1 and EN 50549-2 define the *type* of
requirement (voltage dead-band, proportional reactive-current support
gain `k`, and a minimum voltage/time ride-through corridor) but leave
the exact numeric values to each country's **National Annex (NA)** /
the connecting DSO's grid connection agreement. The values above, and
the FRT boundary knee-points hardcoded inside `ccs_current_limiter.m`
(`V1=0.15 pu @ t1=0.15 s`, `V2=0.85 pu @ t2=1.5 s`), are common
illustrative defaults and **must be replaced with the values from the
applicable NA** before the model is used for compliance work.

## Notes on `itd` / `vq` inputs mentioned alongside `idq`, `vdq`

If your model also carries the raw inverter-side inductor current
(`itdq`, i.e. `itd`/`itq`) separately from the output/grid-side current
`idq` (common in an LCL-filtered GFM converter, where the current
controller regulates `itdq` while the CCS/voltage loop reasons about
the grid-side/output current), feed the CCS block with whichever
current is used by your voltage-loop's `Id_ref`/`Iq_ref` derivation,
and keep the current controller's own inner feedback on `itdq` as
usual - the CCS only touches the **reference**, not the inner-loop
feedback signal.
