function [Id_ref_lim, Iq_ref_lim, lvrt_active, must_stay_connected, Vpu] = ...
    ccs_current_limiter(Id_ref, Iq_ref, Vd_meas, Vq_meas, t, ...
                         Imax, Vdb_lo, Vdb_hi, k_lvrt, Tramp, Ts)
%#codegen
% CCS_CURRENT_LIMITER  Current Command Saturation (CCS) stage for a
% grid-forming (GFM) droop-controlled converter, with Low/High Voltage
% Ride-Through (LVRT/HVRT) support per EN 50549-1 (LV networks, <=1 kV)
% and EN 50549-2 (MV networks, 1-110 kV).
%
% Place this MATLAB Function block BETWEEN the outer voltage controller
% and the inner current controller of the cascaded GFM structure:
%
%   APC -> w, theta         (outer active-power/frequency droop)
%   RPC -> Vd_ref, Vq_ref=1 (outer reactive-power/voltage droop)
%          |
%          v
%   Voltage controller -> Id_ref, Iq_ref   (unsaturated current demand)
%          |
%          v
%   *** CCS (this block) *** -> Id_ref_lim, Iq_ref_lim
%          |
%          v
%   Current controller -> modulating voltage / PWM reference
%
% All quantities are in a common per-unit, converter-side dq frame
% aligned with theta produced by the APC (i.e. the same frame used by
% Vd_meas/Vq_meas, the voltage controller and the current controller).
%
% -------------------------------------------------------------------
% INPUTS
%   Id_ref, Iq_ref   Unsaturated dq current references out of the
%                    voltage controller [pu]
%   Vd_meas, Vq_meas Measured converter terminal / PCC voltage, dq [pu]
%   t                Simulation time [s] (Simulink: use Clock block).
%                    Only used for the FRT "must stay connected" curve.
%
%   Block parameters (mask / constants):
%   Imax     Maximum converter current magnitude [pu]        default 1.0-1.2
%   Vdb_lo   Lower voltage dead-band edge [pu]                default 0.90
%   Vdb_hi   Upper voltage dead-band edge [pu]                default 1.10
%   k_lvrt   Additional reactive current gain, dIq/dV [pu/pu] default 2.0
%            (EN 50549 leaves k, dead-band and timings to the National
%            Annex (NA) of each country/DSO -> re-tune these to match
%            the applicable NA before use)
%   Tramp    Ramp-out time constant used to blend the extra reactive
%            current back to zero once voltage has recovered [s]  ~0.2-1.0
%   Ts       Sample time of this block [s] (must match the discrete
%            solver / block sample time used in Simulink)
%
% OUTPUTS
%   Id_ref_lim, Iq_ref_lim   Saturated current references sent to the
%                            inner current controller [pu]
%   lvrt_active              1 while outside the voltage dead-band
%                            (undervoltage OR overvoltage support active)
%   must_stay_connected      1 while the measured voltage is still above
%                            the EN 50549 ride-through boundary curve,
%                            i.e. the converter is required to remain
%                            connected and current-limit rather than trip
%   Vpu                      Diagnostic: measured voltage magnitude [pu]
%
% -------------------------------------------------------------------
% CONTROL LOGIC
% 1) Reactive current priority during voltage dips/swells:
%       Vpu = sqrt(Vd_meas^2 + Vq_meas^2)
%       if Vpu < Vdb_lo:  Iq_support = +k_lvrt*(Vdb_lo - Vpu)  (capacitive)
%       if Vpu > Vdb_hi:  Iq_support = -k_lvrt*(Vpu - Vdb_hi)  (inductive)
%       else:             Iq_support = 0
%    The reactive current command is the envelope of the voltage-loop
%    demand and the mandatory support current, then hard-clamped to Imax.
%    Any remaining current budget is given to the active (Id) axis, so
%    reactive/voltage-support current always has priority over active
%    current, as required by LVRT/HVRT grid codes.
%
% 2) Fast entry / smooth exit:
%    The extra support current engages instantly (within one sample) to
%    meet the fast-response requirement of LVRT. On recovery it is
%    ramped back to the voltage-loop demand over Tramp seconds (a
%    persistent first-order blend) so the handover back to normal droop
%    operation does not reintroduce a current-reference step.
%
% 3) FRT boundary monitor (must_stay_connected):
%    A generic two-segment EN 50549-style ride-through boundary is
%    evaluated as a diagnostic/interlock flag:
%       t in [0, t1)      : Vboundary = V1
%       t in [t1, t2)     : Vboundary = V1 + (V2-V1)*(t-t1)/(t2-t1)
%       t >= t2           : Vboundary = V2
%    must_stay_connected = (Vpu >= Vboundary) while inside an active dip
%    (i.e. converter must remain connected/current-limiting rather than
%    trip). Default knee points (V1=0.15 pu @ t1=0.15 s,
%    V2=0.85 pu @ t2=1.5 s) follow the common EN 50549-1/2 style FRT
%    corridor and MUST be replaced with the exact points from the
%    applicable National Annex / DSO connection agreement.
%
% -------------------------------------------------------------------
% NOTE ON EN 50549-1 vs EN 50549-2
%   EN 50549-1 applies to generating units connected to LV networks
%   (<=1 kV), EN 50549-2 to MV networks (1-110 kV). Both define the same
%   *type* of requirement (a voltage dead-band, a proportional reactive
%   current support gain k, and a minimum ride-through time/voltage
%   corridor) but the numeric values are set nationally. This block
%   keeps every numeric value as a tunable parameter for that reason.

    persistent Iq_support_state initialized

    if isempty(initialized)
        Iq_support_state = 0;
        initialized = true;
    end

    % ---- 1. Voltage magnitude and dead-band evaluation -------------
    Vpu = sqrt(Vd_meas^2 + Vq_meas^2);

    if Vpu < Vdb_lo
        Iq_support_target = k_lvrt * (Vdb_lo - Vpu);   % capacitive (+)
        lvrt_active = true;
    elseif Vpu > Vdb_hi
        Iq_support_target = -k_lvrt * (Vpu - Vdb_hi);  % inductive (-)
        lvrt_active = true;
    else
        Iq_support_target = 0;
        lvrt_active = false;
    end
    Iq_support_target = max(min(Iq_support_target, Imax), -Imax);

    % ---- 2. Fast engage / ramped disengage of support current ------
    if abs(Iq_support_target) > abs(Iq_support_state)
        % Voltage event getting worse (or new event): react immediately.
        Iq_support_state = Iq_support_target;
    else
        % Recovering: blend support current back down over Tramp.
        if Tramp > 0
            alpha = Ts / Tramp;
            alpha = max(min(alpha, 1), 0);
        else
            alpha = 1;
        end
        Iq_support_state = Iq_support_state + alpha * (Iq_support_target - Iq_support_state);
    end

    % ---- 3. Reactive current priority + saturation ------------------
    if Iq_support_state >= 0
        Iq_cmd = max(Iq_ref, Iq_support_state);
    else
        Iq_cmd = min(Iq_ref, Iq_support_state);
    end
    Iq_ref_lim = max(min(Iq_cmd, Imax), -Imax);

    Id_max_avail = sqrt(max(Imax^2 - Iq_ref_lim^2, 0));
    Id_ref_lim = max(min(Id_ref, Id_max_avail), -Id_max_avail);

    % ---- 4. FRT "must stay connected" boundary (diagnostic/interlock)
    V1 = 0.15;  t1 = 0.15;   % <-- replace with National Annex values
    V2 = 0.85;  t2 = 1.5;    % <-- replace with National Annex values

    if t < t1
        Vboundary = V1;
    elseif t < t2
        Vboundary = V1 + (V2 - V1) * (t - t1) / (t2 - t1);
    else
        Vboundary = V2;
    end
    must_stay_connected = (Vpu >= Vboundary);

end
