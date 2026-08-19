function [Vd_ref, Vq_ref] = rpc_reactive_power_control(Vd_meas, Vq_meas, Id_meas, Iq_meas, ...
                                                         Q_ref, V0, m_q)
%#codegen
% RPC_REACTIVE_POWER_CONTROL  Reactive Power Control (Q-V droop) stage
% of a grid-forming (GFM) droop converter. Produces the internal
% voltage-magnitude reference Vd_ref; Vq_ref is fixed at 1 pu, i.e. the
% dq frame convention used throughout this control set has the
% commanded internal EMF magnitude carried on the q-axis (Vq_ref = 1 pu
% at rated voltage) with Vd_ref carrying the droop-induced deviation.
%
% INPUTS
%   Vd_meas, Vq_meas   Measured converter terminal voltage, dq [pu]
%   Id_meas, Iq_meas   Measured converter output current, dq [pu]
%   Q_ref              Reactive power set-point [pu]
%   V0                 Nominal voltage magnitude set-point [pu], 1.0 = Vn
%   m_q                Reactive power droop gain [pu V / pu Q]
%
% OUTPUTS
%   Vd_ref             Droop-adjusted d-axis voltage reference [pu]
%   Vq_ref             Fixed q-axis voltage reference = 1 pu
%
% Q = Vq*Id - Vd*Iq (instantaneous reactive power in the converter dq
% frame, consistent with Vq carrying the voltage magnitude).
% Droop law: Vd_ref = m_q * (Q_ref - Q_meas), referenced about V0 on Vq.

    Q_meas = Vq_meas * Id_meas - Vd_meas * Iq_meas;

    Vd_ref = m_q * (Q_ref - Q_meas);
    Vq_ref = V0;   % fixed at 1 pu (rated) per the stated convention

end
