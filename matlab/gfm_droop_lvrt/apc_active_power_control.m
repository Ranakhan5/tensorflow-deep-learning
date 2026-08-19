function [w, theta] = apc_active_power_control(Vd_meas, Vq_meas, Id_meas, Iq_meas, ...
                                                 P_ref, w0, m_p, wbase, Ts)
%#codegen
% APC_ACTIVE_POWER_CONTROL  Active Power Control (P-f droop) stage of a
% grid-forming (GFM) droop converter. Produces the angular frequency w
% and the internal angle theta used as the common dq reference frame
% for the RPC, voltage controller, CCS and current controller.
%
% INPUTS
%   Vd_meas, Vq_meas   Measured converter terminal voltage, dq [pu]
%   Id_meas, Iq_meas   Measured converter output current, dq [pu]
%   P_ref              Active power set-point [pu]
%   w0                 Nominal angular frequency set-point [pu], 1.0 = fn
%   m_p                Active power droop gain [pu w / pu P]
%   wbase              Base angular frequency [rad/s] (= 2*pi*fn) used to
%                       convert w from pu to rad/s before integrating theta
%   Ts                 Sample time [s]
%
% OUTPUTS
%   w                  Angular frequency reference [pu]
%   theta              Integrated angle [rad], wrapped to [-pi, pi]
%
% P = Vd*Id + Vq*Iq (instantaneous power in the converter dq frame).
% Droop law: w = w0 - m_p * (P_meas - P_ref)          [pu]
% theta is obtained by discrete integration of w*wbase (this IS the
% angle used to build every other dq frame in the control, i.e. this
% block is the "virtual rotor" of the GFM unit).

    persistent theta_state initialized
    if isempty(initialized)
        theta_state = 0;
        initialized = true;
    end

    P_meas = Vd_meas * Id_meas + Vq_meas * Iq_meas;

    w = w0 - m_p * (P_meas - P_ref);

    theta_state = theta_state + (w * wbase) * Ts;

    % wrap to [-pi, pi]
    theta_state = mod(theta_state + pi, 2*pi) - pi;

    theta = theta_state;
end
