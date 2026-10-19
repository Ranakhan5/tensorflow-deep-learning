%% LCL FILTER DESIGN & OPTIMISATION FOR A GRID-FORMING (GFM) TWO-LEVEL VSC
%  MATLAB version (no toolboxes required; R2016b or newer for implicit
%  expansion and local functions in scripts).  Mirrors lcl_gfm_design.py.
%
%  Two configurations are designed with the same code:
%    'A' : no transformer   converter - L1 - C(+damping) - L2 (discrete inductor) - PCC - grid
%    'B' : with transformer converter - L1 - C(+damping) - transformer leakage X_T/R_T - PCC - grid
%          (the transformer leakage IS the grid-side inductance; its uk is a design variable)
%
%  Damping topologies (p.damping):
%    'series' : Rd in series with the whole capacitor              (Liserre 2005)
%    'split'  : C = Cf + Cd, Rd in series with Cd only             (Pena-Alzola 2013)
%
%  Design flow
%   1 base quantities, grid impedance from SCR and X/R
%   2 classical step-by-step start design                        [Liserre 2005, Reznik 2014]
%   3 exact spectrum of regular-sampled SPWM/SVPWM               [Holmes & Lipo 2003]
%   4 grid-current TDD, h>=35 harmonics, capacitor/PCC THDv      [IEEE 519-2022, IEEE 1547-2018]
%   5 discrete-time cascaded GFM control (P+R voltage loop, P current loop,
%     i2 feed-forward, 1-sample delay + ZOH) over an SCR set:
%     closed-loop poles, GM, PM, modulus margin                  [Parker 2014, Dannehl 2010, Skogestad 2005]
%   6 power-synchronisation (VSM/droop) damping vs coupling X    [Wu 2016, Wang 2020, Rosso 2021]
%   7 constrained differential-evolution optimisation            [Muehlethaler 2013, Jayalath 2017]
%
%  Edit default_params() at the bottom of this file, then run the script.
%
%  This copy is parameterised for a 2 MW GFM inverter (Vdc = 763 V) connected
%  through a 20 kV / 400 V transformer to a 50 Hz, 20 kV grid with moderate X/R.
%  The step-up transformer is mandatory, so only case 'B' is run.

clear; clc; close all;
configs = {'B'};
results = cell(1, numel(configs));
for ic = 1:numel(configs)
    p = default_params(configs{ic});
    if strcmp(p.config, 'A'), tag = 'WITHOUT'; else, tag = 'WITH'; end
    fprintf('\n=========== Case %s: %s transformer ===========\n', p.config, tag);
    r_cls = evaluate(classical_design(p), p, false);
    describe(r_cls, p, 'Classical step-by-step design (Liserre / Reznik)');
    [x_opt, logd] = optimise(p);
    r_opt = evaluate(x_opt, p, true);
    describe(r_opt, p, 'Optimised design (continuous variables)');
    if strcmp(p.config, 'B')
        [uk_opt, uk_std, x_std, r_lo, r_hi, log2] = finalise_transformer(p, x_opt);
        r_opt = evaluate(x_std, p, true);
        logd = [logd; log2]; %#ok<AGROW>
        fprintf(' Optimal uk = %.2f %%  ->  catalogue transformer uk = %.1f %% (+/-%.0f %% tolerance)\n', ...
                uk_opt*100, uk_std*100, p.trafo_uk_tol*100);
        describe(r_opt, p, sprintf('Final design with uk = %.1f %%', uk_std*100));
        rr = {r_lo, r_hi};  sg = '-+';
        for k = 1:2
            fn = fieldnames(rr{k}.viol);  bad = {};
            for j = 1:numel(fn), if rr{k}.viol.(fn{j}) > 0, bad{end+1} = fn{j}; end, end %#ok<AGROW>
            if isempty(bad), bad = {'none'}; end
            fprintf(['   uk%stol = %.2f %%: zeta(SCR%g) = %.3f, modulus margin = %.3f, THDv = %.3f %%, ' ...
                     'm = %.3f, violated: %s\n'], sg(k), rr{k}.d.uk*100, max(p.SCR_set), ...
                    rr{k}.ps.stiff.zeta, min(rr{k}.sm_min, rr{k}.sm_inner_min), ...
                    rr{k}.harm_nom.THD_vf*100, rr{k}.m_req, strjoin(bad, ', '));
        end
    end
    results{ic} = struct('p', p, 'r', r_opt, 'log', logd);
end
make_plots(results);

%% ========================================================================
%  0. PARAMETERS (edit here)
%  ========================================================================
function p = default_params(config)
p.config   = config;          % 'A' = no transformer, 'B' = with transformer
% --- converter / grid ratings
p.P_max    = 2e6;             % inverter active-power rating [W]
p.S        = 2.2e6;           % inverter apparent power [VA] (P_max with |Q| <= 0.33 pu of S)
p.Vll_grid = 20e3;            % grid (MV) line-line rms voltage [V]
p.Vll_conv = 400;             % converter-side line-line rms voltage [V] (trafo LV side)
p.f1       = 50;              % grid frequency [Hz]
p.Vdc      = 763;             % dc-link voltage [V]
p.fsw      = 4e3;             % switching frequency [Hz] (MW-class IGBT stack, Tsw = 250 us)
p.update   = 'double';        % 'single' (fs = fsw) or 'double' (fs = 2 fsw = 8 kHz)
p.modulation = 'svpwm';       % 'svpwm' (min-max injection) or 'spwm'
p.damping  = 'split';         % 'split' or 'series'
% --- grid
p.SCR_nom  = 7;               % nominal SCR at the PCC (20 kV side, on base S)
p.XR_grid  = 5;               % grid X/R (moderate, typical 20 kV distribution feeder)
p.SCR_set  = [3 5 7 10 20];   % robustness set (must contain SCR_nom)
% --- parasitics
p.QL1 = 60;  p.QL2 = 60;      % X/R of L1 and discrete L2 at 50 Hz (MW-class reactors)
% --- transformer (case B)
p.S_trafo         = 2.5e6;        % transformer rating [VA] (next standard size above S)
p.trafo_rk        = 0.01;         % resistive short-circuit voltage [pu on S_trafo] (~25 kW load loss)
p.trafo_uk_bounds = [0.06 0.10];  % [pu on S_trafo]; IEC 60076-5: 6 % min. for 1251-2500 kVA
p.trafo_extra_L2  = 0;            % optional discrete inductor in series with the trafo [pu on S]
p.trafo_uk_std    = [0.06 0.065 0.07 0.075 0.08 0.09 0.10];  % catalogue uk values [pu on S_trafo]
p.trafo_uk_tol    = 0.10;         % uk tolerance (IEC 60076-1: +/-10 % for uk < 10 %)
p.de_maxiter_final = 40;          % DE generations for the re-optimisation with catalogue uk
% --- operating envelope
p.P_op  = p.P_max/p.S;  p.Q_max = 0.33;  p.V_pcc = 1;   % P_op = 0.909 pu
p.m_max = 0.95*2/sqrt(3);     % SVPWM linear limit with 5 % headroom
% --- cascaded GFM controller
p.fci_ratio = 1/20;           % inner current-loop crossover = ratio * fs
p.fcv_ratio = 1/5;            % outer voltage-loop crossover = ratio * fci
p.krv_ratio = 10;             % 50 Hz resonant gain / Kpv
p.wc_res    = 2*pi*2;         % resonant bandwidth [rad/s]
p.k_ff_i2   = 1;              % grid-current feed-forward gain
% --- power synchronisation (VSM / droop)
p.H_vsm = 1;  p.mp_droop = 0.02;  p.zeta_min = 0.30;  p.delta_max_deg = 30;
% --- limits
p.TDD_lim = 0.05;  p.Ih_lim = 0.003;  p.THDv_lim = 0.05;  p.ripple_max = 0.35;
p.x_max = 0.05;    p.Ltot_max = 0.25; p.fres_min_mult = 10; p.fres_max_frac = 0.5;
% (low fsw at MW scale needs more inductance: Ltot up to 0.25 pu, i1 ripple up to 35 %;
%  the voltage drop is still checked through the modulation-index limit m_max)
p.Pd_max = 0.003;  p.sm_min = 0.40;
% --- optimisation
p.bounds_L1 = [0.03 0.25];  p.bounds_C = [0.005 NaN];  p.bounds_L2 = [0.005 0.15];
p.bounds_Rd = [0.002 1.0];  p.bounds_kd = [0.1 0.9];
p.w_THDi = 1; p.w_THDv = 1; p.w_margin = 2; p.w_size = 0.1;
p.ref_THDi = 0.05; p.ref_THDv = 0.05; p.ref_margin = 0.5; p.ref_size = 0.10;
p.de_popsize = 20; p.de_maxiter = 80; p.de_seed = 1;
% --- numerics
p.n_freq = 3000;  p.harm_upper_fsw = 4.5;
% --- classical start design
p.cls_ripple = 0.20;  p.cls_x = 0.05;  p.cls_ka = 0.20;
if strcmp(config, 'B')
    p.bounds_L2 = p.trafo_uk_bounds;   % 3rd design variable becomes uk
else
    if p.Vll_grid ~= p.Vll_conv
        error('Case A (no transformer) needs Vll_grid = Vll_conv = 400 V.');
    end
    p.S_trafo = p.S;
end
end

%% ========================================================================
%  1. BASE QUANTITIES & GRID
%  ========================================================================
function b = base(p)
b.w1 = 2*pi*p.f1;  b.Vb = p.Vll_conv;  b.Zb = b.Vb^2/p.S;
b.Lb = b.Zb/b.w1;  b.Cb = 1/(b.w1*b.Zb);
b.Vph_pk = sqrt(2)*b.Vb/sqrt(3);  b.I_pk = sqrt(2)*p.S/(sqrt(3)*b.Vb);
b.fs = p.fsw*(1 + strcmp(p.update, 'double'));  b.Ts = 1/b.fs;
end

function [R, X] = grid_pu(SCR, XR)
if isinf(SCR), R = 0; X = 0; return; end
R = (1/SCR)/sqrt(1 + XR^2);  X = R*XR;
end

function nv = n_vars(p)
nv = 4 + strcmp(p.damping, 'split');
end

function d = unpack(x, p, b)
% x = [L1 pu, C pu, L2 pu (A) or uk (B), Rd pu, (kd = Cd/C for 'split')]
d.L1pu = x(1);  d.Cpu = x(2);  d.Rdpu = x(4);
if strcmp(p.damping, 'split'), d.kd = x(5); else, d.kd = 1; end
if strcmp(p.config, 'B')
    uk = x(3);  rk = min(p.trafo_rk, 0.999*uk);  d.uk = uk;   % on S_trafo base
    kt = p.S/p.S_trafo;                                      % S_trafo base -> inverter base S
    d.X2pu = sqrt(uk^2 - rk^2)*kt + p.trafo_extra_L2;
    d.R2pu = rk*kt + p.trafo_extra_L2/p.QL2;
else
    d.uk = NaN;  d.X2pu = x(3);  d.R2pu = x(3)/p.QL2;
end
d.R1pu = d.L1pu/p.QL1;
d.L1 = d.L1pu*b.Lb;  d.C = d.Cpu*b.Cb;  d.Cd = d.kd*d.C;  d.Cf = d.C - d.Cd;
d.L2 = d.X2pu*b.Lb;  d.R1 = d.R1pu*b.Zb;  d.R2 = d.R2pu*b.Zb;  d.Rd = d.Rdpu*b.Zb;
end

function Z = cap_branch_Z(d, s)
Zd = d.Rd + 1./(s*d.Cd);
if d.Cf <= 0, Z = Zd; return; end
Zf = 1./(s*d.Cf);  Z = Zf.*Zd./(Zf + Zd);
end

%% ========================================================================
%  2. CLASSICAL STEP-BY-STEP DESIGN (Liserre 2005 / Reznik 2014)
%  ========================================================================
function x = classical_design(p)
b = base(p);
dI = p.cls_ripple*b.I_pk;
L1 = p.Vdc/(6*p.fsw*dI);                       % max ripple Vdc/(6 fsw L1) at m = 0.5
C  = p.cls_x*b.Cb;
wsw = 2*pi*p.fsw;
r  = (1/p.cls_ka + 1)/(L1*C*wsw^2 - 1);         % ka = 1/|1 + r(1 - L1 C wsw^2)|
X2 = r*L1/b.Lb;
if strcmp(p.config, 'B')
    X2 = X2*p.S_trafo/p.S;                      % reactance -> uk on S_trafo base
    X2 = min(max(sqrt(X2^2 + p.trafo_rk^2), p.trafo_uk_bounds(1)), p.trafo_uk_bounds(2));
end
[~, Xg] = grid_pu(p.SCR_nom, p.XR_grid);
L2t  = r*L1 + Xg*b.Lb;
wres = sqrt((L1 + L2t)/(L1*L2t*C));
Rd   = 1/(3*wres*C);                            % Liserre's rule of thumb
x = [L1/b.Lb, C/b.Cb, X2, Rd/b.Zb];
if strcmp(p.damping, 'split'), x(5) = 0.5; end
end

%% ========================================================================
%  3. PWM SPECTRUM (exact for regular-sampled carrier PWM)
%  ========================================================================
function [n, Van] = pwm_spectrum(m, p)
% Peak phasors V_n (n = 1..Nh) of the phase voltage w.r.t. the floating star point.
% Each leg's high pulse in carrier period k is [ta, tb]; its Fourier integral is analytic.
persistent cache
if isempty(cache), cache = containers.Map('KeyType', 'char', 'ValueType', 'any'); end
key = sprintf('%.3f|%s|%s|%g|%g|%g|%g', round(m*1000)/1000, p.update, p.modulation, ...
              p.fsw, p.Vdc, p.f1, p.harm_upper_fsw);
if isKey(cache, key)
    v = cache(key);  n = v{1};  Van = v{2};  return;
end
Np = round(p.fsw/p.f1);  T1 = 1/p.f1;  Tsw = 1/p.fsw;  w1 = 2*pi*p.f1;
tk = (0:Np-1)*Tsw;
if strcmp(p.update, 'double'), ts2 = tk + Tsw/2; else, ts2 = tk; end
r1 = pwm_refs(tk, m, w1, p.modulation);
r2 = pwm_refs(ts2, m, w1, p.modulation);
% carrier +1 at tk, -1 at tk+Tsw/2, +1 at tk+Tsw; leg = +Vdc/2 while ref > carrier
ta = tk + (1 - r1)*Tsw/4;
tb = tk + Tsw/2 + (1 + r2)*Tsw/4;
Nh = ceil(p.harm_upper_fsw*p.fsw/p.f1);
n  = 1:Nh;  w = n*w1;
V = zeros(3, Nh);
for k = 1:3
    V(k, :) = (2/T1)*p.Vdc*sum(exp(-1i*ta(k, :).'*w) - exp(-1i*tb(k, :).'*w), 1)./(1i*w);
end
Van = V(1, :) - sum(V, 1)/3;                    % remove common mode (three-wire)
cache(key) = {n, Van};
end

function r = pwm_refs(t, m, w1, modulation)
r = [m*cos(w1*t); m*cos(w1*t - 2*pi/3); m*cos(w1*t - 4*pi/3)];
if strcmp(modulation, 'svpwm')
    r = r - 0.5*(max(r, [], 1) + min(r, [], 1));
end
r = min(max(r, -1), 1);
end

%% ========================================================================
%  4. OPERATING POINT, HARMONICS
%  ========================================================================
function op = operating_point(d, p, Q)
i2 = conj((p.P_op + 1i*Q)/p.V_pcc);
vf = p.V_pcc + (d.R2pu + 1i*d.X2pu)*i2;
Zd = d.Rdpu - 1i/(d.kd*d.Cpu);
if d.kd < 1
    Zf = -1i/((1 - d.kd)*d.Cpu);  Zc = Zf*Zd/(Zf + Zd);
else
    Zc = Zd;
end
i1 = i2 + vf/Zc;
op.v1 = vf + (d.R1pu + 1i*d.L1pu)*i1;
op.vf = vf;  op.i1 = i1;  op.i2 = i2;  op.id = vf/Zd;
end

function m = m_index(v1pu, p, b)
m = abs(v1pu)*b.Vph_pk/(p.Vdc/2);
end

function H = harmonics(d, p, b, SCR)
[Rgpu, Xgpu] = grid_pu(SCR, p.XR_grid);
Rg = Rgpu*b.Zb;  Lg = Xgpu*b.Lb;
op = operating_point(d, p, 0);
H.m = m_index(op.v1, p, b);
[n, Vn] = pwm_spectrum(H.m, p);
s  = 1i*n*b.w1;
Z1 = d.R1 + s*d.L1;  Zc = cap_branch_Z(d, s);  Z2 = d.R2 + Rg + s*(d.L2 + Lg);
I1 = Vn./(Z1 + Zc.*Z2./(Zc + Z2));
I2 = I1.*Zc./(Zc + Z2);
Vf = I2.*Z2;                                    % filter-node (capacitor) voltage
Vpcc = I2.*(Rg + s*Lg);
Id = Vf./(d.Rd + 1./(s*d.Cd));
h = n >= 2;
H.TDD_i2   = sqrt(sum(abs(I2(h)).^2))/b.I_pk;
H.Ih_max   = max(abs(I2(n >= 35)))/b.I_pk;
H.THD_vf   = sqrt(sum(abs(Vf(h)).^2))/b.Vph_pk;
H.THD_vpcc = sqrt(sum(abs(Vpcc(h)).^2))/b.Vph_pk;
% switching ripple of i1 over one fundamental period
hs = n >= 0.5*p.fsw/p.f1;
N  = 2*numel(n);
X  = zeros(1, N);
X(1 + n(hs)) = I1(hs)*N/2;
rip = ifft(X, 'symmetric');
H.ripple_pp = (max(rip) - min(rip))/b.I_pk;
Id1 = abs(op.id)*b.I_pk;
H.Pd = 3*d.Rd*(Id1^2 + sum(abs(Id(h)).^2))/2/p.S;
H.n = n;  H.Vn = Vn;  H.I2 = I2;  H.Vf = Vf;
end

function f = resonance_hz(d, Lg)
L2t = d.L2 + Lg;
f = sqrt((d.L1 + L2t)/(d.L1*L2t*d.C))/(2*pi);
end

%% ========================================================================
%  5. DISCRETE-TIME CASCADED GFM CONTROL & STABILITY MARGINS
%  ========================================================================
function [Ad, Bd] = c2d_zoh(A, B, T)
n = size(A, 1);  m = size(B, 2);
M = zeros(n + m);  M(1:n, 1:n) = A;  M(1:n, n+1:end) = B;
E = expm(M*T);  Ad = E(1:n, 1:n);  Bd = E(1:n, n+1:end);
end

function [A, B, Ci1, Cvf, Ci2] = plant_ss(d, R2t, L2t)
L1 = d.L1; R1 = d.R1; Rd = d.Rd; C = d.C;
if d.Cf <= 0          % 'series': x = [i1 vc i2]
    A = [-(R1+Rd)/L1, -1/L1,  Rd/L1;
          1/C,          0,    -1/C;
          Rd/L2t,     1/L2t, -(R2t+Rd)/L2t];
    B = [1/L1; 0; 0];
    Ci1 = [1 0 0];  Cvf = [Rd 1 -Rd];  Ci2 = [0 0 1];
else                  % 'split': x = [i1 vcf vcd i2]
    Cf = d.Cf;  Cd = d.Cd;
    A = [-R1/L1, -1/L1,          0,             0;
          1/Cf,  -1/(Rd*Cf),     1/(Rd*Cf),    -1/Cf;
          0,      1/(Rd*Cd),    -1/(Rd*Cd),     0;
          0,      1/L2t,         0,            -R2t/L2t];
    B = [1/L1; 0; 0; 0];
    Ci1 = [1 0 0 0];  Cvf = [0 1 0 0];  Ci2 = [0 0 0 1];
end
end

function [Kpi, Kpv, Krv] = controller_gains(d, p, b)
fci = p.fci_ratio*b.fs;
Kpi = 2*pi*fci*d.L1;                  % i1/v1 ~ 1/(s L1) near crossover
Kpv = 2*pi*p.fcv_ratio*fci*d.C;       % vf/i1 ~ 1/(s C) with i2 feed-forward
Krv = p.krv_ratio*Kpv;
end

function [Aol, Bw, Ku, Acl] = build_loops(d, p, b, SCR, outer)
% States: [plant (3 or 4) | delayed modulator voltage | resonant controller (2)]
%   u[k] = Kpi*(i1ref - i1), i1ref = Kpv*(-vf) + Krv*R(z)(-vf) + kff*i2,  v1[k+1] = u[k]
[Rgpu, Xgpu] = grid_pu(SCR, p.XR_grid);
[A, B, Ci1, Cvf, Ci2] = plant_ss(d, d.R2 + Rgpu*b.Zb, d.L2 + Xgpu*b.Lb);
[Ad, Bd] = c2d_zoh(A, B, b.Ts);
[Kpi, Kpv, Krv] = controller_gains(d, p, b);
kff = p.k_ff_i2;
if ~outer, Kpv = 0; Krv = 0; kff = 0; end
wc = p.wc_res;
[Ard, Brd] = c2d_zoh([0 1; -b.w1^2 -2*wc], [0; 1], b.Ts);
Cr = [0 2*wc];                        % R(s) = 2 wc s/(s^2 + 2 wc s + w1^2)
npl = size(A, 1);  n = npl + 3;  iu = npl + 1;
Aol = zeros(n);
Aol(1:npl, 1:npl) = Ad;
Aol(1:npl, iu)    = Bd(:, 1);
Aol(npl+2:n, 1:npl)   = -Brd*Cvf;
Aol(npl+2:n, npl+2:n) = Ard;
Bw = zeros(n, 1);  Bw(iu) = 1;
Ku = zeros(1, n);
Ku(1:npl)   = Kpi*(-Kpv*Cvf + kff*Ci2 - Ci1);
Ku(npl+2:n) = Kpi*Krv*Cr;
Acl = Aol;  Acl(iu, :) = Ku;
end

function H = freq_resp(A, B, C, z)
% C (zI - A)^-1 B on a vector of z (modal form, direct solve if ill-conditioned)
[V, D] = eig(A);  lam = diag(D);
if cond(V) < 1e10
    Vi = V\B;  CV = C*V;
    H = sum((CV.*Vi.')./(z(:) - lam.'), 2);
else
    H = zeros(numel(z), 1);  I = eye(size(A));
    for k = 1:numel(z), H(k) = C*((z(k)*I - A)\B); end
end
end

function mg = margins(L)
% modulus margin min|1+L|; PM = angular distance from -1 at each gain crossover;
% GM = smallest gain change (up or down) moving a -180 deg crossing onto -1.
L = L(:);  mag = abs(L);  ph = angle(L);
mg.sm = min(abs(1 + L));
idx = find(diff(sign(mag - 1)) ~= 0);
if isempty(idx), mg.PM = 180; else, mg.PM = min(180 - abs(ph(idx))*180/pi); end
GMs = [];
idx = find(diff(sign(imag(L))) ~= 0);
for i = idx.'
    t  = imag(L(i))/(imag(L(i)) - imag(L(i+1)));
    Lc = L(i) + t*(L(i+1) - L(i));
    if real(Lc) < 0, GMs(end+1) = abs(20*log10(abs(real(Lc)))); end %#ok<AGROW>
end
if isempty(GMs), mg.GM = Inf; else, mg.GM = min(GMs); end
end

function st = stability(d, p, b, SCR)
st.f = logspace(0, log10(0.999*b.fs/2), p.n_freq).';
z = exp(1i*2*pi*st.f*b.Ts);
names = {'sys', 'inner'};  outers = [true false];
for k = 1:2
    [Aol, Bw, Ku, Acl] = build_loops(d, p, b, SCR, outers(k));
    L  = -freq_resp(Aol, Bw, Ku, z);           % loop gain at the modulator input
    mg = margins(L);
    mg.rho = max(abs(eig(Acl)));
    mg.L = L;
    st.(names{k}) = mg;
end
end

%% ========================================================================
%  6. POWER-SYNCHRONISATION (VSM / DROOP) DAMPING
%  ========================================================================
function ps = power_sync(d, p, b, SCR)
% (2H/wb) s^2 + (D/wb) s + Ks = 0, Ks = E V cos(d0)/X, X = X2 + Xg
[~, Xg] = grid_pu(SCR, p.XR_grid);
X  = d.X2pu + Xg;
d0 = asin(min(p.P_op*X, 0.999));
Ks = cos(d0)/X;
ps.wn = sqrt(b.w1*Ks/(2*p.H_vsm));
ps.zeta = (1/p.mp_droop)/(4*p.H_vsm*ps.wn);
ps.delta_deg = d0*180/pi;  ps.Xtot = X;
end

%% ========================================================================
%  7. EVALUATION, COST, OPTIMISATION
%  ========================================================================
function r = evaluate(x, p, keep)
b = base(p);  d = unpack(x, p, b);
r.x = x;  r.d = d;
Qs = [-p.Q_max 0 p.Q_max];  m = zeros(1, 3);
for k = 1:3
    op = operating_point(d, p, Qs(k));
    m(k) = m_index(op.v1, p, b);
end
r.m_req = max(m);
v.m    = r.m_req/p.m_max - 1;
v.x    = d.Cpu/p.x_max - 1;
v.Ltot = (d.L1pu + d.X2pu)/p.Ltot_max - 1;
nS = numel(p.SCR_set);  inom = find(p.SCR_set == p.SCR_nom, 1);
TDD = zeros(1, nS);  Ih = TDD;  THv = TDD;  fr = TDD;
sm = TDD;  smi = TDD;  rho = TDD;  PM = TDD;  GM = TDD;  PMi = TDD;  GMi = TDD;
st = cell(1, nS);
for k = 1:nS
    S = p.SCR_set(k);
    Hk = harmonics(d, p, b, S);
    if k == inom, r.harm_nom = Hk; end
    TDD(k) = Hk.TDD_i2;  Ih(k) = Hk.Ih_max;  THv(k) = Hk.THD_vf;
    [~, Xg] = grid_pu(S, p.XR_grid);
    fr(k) = resonance_hz(d, Xg*b.Lb);
    st{k} = stability(d, p, b, S);
    sm(k) = st{k}.sys.sm;  smi(k) = st{k}.inner.sm;
    rho(k) = max(st{k}.sys.rho, st{k}.inner.rho);
    PM(k) = st{k}.sys.PM;  GM(k) = st{k}.sys.GM;  PMi(k) = st{k}.inner.PM;  GMi(k) = st{k}.inner.GM;
end
Hn = r.harm_nom;
r.TDD_worst = max(TDD);  r.Ih_worst = max(Ih);  r.THDvf_worst = max(THv);
v.TDD    = r.TDD_worst/p.TDD_lim - 1;
v.Ih     = r.Ih_worst/p.Ih_lim - 1;
v.THDv   = r.THDvf_worst/p.THDv_lim - 1;
v.ripple = Hn.ripple_pp/p.ripple_max - 1;
v.Pd     = Hn.Pd/p.Pd_max - 1;
r.fres_range = [min(fr) max(fr)];
v.fres_lo = p.fres_min_mult*p.f1/min(fr) - 1;
v.fres_hi = max(fr)/(p.fres_max_frac*p.fsw) - 1;
if keep, r.stab = st; else, r.stab = []; end
r.sm_min = min(sm);  r.sm_inner_min = min(smi);  r.rho_max = max(rho);
r.PM_min = min(PM);  r.GM_min = min(GM);  r.PM_inner = min(PMi);  r.GM_inner = min(GMi);
smw = min(r.sm_min, r.sm_inner_min);
v.stable = (r.rho_max - (1 - 1e-7))*1e3;
v.sm     = p.sm_min/max(smw, 1e-6) - 1;
r.ps.stiff = power_sync(d, p, b, max(p.SCR_set));
r.ps.weak  = power_sync(d, p, b, min(p.SCR_set));
r.ps.nom   = power_sync(d, p, b, p.SCR_nom);
v.zeta  = p.zeta_min/r.ps.stiff.zeta - 1;
v.delta = r.ps.weak.delta_deg/p.delta_max_deg - 1;
r.size = d.L1pu + d.X2pu + d.Cpu;
J = p.w_THDi*Hn.TDD_i2/p.ref_THDi + p.w_THDv*Hn.THD_vf/p.ref_THDv ...
  + p.w_margin*p.ref_margin/max(smw, 1e-3) + p.w_size*r.size/p.ref_size;
fn = fieldnames(v);  pen = 0;
for k = 1:numel(fn), pen = pen + max(0, v.(fn{k})); end
r.viol = v;  r.feasible = (pen == 0);  r.J = J;  r.cost = J + 1e3*pen;
end

function bnds = opt_bounds(p)
bC = p.bounds_C;  if isnan(bC(2)), bC(2) = p.x_max; end
bnds = [p.bounds_L1; bC; p.bounds_L2; p.bounds_Rd];
if strcmp(p.damping, 'split'), bnds = [bnds; p.bounds_kd]; end
end

function [xbest, logd] = optimise(p)
% Differential evolution, best/1/bin with dithered F (same settings as the Python file)
bnds = opt_bounds(p);  lo = bnds(:, 1).';  hi = bnds(:, 2).';  nv = numel(lo);
NP = p.de_popsize*nv;  rng(p.de_seed);
pop = lo + rand(NP, nv).*(hi - lo);
pop(1, :) = min(max(classical_design(p), lo), hi);       % seed with the classical design
cost = zeros(NP, 1);
logd = zeros(NP*(p.de_maxiter + 1), 5);  nl = 0;
tic;
for i = 1:NP
    r = evaluate(pop(i, :), p, false);  cost(i) = r.cost;
    nl = nl + 1;  logd(nl, :) = log_row(r);
end
for g = 1:p.de_maxiter
    [~, ib] = min(cost);  best = pop(ib, :);
    F = 0.5 + 0.4*rand;
    trial = pop;
    for i = 1:NP
        idx = randperm(NP, 3);
        while any(idx == i), idx = randperm(NP, 3); end
        mutant = best + F*(pop(idx(1), :) - pop(idx(2), :));
        cr = rand(1, nv) < 0.8;  cr(randi(nv)) = true;
        t = pop(i, :);  t(cr) = mutant(cr);
        out = t < lo | t > hi;
        t(out) = lo(out) + rand(1, nnz(out)).*(hi(out) - lo(out));
        trial(i, :) = t;
    end
    for i = 1:NP
        r = evaluate(trial(i, :), p, false);
        nl = nl + 1;  logd(nl, :) = log_row(r);
        if r.cost <= cost(i), pop(i, :) = trial(i, :); cost(i) = r.cost; end
    end
    if std(cost) <= 1e-7*abs(mean(cost)), break; end
end
logd = logd(1:nl, :);
[~, ib] = min(cost);  xbest = pop(ib, :);
fprintf('   DE finished in %.0f s, %d evaluations\n', toc, nl);
end

function [uk_opt, uk_std, x_std, r_lo, r_hi, logd] = finalise_transformer(p, x_opt)
% Case B: smallest catalogue uk whose lower tolerance limit still covers the optimal uk,
% re-optimise the other elements with that uk fixed, check at the uk tolerance limits.
uk_opt = x_opt(3);
k = find(p.trafo_uk_std*(1 - p.trafo_uk_tol) >= uk_opt - 1e-9, 1);
if isempty(k), uk_std = p.trafo_uk_std(end); else, uk_std = p.trafo_uk_std(k); end
q = p;  q.bounds_L2 = [uk_std uk_std*(1 + 1e-9)];  q.de_maxiter = p.de_maxiter_final;
[x_std, logd] = optimise(q);
x_lo = x_std;  x_lo(3) = uk_std*(1 - p.trafo_uk_tol);
x_hi = x_std;  x_hi(3) = uk_std*(1 + p.trafo_uk_tol);
r_lo = evaluate(x_lo, p, false);
r_hi = evaluate(x_hi, p, false);
end

function row = log_row(r)
row = [r.size, r.harm_nom.TDD_i2, r.harm_nom.THD_vf, min(r.sm_min, r.sm_inner_min), double(r.feasible)];
end

%% ========================================================================
%  8. REPORT / PLOTS
%  ========================================================================
function describe(r, p, title)
b = base(p);  d = r.d;  Hn = r.harm_nom;  ps = r.ps;
fprintf('--- %s ---\n', title);
fprintf('  Base: %.2f MVA, %.0f V, I_rated = %.0f A rms, Zb = %.2f mohm\n', p.S/1e6, b.Vb, ...
        b.I_pk/sqrt(2), b.Zb*1e3);
fprintf('  L1 = %7.2f uH  (%.4f pu)\n', d.L1*1e6, d.L1pu);
if d.Cf > 0
    fprintf('  C  = %7.2f uF per phase, Y (%.4f pu) = Cf %.2f uF || (Rd + Cd %.2f uF)\n', ...
            d.C*1e6, d.Cpu, d.Cf*1e6, d.Cd*1e6);
else
    fprintf('  C  = %7.2f uF per phase, Y (%.4f pu), Rd in series\n', d.C*1e6, d.Cpu);
end
if strcmp(p.config, 'B')
    fprintf(['  Transformer %.1f MVA, %.0f kV / %.0f V, uk = %.2f %% (rk = %.1f %%) -> X_T = %.4f pu ' ...
             '(inverter base) = %.3f mohm = %.2f uH per phase (LV side)\n'], p.S_trafo/1e6, ...
            p.Vll_grid/1e3, p.Vll_conv, d.uk*100, p.trafo_rk*100, d.X2pu, d.X2pu*b.Zb*1e3, d.L2*1e6);
else
    fprintf('  L2 = %7.3f mH  (%.4f pu)\n', d.L2*1e3, d.X2pu);
end
fprintf('  Rd = %7.3f mohm (%.4f pu)\n', d.Rd*1e3, d.Rdpu);
fprintf('  f_res over SCR %g..%g = %.0f .. %.0f Hz   (fs/6 = %.0f Hz)\n', min(p.SCR_set), ...
        max(p.SCR_set), r.fres_range(1), r.fres_range(2), b.fs/6);
fprintf('  Worst-case modulation index (P=%.3g, |Q|<=%g) = %.3f   (limit %.3f)\n', p.P_op, p.Q_max, ...
        r.m_req, p.m_max);
fprintf('  Grid-current TDD   SCR%g: %.3f %%   worst: %.3f %%\n', p.SCR_nom, Hn.TDD_i2*100, r.TDD_worst*100);
fprintf('  Largest Ih, h>=35  (worst SCR): %.4f %%   (limit %.1f %%)\n', r.Ih_worst*100, p.Ih_lim*100);
fprintf('  Capacitor-node THDv SCR%g: %.3f %%   worst: %.3f %%\n', p.SCR_nom, Hn.THD_vf*100, r.THDvf_worst*100);
fprintf('  PCC THDv           SCR%g: %.3f %%\n', p.SCR_nom, Hn.THD_vpcc*100);
fprintf('  i1 ripple pk-pk / I_rated,pk: %.1f %%     damping loss: %.2f kW\n', Hn.ripple_pp*100, Hn.Pd*p.S/1e3);
fprintf('  Whole GFM loop (worst SCR): modulus margin %.3f, GM %.1f dB, PM %.1f deg, max|pole| %.4f\n', ...
        r.sm_min, r.GM_min, r.PM_min, r.rho_max);
fprintf('  Inner current loop alone  : modulus margin %.3f, GM %.1f dB, PM %.1f deg\n', ...
        r.sm_inner_min, r.GM_inner, r.PM_inner);
fprintf('  Power-sync damping: zeta(SCR%g) = %.3f, zeta(SCR%g) = %.3f, load angle(SCR%g) = %.1f deg\n', ...
        max(p.SCR_set), ps.stiff.zeta, p.SCR_nom, ps.nom.zeta, min(p.SCR_set), ps.weak.delta_deg);
fn = fieldnames(r.viol);  bad = {};
for k = 1:numel(fn), if r.viol.(fn{k}) > 0, bad{end+1} = fn{k}; end, end %#ok<AGROW>
if isempty(bad), fprintf('  Feasible: true\n');
else, fprintf('  Feasible: false   (violated: %s)\n', strjoin(bad, ', ')); end
end

function make_plots(results)
figure('Name', 'LCL design for GFM', 'Position', [80 80 1250 850]);
col = {[0.12 0.44 0.71], [0.78 0.21 0.17]};  mk = {'o', '^'};
for ic = 1:numel(results)
    p = results{ic}.p;  r = results{ic}.r;  lg = results{ic}.log;  b = base(p);
    for k = 1:numel(p.SCR_set)
        s = r.stab{k};  L = s.sys.L;
        if p.SCR_set(k) == p.SCR_nom, lw = 2.2; else, lw = 0.7; end
        subplot(2, 2, 1); semilogx(s.f, 20*log10(abs(L)), 'Color', col{ic}, 'LineWidth', lw); hold on;
        subplot(2, 2, 3); semilogx(s.f, angle(L)*180/pi, 'Color', col{ic}, 'LineWidth', lw); hold on;
    end
    Hn = r.harm_nom;
    subplot(2, 2, 2);
    semilogy(Hn.n(2:end)*p.f1/1e3, abs(Hn.I2(2:end))/b.I_pk*100 + 1e-9, '.', 'Color', col{ic}, ...
             'MarkerSize', 4, 'DisplayName', ['case ' p.config]); hold on;
    subplot(2, 2, 4);
    ok = lg(:, 5) > 0;
    scatter(lg(ok, 1), lg(ok, 3)*100, 6, lg(ok, 4), mk{ic}, 'HandleVisibility', 'off'); hold on;
    plot(r.size, Hn.THD_vf*100, 'p', 'MarkerSize', 16, 'MarkerFaceColor', col{ic}, ...
         'MarkerEdgeColor', 'k', 'DisplayName', ['selected, case ' p.config]);
end
subplot(2, 2, 1); yline(0, 'k'); grid on; ylabel('|L(e^{j\omegaT_s})| [dB]');
title('Loop gain at modulator input (thick: SCR nominal)');
subplot(2, 2, 3); yline(-180, 'k'); yline(180, 'k'); grid on; ylabel('phase [deg]'); xlabel('f [Hz]');
subplot(2, 2, 2); yline(0.3, 'k--'); grid on; xlim([0 4.5*results{1}.p.fsw/1e3]); ylim([1e-6 1]);
xlabel('f [kHz]'); ylabel('|I_{2,h}| / I_{rated,pk} [%]'); legend('show');
title('Grid-current harmonics at SCR nominal (dashed: 0.3 % limit)');
subplot(2, 2, 4); grid on; colormap(gca, parula); caxis([0.3 0.7]);
cb = colorbar; cb.Label.String = 'modulus margin (worst SCR)';
xlabel('filter size L1 + X2 + C [pu]'); ylabel('capacitor-node THD_v [%]'); legend('show');
title('Feasible designs visited by the optimiser');
end
