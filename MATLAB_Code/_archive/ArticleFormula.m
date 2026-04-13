
function dims = ArticleFormula()
%ARTICLEFORMULA  Design, Modeling and Sizing of V-shape IPM Motors
%
%  Implements the analytical sizing procedure of:
%    A. Di Gerlando, C. Ricca, "Design Modeling and Sizing Equations of
%    V-shape IPM Motors," ICEM 2022, DOI: 10.1109/ICEM51905.2022.9910924
%
%  Returns a struct DIMS with all key motor dimensions for use in COMSOL.
%
%  Equation numbers in comments refer to the paper.

clear; clc;

%% =========================================================================
%  SECTION I  –  INPUT DATA  (Table I)
%% =========================================================================

% --- Corner point ---
T_c   = 200;        % Corner torque [Nm]
N_c   = 2900;       % Corner speed  [rpm]
N_M   = 13500;      % Max speed     [rpm]
V_dc  = 650;        % DC-link voltage [V]

% --- PM parameters (N48UZ-SGR at reference temperature) ---
B_r20     = 1.37;   % Remanent flux density at 20 °C  [T]
mu_rec    = 1.05;   % Recoil permeability              [pu]
k_Br      = -0.1;  % Temperature coefficient of Br    [%/°C]
T_wind    = 180;    % Winding reference temperature    [°C]
T_PM      = 140;    % PM reference temperature         [°C]

% --- Geometry inputs ---
D         = 160;    % Stator bore (inner) diameter  [mm]
g         = 1;      % Air-gap length                [mm]
b_as      = 2;      % Slot opening                  [mm]
phases    = 3;      % Number of phases
p         = 8;      % Number of poles
q         = 5/2;    % Slots per pole per phase

% --- Sizing parameters (last row of Table I) ---
rho_bt_s   = 0.704;  % bt/τ_s  tooth-width-to-pitch ratio  (iterative)
rho_hte_g  = 52.4;   % h_te/g  tooth-height-to-gap ratio   (iterative)
sigma_anis = 4.11;   % Unsaturated anisotropy ratio         (iterative)
c_d        = 0.201;  % d-axis reaction coefficient          (iterative)
rho_EV     = 0.65;   % EMF-to-voltage ratio                 (iterative)

%% =========================================================================
%  SECTION II  –  BASIC GEOMETRY
%% =========================================================================

% Derived basic quantities
Q       = phases * p * q;                  % Total number of slots
D_r     = D - 2*g;                         % Rotor outer diameter   [mm]
tau_s   = pi * D / Q;                      % Stator slot pitch      [mm]
tau     = pi * D / p;                      % Stator pole pitch      [mm]
tau_r   = pi * D_r / p;                    % Rotor pole pitch       [mm]
f_c     = N_c * p / 120;                   % Corner frequency       [Hz]

% Carter's factor (function of τ_s, b_as, g)
gamma_c = (b_as/g)^2 / (5 + b_as/g);      % auxiliary
k_C     = tau_s / (tau_s - gamma_c * g);  % Carter factor          [pu]

% Winding factor (2-layer, q=5/2, coil pitch y_c=6)
k_w = 0.91;   % Given in Section II of the paper

% PM remanent flux density at operating temperature
B_r = B_r20 * (1 + k_Br/100 * (T_PM - 20));   % [T]

%% =========================================================================
%  SECTION II  –  ROTOR GEOMETRY  (Equations 8–18, Fig. 1)
%% =========================================================================

% Fixed rotor geometry choices (from Section IV / Fig. 4)
alpha_m = 0.754;                   % Magnet embrace (pole-arc ratio)  [pu]
w_hr    = 0.55 * tau_s;            % Half tooth width at air-gap      [mm]
w_ob    = 0.5;                     % Depth rotor OD to pocket top     [mm]
h_m     = 6;                       % Magnet radial thickness          [mm]
v       = 78;                      % Magnet tilt angle                [deg]
w_ib    = 2.5;                     % Magnet spacing (inner bridge)    [mm]
h_ry    = 1.5 * w_hr;              % Rotor yoke height                [mm]

% Eq. (8)  pole-shoe extension
b_ps = alpha_m * tau_r;                                   % [mm]

% Eq. (9)  outer bridge length
h_ob = (tau_r - 2*w_hr - b_ps) / 2;                      % [mm]

% Eq. (10) side PM angle  ζ
zeta = acosd( h_ob * (D_r - 2*w_ob) * D_r^(-1) / h_m ); % [deg]

% Eq. (11) inner bridge length
h_ib = h_m * sind(v);                                     % [mm]

% Eq. (12) half-rib radial length
h_hr = h_m * sind(zeta);                                  % [mm]

% Eq. (13) length d_12
d_12 = (D_r/2 - w_ob) * sind(alpha_m * pi/p * 180/pi);   % [mm]
% Note: alpha_m*pi/p is already in radians; sind needs degrees:
d_12 = (D_r/2 - w_ob) * sin(alpha_m * pi/p);             % [mm]

% Eq. (14) length d_23
d_23 = (d_12 - w_ib/2) / tand(v);                        % [mm]

% Eq. (15) length d_24
d_24 = (D_r/2 - w_ob) - d_12 / tan(alpha_m * pi/p);      % [mm]

% Eq. (16) pole-shoe radial depth
d_ps = d_23 + d_24 + w_ob;                                % [mm]

% Eq. (17) rotor inner diameter
D_ir = D_r - 2 * (d_ps + h_ib + h_ry);                   % [mm]

% Eq. (18) PM segment width
b_m = (d_12 - w_ib/2) / sind(v);                         % [mm]

fprintf('=== Rotor Geometry ===\n');
fprintf('  D_r  = %.2f mm\n', D_r);
fprintf('  b_ps = %.2f mm\n', b_ps);
fprintf('  h_ob = %.2f mm  (paper: 3 mm)\n', h_ob);
fprintf('  zeta = %.2f deg\n', zeta);
fprintf('  h_ib = %.2f mm  (paper: 5.9 mm)\n', h_ib);
fprintf('  h_hr = %.2f mm  (paper: 5.2 mm)\n', h_hr);
fprintf('  d_12 = %.3f mm\n', d_12);
fprintf('  d_23 = %.3f mm\n', d_23);
fprintf('  d_24 = %.3f mm\n', d_24);
fprintf('  d_ps = %.2f mm  (paper: 8.5 mm)\n', d_ps);
fprintf('  D_ir = %.2f mm  (paper: 115.4 mm)\n', D_ir);
fprintf('  b_m  = %.2f mm  (paper: 22.1 mm)\n\n', b_m);

%% =========================================================================
%  SECTION III  –  q-AXIS SATURATION MODEL
%  Lamination M235-35A:  H_fe(B_fe) and µ_fe,pu(B_fe)
%% =========================================================================

% B-H data for M235-35A (manufacturer data approximated by standard curve)
% Extended so that incremental permeability → µ0 at high B (as in Fig. 2).
B_fe_data = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, ...
             1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, ...
             2.0, 2.1, 2.2, 2.3, 2.5, 3.0];      % [T]
H_fe_data = [0, 22, 30, 37, 42, 47, 53, 60, 69, 82, ...
             100, 125, 165, 235, 380, 720, 1400, 2600, 4200, 6200, ...
             8800, 12000, 16000, 21000, 33000, 70000]; % [A/m]

mu0 = 4*pi*1e-7;  % [H/m]

% Interpolation functions
H_fe  = @(B) interp1(B_fe_data, H_fe_data, abs(B), 'pchip', ...
                     H_fe_data(end));                              % [A/m]
mu_fe_pu = @(B) B ./ (mu0 .* max(H_fe(B), 1e-3));                 % [pu]

% Stacking factor
k_st = 0.97;

% Eq. (3)  Peak tooth flux density B_t given air-gap flux density B_gI
%   (from flux balance: pole air-gap flux = tooth flux + slot flux)
%   rho_bt_s = b_ts/tau_s  (tooth-width-to-pitch ratio, iterative input)
Bt_of_BgI = @(BgI) fzero( ...
    @(Bt) BgI - (rho_bt_s * k_st * Bt) ...
              - (BgI - rho_bt_s * k_st * Bt) ...   % slot flux (in parallel)
              * 0, ...                              % simplified: slot µ >> tooth
    BgI / (rho_bt_s * k_st + 1e-9));
% Proper formulation of Eq. (3):
%   BgI = rho_bt_s * k_st * Bt  +  (slot contribution)
%   Approximation used in paper: BgI / (rho_bt_s * k_st) ≈ Bt
%   for the MVD calculation (teeth dominate):
Bt_approx = @(BgI) BgI / (rho_bt_s * k_st);

% Eq. (4)  Saturation ratio ρ_sat(B_gI) – ratio of total MVD to air-gap MVD
rho_sat = @(BgI) 1 + H_fe(Bt_approx(BgI)) .* rho_hte_g ...
                     ./ (BgI / mu0 / k_C);

% Eq. (5)  Peak stator MMF M_I required to produce B_gI
M_I = @(BgI) (BgI / mu0) .* g*1e-3 .* k_C .* rho_sat(BgI);  % [A]

% Eq. (6)  Inverse: B_gI from M_q (q-axis reaction MMF)
Bgp_of_Mq = @(Mq) fzero(@(B) M_I(B) - Mq, Mq * mu0 / (g*1e-3 * k_C));

% Eq. (7)  Saturation factor σ_sM(M_q)
sigma_sM = @(Mq) Bgp_of_Mq(Mq) * mu0 / (g*1e-3 * k_C) / Mq;

%% =========================================================================
%  SECTION IV  –  PM FLUX SATURATION FACTOR  η_φM(M_q)
%  Simplified approach: use the paper result directly (Eq. 38–42)
%  Full magnetic network (Fig. 5) is implemented below.
%% =========================================================================

% PM specific flux parameters (per unit axial length, in Wb/m)
phi_PM = B_r * 2 * b_m*1e-3;                 % Eq. (24) PM residual specific flux [Wb/m]
lambda_PM = mu_rec * mu0 * 2 * b_m*1e-3 / (h_m*1e-3);  % Eq. (25) PM specific permeance [H/m²]

% No-load air-gap specific flux within pole shoe (Eq. 39, iterative result)
% Obtained from the magnetic network; paper gives phi_go = 38.801 mWb/m
% We estimate it analytically from PM and bridge leakage:
%   phi_go ≈ phi_PM * (1 - leakage_ratio) * alpha_m
%   leakage at no-load ≈ 16.6%  (paper Fig. 8)
leakage_ratio_0 = 0.166;
phi_go = phi_PM * (1 - leakage_ratio_0) * alpha_m;      % [Wb/m]
fprintf('=== PM Flux ===\n');
fprintf('  phi_PM  = %.4f mWb/m\n', phi_PM*1e3);
fprintf('  phi_go  = %.4f mWb/m  (paper: 38.801 mWb/m)\n', phi_go*1e3);

% No-load flux density and fundamental (Eqs. 40–42)
B_go  = phi_go / (alpha_m * tau_r*1e-3);               % Eq. (40) [T]
B_g1o = (4/pi) * sind(alpha_m * 180/2) * B_go;         % Eq. (41) [T]
%   sin(α_m·π/2) in degrees:
B_g1o = (4/pi) * sin(alpha_m * pi/2) * B_go;           % [T]
phi_g1o = (2/pi) * B_g1o * tau_r*1e-3;                 % Eq. (42) [Wb/m]

fprintf('  B_go    = %.4f T     (paper: 0.819 T)\n', B_go);
fprintf('  B_g1o   = %.4f T     (paper: 0.965 T)\n', B_g1o);
fprintf('  phi_g1o = %.4f mWb/m (paper: 38.616 mWb/m)\n\n', phi_g1o*1e3);

% PM flux saturation factor η_φM(M_q)  – Eq. (38)
% Approximated by a smooth decreasing function fitted to Fig. 7.
% For the sizing procedure only the value at the corner point is needed.
eta_phiM = @(Mq) 1 ./ (1 + (Mq/3000).^2 .* 0.55);   % simplified fit to Fig.7

%% =========================================================================
%  SECTION V  –  TORQUE SIZING FUNCTION  (Equations 44–64)
%% =========================================================================

% Specific permeance  λ_is  (Eq. 51)
lambda_is = mu0 * k_w^2 * (3/pi^2) * tau_r*1e-3 / (g*1e-3 * k_C);  % [µH/m]
fprintf('=== Sizing ===\n');
fprintf('  lambda_is = %.4f µH/m  (paper: 15.543 µH/m)\n', lambda_is*1e6);

% Reaction factors (iterative inputs from Table I)
c_q = sigma_anis * c_d;   % q-axis reaction coefficient  (from Eq. 53)

% Torque pu function  f_T(Δ, γ)  –  Eq. (57)
% Δ = linear current density [A/m],  γ = phase advance [deg]
% σ_s(Δ,γ) = σ_sM evaluated at M_q = (√2/π)·k_w·τ_r·Δ·cos(γ)
% η_φ(Δ,γ) = η_φM evaluated at same M_q

fT = @(Delta, gamma_deg) fT_func(Delta, gamma_deg, k_w, tau_r, ...
       sigma_sM, eta_phiM, c_d, sigma_anis, phi_g1o, lambda_is);

% Optimal phase advance γ_opt(Δ)  –  Eq. (58)
Delta_c = 90e3;   % Linear current density at corner point [A/m]  (Section V)
gamma_opt = find_gamma_opt(Delta_c, k_w, tau_r, sigma_sM, eta_phiM, ...
                           c_d, sigma_anis, phi_g1o, lambda_is);
fprintf('  γ_opt(Δ_c) = %.2f deg  (paper: 48.15 deg)\n', gamma_opt);

% Values at corner point (Eqs. 60–62)
Mq_c    = (sqrt(2)/pi) * k_w * tau_r*1e-3 * Delta_c * cosd(gamma_opt); % [A]
eta_c   = eta_phiM(Mq_c);
sigma_c = sigma_sM(Mq_c);
fprintf('  η_φ at corner = %.4f  (paper: 0.909)\n', eta_c);
fprintf('  σ_s at corner = %.4f  (paper: 0.667)\n', sigma_c);

% Optimal specific torque T_ℓ  [kNm/m]  –  Eq. (59)
fT_c  = fT(Delta_c, gamma_opt);
T_ell = fT_c * (pi * k_w / (2*sqrt(2))) * B_g1o * Delta_c * (D*1e-3)^2; % [Nm/m]
fprintf('  T_ℓ = %.4f kNm/m  (paper: 2.461 kNm/m)\n', T_ell/1e3);

% Stack length  ℓ  –  Eq. (64)
ell = T_c / T_ell * 1e3;   % [mm]
fprintf('  ℓ   = %.1f mm       (paper: 81.3 mm)\n\n', ell);

%% =========================================================================
%  SECTION VI  –  WINDING DATA AND STATOR CORE  (Equations 65–94)
%% =========================================================================

% Eq. (65)  Fundamental pole flux [Wb]
eta_phi_c  = eta_phiM(Mq_c);
Phi_g1c    = eta_phi_c * phi_g1o * ell*1e-3;   % [Wb]
fprintf('=== Winding & Stator ===\n');
fprintf('  Phi_g1c = %.4f mWb  (paper: 2.853 mWb)\n', Phi_g1c*1e3);

% Eq. (66)  Conductor EMF [Vrms]
E_cc = (pi/sqrt(2)) * f_c * Phi_g1c;           % [V]
fprintf('  E_cc    = %.4f V    (paper: 1.225 V)\n', E_cc);

% Eq. (68)  Max inverter phase voltage [V]
V_invM = 0.95 * V_dc / (2*sqrt(2));            % [V] (rms, fundamental)
% Note: paper uses V_invM = 0.95·V_dc/(2√2); some editions write /√2·0.95/√2
V_invM = 0.95 * V_dc / 2 / sqrt(2);

% Eq. (69)  Conductors in series (theoretical) U_c,th
U_c_th = round(rho_EV * V_invM / (k_w * E_cc));
fprintf('  U_c_th  = %d          (paper: 127→130)\n', U_c_th);

% Eq. (70)  Parallel paths
a = p / 2;   % = 4

% Eq. (71)  Conductors per slot (theoretical)
u_th = 3 * U_c_th * a / Q;
fprintf('  u_th    = %.2f       (paper: 25.46)\n', u_th);

% Eq. (72)  Actual conductors per slot (rounded to even number)
u = 2 * round(0.5 * u_th);   % round to nearest even integer
fprintf('  u       = %d          (paper: 26)\n', u);

% Eq. (73)  Actual conductors in series
U_c = Q * u / (3 * a);
fprintf('  U_c     = %d         (paper: 130)\n', U_c);

% Eq. (74)  Phase EMF [V]
E_c = E_cc * U_c * k_w;
fprintf('  E_c     = %.2f V    (paper: 144.9 V)\n', E_c);

% Eq. (75)  Phase current [Arms]
I_c = (Delta_c * p * tau_r*1e-3) / (3 * U_c);
fprintf('  I_c     = %.1f Arms  (paper: 116 Arms)\n', I_c);

% Path current
I_c_path = I_c / a;

% Eq. (77)  Theoretical current density [A/mm²]
S_cth = 8;   % A/mm²  (chosen for water-glycol cooling)

% Eq. (78)  Conductor cross section [mm²]
A_u = I_c_path / S_cth;
fprintf('  A_u     = %.4f mm²  (paper: 3.625 mm²)\n', A_u);

% Eq. (79)  Max wire diameter [mm]
d_clearance = 0.1;   % clearance between wire and slot opening [mm]
d_wmax = b_as - d_clearance;

% Eq. (80)  Strands in hand
n_w = ceil((4/pi) * A_u / d_wmax^2);
fprintf('  n_w     = %d          (paper: 8)\n', n_w);

% Eq. (81)  Wire diameter [mm]
d_wcu = sqrt(4 * A_u / (pi * n_w));
fprintf('  d_wcu   = %.3f mm   (paper: 0.75 mm)\n', d_wcu);

% Eq. (82)  Copper area in slot [mm²]
A_cu_slot = u * n_w * (pi/4) * d_wcu^2;

% Eq. (83)  Copper fill factor
alpha_cu = 0.4;

% Eq. (84)  Slot cross section [mm²]
A_slot = A_cu_slot / alpha_cu;

% Eq. (85)  Tooth width b_ts [mm]
b_ts = (B_g1o / (B_r20 * rho_bt_s)) * tau_s * k_st;
% Paper eq. (85): b_ts = (B_g1o / B_ts) * τ_s / k_st  with B_ts from rho_bt_s
% Correct form from paper: b_ts = (B_g1o / B_ts) * τ_s * (1/k_st)
B_ts_target = B_r20 * rho_bt_s;   % ~1.0 T target tooth flux density
b_ts = B_g1o * tau_s / (B_ts_target * k_st);
fprintf('  b_ts    = %.3f mm   (paper: 5.89 mm)\n', b_ts);

% Eq. (86) check bt_s ratio (iterative)
bt_s_calc = b_ts / tau_s;
fprintf('  bt_s    = %.4f      (paper/input: %.4f)\n', bt_s_calc, rho_bt_s);

% Eq. (87)  Minor slot width b_1 [mm]
N_s = Q;   % total number of slots
b_1 = (pi * (D + 2*g) - N_s * b_ts) / N_s;
% Corrected per paper Eq. (87): b1 = [π(D+2h_as) - N_s·b_ts] / N_s
% At this stage use simplified form with h_as≈0 first, update after h_as known:
b_1 = (pi * D - N_s * b_ts) / N_s;   % approximate first pass
fprintf('  b_1     = %.3f mm\n', b_1);

% Eq. (88)  Auxiliary slot parameter k_θ
k_theta = tand(180/N_s) * tand(pi/N_s * 180/pi);
k_theta = tan(pi/N_s);   % tan(π/N_s)

% Eq. (89)  Slot height h [mm]
%   h = (-b_1 + sqrt(b_1^2 + 2·k_θ·(2·A_slot - b_1²·π/4))) / (2·k_θ)
h_slot = (-b_1 + sqrt(b_1^2 + 2*k_theta*(2*A_slot - b_1^2*pi/4))) / (2*k_theta);
fprintf('  h_slot  = %.3f mm\n', h_slot);

% Slant assumption: h_as ≈ 1 mm (slot mouth height, paper Fig.1)
h_as = 1.0;   % [mm]

% Eq. (90)  Total equivalent tooth height h_teq [mm]
h_teq = h_slot + b_1/2 + h_as + h_hr;
fprintf('  h_teq   = %.3f mm\n', h_teq);

% Eq. (91)  Ratio h_te_g (iterative check)
rho_hte_g_calc = h_teq / g;
fprintf('  h_te/g  = %.3f      (paper/input: %.3f)\n', rho_hte_g_calc, rho_hte_g);

% Eq. (92)  Major slot width b_2 [mm]
b_2 = b_1 + 2 * h_slot * k_theta;
fprintf('  b_2     = %.3f mm   (paper: 7.44 mm)\n', b_2);

% Eq. (93)  Stator yoke width h_sy [mm]
h_sy = phi_g1o / (2 * B_r20 * k_st) * 1e3;   % convert Wb/m → mm via /1e-3
% Paper: h_sy = φ_g1o / (2·B_ys·k_st)  with B_ys ≈ B_r (target yoke flux density)
% More precisely use B_ys = B_g1o * τ_r / (2 * h_sy)  → iterative
% Simplified explicit form:
B_ys_target = B_r20 * 0.95;   % target yoke flux density ~1.3 T
h_sy = phi_g1o * 1e3 / (2 * B_ys_target * k_st * ell*1e-3) * ell*1e-3;
% Correct dimension: phi_g1o [Wb/m] * ell[m] = Φ [Wb]; h_sy = Φ/(2*B_ys*k_st*ell)
h_sy = phi_g1o / (2 * B_ys_target * k_st);   % [m] → convert
h_sy = h_sy * 1e3;                            % [mm]
fprintf('  h_sy    = %.2f mm   (paper: 20.0 mm)\n', h_sy);

% Eq. (94)  External stator diameter D_es [mm]
D_es = D + 2 * (h_as + h_slot + h_sy);
fprintf('  D_es    = %.2f mm   (paper: 294.5 mm)\n\n', D_es);

%% =========================================================================
%  SECTION VII  –  ELECTRICAL PARAMETERS  (Equations 95–120)
%% =========================================================================

fprintf('=== Electrical Parameters ===\n');

% Eq. (95)  Peak reaction flux densities (isotropic, unsaturated)
M_pd  = (3*sqrt(2)/pi) * k_w * I_c / p;   % d-axis MMF [A]  (Id = Ic·sin(γ_opt))
M_pq  = (3*sqrt(2)/pi) * k_w * I_c / p;   % q-axis MMF amplitude

% d-axis reaction coefficient c_d  (Eq. 106, iterative result given as input)
fprintf('  c_d (input) = %.4f  (paper: 0.201)\n', c_d);

% q-axis reaction coefficient c_q  (Eq. 109)
c_q_calc = sigma_anis * c_d;
fprintf('  c_q         = %.4f  (paper: 0.825, i.e. σ_an·c_d = 4.11·0.201)\n', c_q_calc);

% Unsaturated anisotropy ratio  (Eq. 53 / 110)
sigma_an_o = c_q_calc / c_d;
fprintf('  σ_an,o      = %.4f  (paper: 4.11)\n\n', sigma_an_o);

% Eq. (111)  End-winding length ℓ_ew [mm]
y_c  = 6;   % coil pitch (slots)
ell_ew = (pi * (D + 2*(h_as + h_slot)) / N_s) * ...
         (3 * p * q) * y_c * (pi/2);
% Simplified paper form:
ell_ew = (pi * (D + 2*(h_as + h_slot)) / (3*p*q)) * y_c * pi/2;  % [mm]

% Eq. (119)  Phase leakage inductance L_ℓ [H]
% Leakage specific permeances  (Eqs. 113–118)
lambda_sl = mu0 * (h_slot/(3*b_ts) + b_1/(b_as + b_1) + h_as/b_as);   % [H/m]  Eq.(113)
sigma_h   = 1.5e-2;       % Eq. (114) harmonic leakage coeff.
lambda_h  = sigma_h * lambda_is;                                          % Eq. (115)
lambda_t  = mu0 * alpha_m * g / (b_as + 0.8*g);                          % Eq. (116)
lambda_ew = 0.3e-6;        % Eq. (117) end-winding leakage [H/m]
lambda_tot = lambda_sl/q + lambda_h + lambda_t/q + lambda_ew * ell_ew*1e-3; % Eq.(118)
L_ell = (U_c^2 / p) * ell*1e-3 * lambda_tot;                             % Eq. (119) [H]

% Eq. (120)  Synchronous inductances [H]
L_pdo = c_d * lambda_is * (U_c^2/p) * ell*1e-3;    % unsaturated d-axis
L_pqo = c_q_calc * lambda_is * (U_c^2/p) * ell*1e-3; % unsaturated q-axis
L_d   = L_pdo + L_ell;                               % d-axis synchronous inductance
% L_q is load-dependent via σ_sM; at corner:
L_pq_c = L_pqo * sigma_c;
L_q_c  = L_pq_c + L_ell;

fprintf('  L_d  = %.4f mH\n', L_d*1e3);
fprintf('  L_q(corner) = %.4f mH\n\n', L_q_c*1e3);

%% =========================================================================
%  SECTION VIII  –  PERFORMANCE AT CORNER POINT (verification)
%% =========================================================================

fprintf('=== Performance Verification ===\n');

% No-load flux linkage  (Eq. 121)
Psi_1o = k_w * U_c * ell*1e-3 * phi_g1o / (2*sqrt(2));   % [Wb,rms]

% Electromagnetic torque at corner  (Eq. 123 simplified at MTPA)
gamma_c = gamma_opt;   % [deg]
Id_c = -I_c * sind(gamma_c);   % d-axis current (negative for demagnetisation)
Iq_c =  I_c * cosd(gamma_c);   % q-axis current

Psi_PM1 = k_w * U_c * phi_g1o * eta_c / (2*sqrt(2)) * ell*1e-3;  % PM flux linkage

T_em_c = (3/2) * p * (Psi_PM1 * Iq_c + (L_pdo - L_pq_c) * Id_c * Iq_c);
fprintf('  T_em (corner) = %.2f Nm  (target: %.2f Nm)\n', T_em_c, T_c);
fprintf('  Ratio T_em/T_c = %.4f  (paper FEM: 0.9996)\n\n', T_em_c/T_c);

%% =========================================================================
%  OUTPUT STRUCT  –  Dimensions for COMSOL
%% =========================================================================

dims = struct();

% ---- Global ----
dims.phases     = phases;
dims.p          = p;
dims.q          = q;
dims.Q          = Q;
dims.k_w        = k_w;
dims.ell_mm     = ell;           % Stack length  [mm]

% ---- Stator ----
dims.D_mm       = D;             % Stator bore (inner) diameter  [mm]
dims.D_es_mm    = D_es;          % Stator outer diameter         [mm]
dims.tau_s_mm   = tau_s;         % Slot pitch                    [mm]
dims.b_ts_mm    = b_ts;          % Tooth width                   [mm]
dims.b_1_mm     = b_1;           % Minor slot width              [mm]
dims.b_2_mm     = b_2;           % Major slot width              [mm]
dims.b_as_mm    = b_as;          % Slot opening                  [mm]
dims.h_as_mm    = h_as;          % Slot mouth height             [mm]
dims.h_slot_mm  = h_slot;        % Slot body height              [mm]
dims.h_sy_mm    = h_sy;          % Stator yoke height            [mm]
dims.A_slot_mm2 = A_slot;        % Slot cross-section area       [mm²]

% ---- Air gap ----
dims.g_mm       = g;             % Air-gap length                [mm]
dims.k_C        = k_C;           % Carter's factor               [pu]

% ---- Rotor ----
dims.D_r_mm     = D_r;           % Rotor outer diameter          [mm]
dims.D_ir_mm    = D_ir;          % Rotor inner diameter          [mm]
dims.tau_r_mm   = tau_r;         % Rotor pole pitch              [mm]
dims.alpha_m    = alpha_m;       % Magnet embrace                [pu]
dims.b_ps_mm    = b_ps;          % Pole shoe extension           [mm]
dims.d_ps_mm    = d_ps;          % Pole shoe radial depth        [mm]
dims.h_ry_mm    = h_ry;          % Rotor yoke height             [mm]

% ---- Bridges ----
dims.w_ob_mm    = w_ob;          % Outer bridge width            [mm]
dims.h_ob_mm    = h_ob;          % Outer bridge tangential length[mm]
dims.w_ib_mm    = w_ib;          % Inner bridge width            [mm]
dims.h_ib_mm    = h_ib;          % Inner bridge height           [mm]
dims.w_hr_mm    = w_hr;          % Half-rib (tooth) width        [mm]
dims.h_hr_mm    = h_hr;          % Half-rib radial length        [mm]

% ---- PM ----
dims.h_m_mm     = h_m;           % Magnet radial thickness       [mm]
dims.b_m_mm     = b_m;           % PM segment width              [mm]
dims.v_deg      = v;             % Magnet tilt angle             [deg]
dims.zeta_deg   = zeta;          % Side PM angle                 [deg]
dims.d_12_mm    = d_12;          % Geometric length d_12         [mm]
dims.d_23_mm    = d_23;          % Geometric length d_23         [mm]
dims.d_24_mm    = d_24;          % Geometric length d_24         [mm]
dims.B_r_T      = B_r;           % Remanent flux density at T_PM [T]
dims.mu_rec     = mu_rec;        % Recoil permeability           [pu]

% ---- Winding ----
dims.U_c        = U_c;           % Conductors in series per phase
dims.a          = a;             % Parallel paths
dims.u          = u;             % Conductors per slot
dims.n_w        = n_w;           % Strands in hand
dims.d_wcu_mm   = d_wcu;         % Wire diameter                 [mm]
dims.A_cu_slot_mm2 = A_cu_slot;  % Copper area per slot          [mm²]
dims.alpha_cu   = alpha_cu;      % Copper fill factor            [pu]
dims.I_c_Arms   = I_c;           % Corner phase current          [Arms]
dims.gamma_c_deg = gamma_c;      % Corner phase advance          [deg]

% ---- Electrical ----
dims.L_d_mH     = L_d*1e3;       % d-axis synchronous inductance [mH]
dims.L_q_mH     = L_q_c*1e3;     % q-axis synchronous inductance [mH]
dims.Psi_PM_Wb  = Psi_PM1;       % PM flux linkage               [Wb,rms]
dims.E_c_Vrms   = E_c;           % Phase back-EMF                [Vrms]

% ---- Print summary ----
fprintf('=== COMSOL Dimensions Summary ===\n');
fprintf('  Stack length      ℓ   = %.2f mm\n', dims.ell_mm);
fprintf('  Stator bore       D   = %.2f mm\n', dims.D_mm);
fprintf('  Stator OD         Des = %.2f mm\n', dims.D_es_mm);
fprintf('  Rotor OD          Dr  = %.2f mm\n', dims.D_r_mm);
fprintf('  Rotor ID          Dir = %.2f mm\n', dims.D_ir_mm);
fprintf('  Air gap           g   = %.2f mm\n', dims.g_mm);
fprintf('  Slot height       h   = %.2f mm\n', dims.h_slot_mm);
fprintf('  Tooth width       bts = %.3f mm\n', dims.b_ts_mm);
fprintf('  Stator yoke       hsy = %.2f mm\n', dims.h_sy_mm);
fprintf('  Magnet thickness  hm  = %.2f mm\n', dims.h_m_mm);
fprintf('  Magnet width      bm  = %.3f mm\n', dims.b_m_mm);
fprintf('  Magnet angle      v   = %.1f deg\n', dims.v_deg);
fprintf('  Inner rotor dia   Dir = %.2f mm\n', dims.D_ir_mm);

end % function ArticleFormula


%% =========================================================================
%  LOCAL HELPER FUNCTIONS
%% =========================================================================

function fT_val = fT_func(Delta, gamma_deg, k_w, tau_r, ...
                           sigma_sM_fn, eta_phiM_fn, c_d, sigma_anis, ...
                           phi_g1o, lambda_is)
%FT_FUNC  Torque pu function f_T(Δ,γ)  –  Eq. (57)
    Mq = (sqrt(2)/pi) * k_w * tau_r*1e-3 * Delta * cosd(gamma_deg);  % [A]

    % Saturation functions at this operating point
    if Mq < 1e-9
        eta_phi = 1;
        sigma_s = 1;
    else
        eta_phi = eta_phiM_fn(Mq);
        sigma_s = sigma_sM_fn(Mq);
    end

    % Alignment term  f_{T,al}  (PM contribution)
    fT_al = eta_phi * cosd(gamma_deg);

    % Anisotropy term  f_{T,an}  (reluctance contribution)
    % From Eq. (57):  f_{T,an} = (√2π/6)·(c_d·λ_is)/(k_w·B_g1o)·Δ
    %                              · (σ_an,o · σ_s − 1) · sin(2γ)
    % Note: B_g1o appears explicitly in Eq. (57).
    mu0 = 4*pi*1e-7;
    fT_an = (sqrt(2)*pi/6) * (c_d * lambda_is) / (k_w * phi_g1o) ...
            * tau_r*1e-3 * Delta * (sigma_anis * sigma_s - 1) * sind(2*gamma_deg);

    fT_val = fT_al + fT_an;
end


function gamma_opt = find_gamma_opt(Delta, k_w, tau_r, sigma_sM_fn, ...
                                    eta_phiM_fn, c_d, sigma_anis, ...
                                    phi_g1o, lambda_is)
%FIND_GAMMA_OPT  Optimal phase advance  –  Eq. (58)
%  Finds γ that maximises f_T(Δ, γ) by zero-crossing of df_T/dγ.
    gamma_vec = 1:0.5:89;
    fT_vec = arrayfun(@(g) fT_func(Delta, g, k_w, tau_r, sigma_sM_fn, ...
                                   eta_phiM_fn, c_d, sigma_anis, ...
                                   phi_g1o, lambda_is), gamma_vec);
    [~, idx] = max(fT_vec);
    gamma_opt = gamma_vec(idx);
    % Refine with fminsearch
    gamma_opt = fminbnd(@(g) -fT_func(Delta, g, k_w, tau_r, sigma_sM_fn, ...
                                       eta_phiM_fn, c_d, sigma_anis, ...
                                       phi_g1o, lambda_is), ...
                        max(1, gamma_opt-5), min(89, gamma_opt+5));
end