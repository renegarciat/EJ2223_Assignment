% Compare EssonsSizer outputs against paper reference values
% Compare with at least the values required by the COMSOL model
% FIXME: This test is currently failing. EssonsSizer outputs a stator bore different from 160 mm.
% Maybe consider decoupling IPMRotorSizer and EssonsSizer, letting IPMRotorSizer take the stator bore as an input.
clear; clc;
% Paper reference inputs
airgap_mm = 1; % [mm]
torque_Nm = 200; % [Nm]
cornerSpeed_rpm = 2900; % [rpm]
maxSpeed_rpm = 13500; % [rpm]
vdcLink_V = 650; % [V]
poles = 8;
slots = 60;
br20_T = 1.37; % [T]
% LinearCurrentDensity_Am = NaN;
% CurrentDensity_Amm2 = NaN;
% EfficiencyEstimate = NaN;
% PowerFactor = NaN;
% Airgap_mm = NaN; % [mm]
% StackingFactor = NaN;
% IronFillFactor = NaN;
% SlotOpening_mm = NaN;
% CopperFillFactor = NaN;

% Paper reference results
Dis_m = 160e-3; % [m] inner stator diameter
le_m = 81.3e-3; % [m] effective length
tau_p_m = (pi * Dis_m) / poles;    % [m] pole pitch
AspectRatio = le_m/tau_p_m; % [-] le/tau_p ratio
q = slots / (poles * 3); % slots per pole per phase
tau_s_m  = (pi * Dis_m) / slots;   % [m] stator pitch
t_s_m    = 0.704 * tau_s_m;   % stator tooth distance
h_slot_m = 45.25e-3;   
h_cs_m   = 20.0e-3;   
Dro_m    = 158.0e-3;   
Dso_m    = 294.5e-3;
Ratio = Dis_m / Dso_m;

% Tolerances (adjusted to SI units matching your class definitions)
tol_m = 1.5e-3;      % 1.5 mm tolerance for absolute mechanical lengths
tol_rel = 0.05;      % 5% relative tolerance for non-linear solver bounds

% Computation
spec = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, ...
    vdcLink_V, poles, slots, ...
    airgap_mm, br20_T, AspectRatio = AspectRatio);
sizer = EssonsSizer(spec);
sizer.solve();

% Assertions
assert(abs(sizer.q_spp - q) < tol_rel * q, ...
    'q_spp Mismatch! Expected: %.4f, Got: %.4f', q, sizer.q_spp);
assert(abs(sizer.Dis_m - Dis_m) < tol_m, ...
    'Dis (Inner Stator Bore) out of tolerance! Expected: %.2f mm, Got: %.2f mm (Diff: %.2f mm)', ...
    Dis_m * 1e3, sizer.Dis_m * 1e3, abs(sizer.Dis_m - Dis_m) * 1e3);
assert(abs(sizer.le_m - le_m) < tol_m);
assert(abs(sizer.tau_p_m - tau_p_m) < tol_m);
assert(abs(sizer.tau_s_m - tau_s_m) < tol_m);
assert(abs(sizer.t_s_m - t_s_m) < tol_m);
assert(abs(sizer.h_slot_m - h_slot_m) < tol_m);
assert(abs(sizer.h_cs_m - h_cs_m) < tol_m);
assert(abs(sizer.Dro_m - Dro_m) < tol_m);
assert(abs(sizer.Dso_m - Dso_m) < tol_m);
assert(abs(sizer.Ratio - Ratio) < tol_rel * Ratio);
disp('TEST PASSED: EssonsSizer matches paper reference values within tolerances.');