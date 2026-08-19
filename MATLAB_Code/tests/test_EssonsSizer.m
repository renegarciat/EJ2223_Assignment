% Compare EssonsSizer outputs against paper reference values
% Compare with at least the values required by the COMSOL model
% FIXME: This test is currently failing. EssonsSizer outputs a stator bore different from 160 mm.
% Maybe consider decoupling IPMRotorSizer and EssonsSizer, letting IPMRotorSizer take the stator bore as an input.
%
% NOTE: solveD2L() (formerly solve()) was simplified to the plain D^2
% formula -- it now only computes Dis, le, tau_p, Dro. The slot-geometry/
% Dos outputs this test used to check (q_spp, tau_s_m, t_s_m, h_slot_m,
% h_cs_m, Dos_m, Ratio) no longer exist on EssonsSizer, so those
% assertions were removed rather than left to hard-crash on a missing
% property. The Dis mismatch above is the original, still-open issue
% this test tracks -- unrelated to that simplification.
clear; clc;
% Paper reference inputs
airgap_mm = 1; % [mm]
torque_Nm = 200; % [Nm]
cornerSpeed_rpm = 2900; % [rpm]
maxSpeed_rpm = 13500; % [rpm]
vdcLink_V = 650; % [V]
poles = 8;
slots = 60;
% Br20 now lives in MotorMaterials, not MotorSpec (unused by this test anyway)
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
Dro_m    = 158.0e-3;

% Tolerances (adjusted to SI units matching your class definitions)
tol_m = 1.5e-3;      % 1.5 mm tolerance for absolute mechanical lengths

% Computation
spec = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, ...
    vdcLink_V, poles, slots, ...
    airgap_mm, AspectRatio = AspectRatio);
sizer = EssonsSizer(spec);
sizer.solveD2L();

% Assertions
assert(abs(sizer.Dis_m - Dis_m) < tol_m, ...
    'Dis (Inner Stator Bore) out of tolerance! Expected: %.2f mm, Got: %.2f mm (Diff: %.2f mm)', ...
    Dis_m * 1e3, sizer.Dis_m * 1e3, abs(sizer.Dis_m - Dis_m) * 1e3);
assert(abs(sizer.le_m - le_m) < tol_m);
assert(abs(sizer.tau_p_m - tau_p_m) < tol_m);
assert(abs(sizer.Dro_m - Dro_m) < tol_m);
disp('TEST PASSED: EssonsSizer matches paper reference values within tolerances.');