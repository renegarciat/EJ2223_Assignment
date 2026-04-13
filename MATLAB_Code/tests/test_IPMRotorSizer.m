
% Compare IPMRotorSizer outputs against paper reference values
% Compare with at least the values required by the COMSOL model
% FIXME: This test is currently failing. EssonsSizer outputs a stator bore different from 160 mm. IPMRotorSizer depends on that value.
% Paper reference inputs
airgap_mm = 1; % [mm]
torque_Nm = 200; % [Nm]
cornerSpeed_rpm = 2900; % [rpm]
maxSpeed_rpm = 13500; % [rpm]
vdcLink_V = 650; % [V]
poles = 8;
slots = 60;
q = slots / (poles * 3); % slots per pole per phase
br20_T = 1.37; % [T]

spec = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, ...
    vdcLink_V, poles, slots, ...
    airgap_mm, br20_T);

% Stator bore is now derived by EssonsSizer.
ess = EssonsSizer(spec);
ess.solve();
sizer = IPMRotorSizer(spec);
sizer.solve();

% Paper reference results
h_m = 6; % PM segment height [mm]
w_ib = 2.5; % [mm]
D = 160; % [mm] stator bore diameter
D_r = D - 2 * airgap_mm; % rotor bore diameter [mm]
h_ry = 6.9; % = 1.5*w_hr [mm]
angle_m = 78; % [deg]
D_ir = 115.4;   % (17) internal rotor diameter [mm]
b_m = 22.1; % (18) PM segment width [mm]
% Hob_mm = 3.0;
% Hib_mm = 5.9;
% Hhr_mm = 5.2;
% Dps_mm = 8.5;
% RotorID_mm = 115.4;
% PhiGo_mWbpm = 38.801;
% B_go_T = 0.819;
% B_g1o_T = 0.965;
% PhiG1o_mWbpm = 38.616;
% GammaOpt_deg = 48.15;
% EtaPhi_c = 0.909;
% SigmaS_c = 0.667;
% SpecificTorque_kNmm = 2.461;
% StackLength_mm = 81.3;
% Cd = 0.201;
% Cq = 0.825;
% SigmaAnis = 4.11;
% LambdaIs_uHm = 15.543;

% Tolerances
tol_mm = 1.0;        % mm tolerance for dimensional values
tol_phi = 2.0;       % mWb/m tolerance
tol_T = 0.1;         % Tesla tolerance
tol_deg = 5.0;       % degrees
tol_rel = 0.1;       % relative tolerance for ratios/coeffs
tol_kNm = 0.5;       % specific torque
tol_uH = 5.0;        % microhenry per meter

% Compare values (n)
assert(abs(sizer.StatorBore_mm - D) < tol_mm, ...
    'Stator Bore Mismatch! Expected: %.1f mm, Got: %.1f mm (Diff: %.1f mm)', ...
    D, sizer.StatorBore_mm, abs(sizer.StatorBore_mm - D));
assert(abs(sizer.RotorID_mm - D_ir) < tol_mm);
assert(abs(sizer.Hm_mm - h_m) < tol_mm);
assert(abs(sizer.Bm_mm - b_m ) < tol_mm);
assert(abs(sizer.RotorOD_mm - D_r) < tol_mm);
assert(abs(sizer.Wib_mm - w_ib) < tol_mm);
assert(abs(sizer.Hry_mm - h_ry) < tol_mm);
assert(abs(sizer.Vtilt_deg - angle_m) < tol_deg);
disp('TEST PASSED: IPMRotorSizer matches paper reference values within tolerances.');