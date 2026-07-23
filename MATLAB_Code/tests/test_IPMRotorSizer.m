% test_IPMRotorSizer.m
%
% Validates IPMRotorSizer's rotor geometry equations against the worked
% example (Fig. 4 design point) in:
%   A. Di Gerlando, C. Ricca, "Design Modeling and Sizing Equations of
%   V-shape IPM Motors," ICEM 2022, DOI: 10.1109/ICEM51905.2022.9910924
%   (references/Design_Modeling_and_Sizing_Equations_of_V-shape_IPM_Motors.pdf)
%
% Bore is pinned directly via StatorBore_mm=160 instead of being derived
% from EssonsSizer: EssonsSizer implements a different sizing methodology
% (Esson's rule / Lipo book, see EssonsSizer.m) that does not reproduce
% the paper's D=160mm design point (see test_EssonsSizer.m's own FIXME
% and doc/bug_002.md). Pinning the bore isolates IPMRotorSizer's own
% equations from that unrelated mismatch.
%
% Scope: only the rotor geometry equations (paper eqs. 8-18) are asserted
% with tight tolerances -- verified to match the paper to <1%. The
% saturation model / torque sizing / electrical parameter stages (eqs.
% 38-120) are current known heuristic placeholders -- see the comments in
% IPMRotorSizer.computeSaturationModel_ ("smooth analytical fit calibrated
% to paper Fig. 7", not the real magnetic-network model of Fig. 5) and
% computeElectricalParameters_ ("Not updated here; a stator/winding model
% should provide this"). They do not yet reproduce the paper (15-100% off)
% and are logged as diagnostics only, not asserted, pending a future
% implementation pass.

clear; clc;

% ---- Paper reference design parameters (Fig. 1, Table I) ----
airgap_mm       = 1;    % [mm]
torque_Nm       = 200;  % [Nm]
cornerSpeed_rpm = 2900; % [rpm]
maxSpeed_rpm    = 13500;% [rpm]
vdcLink_V       = 650;  % [V]
poles           = 8;
slots           = 60;
br20_T          = 1.37; % [T]

spec = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, ...
    vdcLink_V, poles, slots, airgap_mm, br20_T);

% D=160mm pinned directly (see header note above). Default rotor design
% choices already match the paper's Fig. 4 point (AlphaM=0.754,
% Whr_fraction=0.55, Hm_mm=6, Vtilt_deg=78, Wib_mm=2.5) so no overrides
% are needed for those.
sizer = IPMRotorSizer(spec, StatorBore_mm=160);
sizer.solve();

% ---- Rotor geometry reference values (paper eqs. 8-18) ----
% Tight tolerance: largest observed deviation from the paper is ~0.04mm.
tol_mm = 0.1; % [mm]

check('RotorOD_mm', sizer.RotorOD_mm, 158.0, tol_mm);
check('RotorID_mm', sizer.RotorID_mm, 115.4, tol_mm);
check('Hob_mm',     sizer.Hob_mm,       3.0, tol_mm);
check('Hib_mm',     sizer.Hib_mm,       5.9, tol_mm);
check('Hhr_mm',     sizer.Hhr_mm,       5.2, tol_mm);
check('Dps_mm',     sizer.Dps_mm,       8.5, tol_mm);
check('Bm_mm',      sizer.Bm_mm,       22.1, tol_mm);
check('Hry_mm',     sizer.Hry_mm,       6.9, tol_mm);

fprintf('Rotor geometry (eqs. 8-18): all fields within %.2f mm of paper reference.\n', tol_mm);

% ---- Rotor bridge stress check (eqs. 19-22) -- sanity only ----
% m_1p/R_av are our own first-order approximation of an unstated paper
% formula (magnet-only mass; see doc/rotor_bridge_stress_notes.md), so we
% can't pin an exact match to the paper's ratio (0.783). Sanity-check
% instead: positive, finite, plausible order of magnitude.
assert(sizer.FMaxSpecific_Nm > 0 && isfinite(sizer.FMaxSpecific_Nm), ...
    'f_max should be a positive finite value, got %g', sizer.FMaxSpecific_Nm);
assert(sizer.SigmaIbRatio > 0 && sizer.SigmaIbRatio < 3, ...
    'sigma_ib/sigma_y_lam = %.3f is outside a plausible range (paper reference: 0.783)', ...
    sizer.SigmaIbRatio);
fprintf('Bridge stress ratio sigma_ib/sigma_y_lam = %.3f (paper reference: 0.783)\n', ...
    sizer.SigmaIbRatio);

% ---- Saturation / torque / electrical stages -- diagnostics only ----
% NOT asserted: known heuristic placeholders (see header note), don't yet
% match the paper. Logged so future improvements/regressions are visible.
fprintf('\n%-22s %12s %12s %10s\n', 'Field (diagnostic)', 'Paper', 'Got', 'RelErr%');
diagnose('PhiGo_Wbm',          sizer.PhiGo_Wbm,          38.801e-3);
diagnose('Bg1o_T',             sizer.Bg1o_T,              0.965);
diagnose('PhiG1o_Wbm',         sizer.PhiG1o_Wbm,         38.616e-3);
diagnose('GammaOpt_deg',       sizer.GammaOpt_deg,       48.15);
diagnose('EtaPhi_c',           sizer.EtaPhi_c,            0.909);
diagnose('SigmaS_c',           sizer.SigmaS_c,            0.667);
diagnose('SpecificTorque_kNmm',sizer.SpecificTorque_kNmm, 2.461);
diagnose('StackLength_mm',     sizer.StackLength_mm,     81.3);
diagnose('Cd',                 sizer.Cd,                  0.201);
diagnose('Cq',                 sizer.Cq,                  0.825);
diagnose('SigmaAnis',          sizer.SigmaAnis,           4.11);
diagnose('LambdaIs_uHm',       sizer.LambdaIs_uHm,       15.543);

disp('TEST PASSED: IPMRotorSizer rotor geometry (eqs. 8-18) matches paper reference values within tolerance.');

% ===== local helper functions (must follow all script code) =====
function check(name, got, expected, tol)
    assert(abs(got - expected) < tol, ...
        '%s mismatch! Expected: %.4f, Got: %.4f (diff: %.4f, tol: %.4f)', ...
        name, expected, got, abs(got - expected), tol);
end

function diagnose(name, got, expected)
    relerr = 100 * (got - expected) / expected;
    fprintf('%-22s %12.4f %12.4f %9.2f%%\n', name, expected, got, relerr);
end
