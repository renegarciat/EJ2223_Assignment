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
% and README.md's "Known issues" section). Pinning the bore isolates IPMRotorSizer's own
% equations from that unrelated mismatch.
%
% Scope: the rotor geometry equations (paper eqs. 8-18) and the electrical
% reaction coefficients (Cd, Cq, SigmaAnis -- eqs. 96-110, now real closed-
% form integrals of the paper's eqs. 98/107 flux distributions, not
% approximations) are asserted with tight tolerances -- both verified to
% match the paper to <1%.
%
% Still diagnostic-only, not asserted:
%   - Bg1o_T/PhiGo_Wbm/PhiG1o_Wbm (~13% off): computeSaturationModel_'s
%     eq. 38 (eta_phiM cross-coupling network) and the no-load leakage
%     flux (eqs. 24-30) are still approximations -- see that method's
%     comments. This is the dominant remaining error source and cascades
%     into everything downstream of it (GammaOpt_deg, StackLength_mm,
%     SpecificTorque_kNmm are 10-20% off as a result, down from 40-98%
%     before the eq. 57/98/104-109 fixes).
%   - LambdaIs_uHm (~18% off): not yet investigated.

clear; clc;

% ---- Paper reference design parameters (Fig. 1, Table I) ----
airgap_mm       = 1;    % [mm]
torque_Nm       = 200;  % [Nm]
cornerSpeed_rpm = 2900; % [rpm]
maxSpeed_rpm    = 13500;% [rpm]
vdcLink_V       = 650;  % [V]
poles           = 8;
slots           = 60;
% Br20 now lives in MotorMaterials (default 1.37T, matches this test's
% old br20_T), not MotorSpec.

spec = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, ...
    vdcLink_V, poles, slots, airgap_mm);

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
% formula (magnet-only mass; see README.md's "Rotor bridge stress check"
% section), so we can't pin an exact match to the paper's ratio (0.783). Sanity-check
% instead: positive, finite, plausible order of magnitude.
assert(sizer.FMaxSpecific_Nm > 0 && isfinite(sizer.FMaxSpecific_Nm), ...
    'f_max should be a positive finite value, got %g', sizer.FMaxSpecific_Nm);
assert(sizer.SigmaIbRatio > 0 && sizer.SigmaIbRatio < 3, ...
    'sigma_ib/sigma_y_lam = %.3f is outside a plausible range (paper reference: 0.783)', ...
    sizer.SigmaIbRatio);
fprintf('Bridge stress ratio sigma_ib/sigma_y_lam = %.3f (paper reference: 0.783)\n', ...
    sizer.SigmaIbRatio);

% ---- Electrical reaction coefficients (eqs. 96-110) -- tight asserts ----
% Real closed-form integrals of the paper's piecewise flux distributions
% (eqs. 98, 107), not approximations -- verified to match paper's Table I
% to <1%.
tol_rel = 0.02; % 2% relative tolerance
checkRel('Cd',        sizer.Cd,        0.201, tol_rel);
checkRel('Cq',        sizer.Cq,        0.825, tol_rel);
checkRel('SigmaAnis', sizer.SigmaAnis, 4.11,  tol_rel);
fprintf('Electrical reaction coefficients (eqs. 96-110): Cd, Cq, SigmaAnis within %.0f%% of paper.\n', tol_rel*100);

% ---- Saturation model / torque sizing -- diagnostics only ----
% NOT asserted: still bottlenecked by the eq. 38/24-30 approximations in
% computeSaturationModel_ (see header note). Logged so future
% improvements/regressions are visible.
fprintf('\n%-22s %12s %12s %10s\n', 'Field (diagnostic)', 'Paper', 'Got', 'RelErr%');
diagnose('PhiGo_Wbm',          sizer.PhiGo_Wbm,          38.801e-3);
diagnose('Bg1o_T',             sizer.Bg1o_T,              0.965);
diagnose('PhiG1o_Wbm',         sizer.PhiG1o_Wbm,         38.616e-3);
diagnose('GammaOpt_deg',       sizer.GammaOpt_deg,       48.15);
diagnose('EtaPhi_c',           sizer.EtaPhi_c,            0.909);
diagnose('SigmaS_c',           sizer.SigmaS_c,            0.667);
diagnose('SpecificTorque_kNmm',sizer.SpecificTorque_kNmm, 2.461);
diagnose('StackLength_mm',     sizer.StackLength_mm,     81.3);
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

function checkRel(name, got, expected, tolRel)
    relerr = abs(got - expected) / abs(expected);
    assert(relerr < tolRel, ...
        '%s mismatch! Expected: %.4f, Got: %.4f (relErr: %.2f%%, tol: %.2f%%)', ...
        name, expected, got, relerr*100, tolRel*100);
end
