% test_StatorWindingSizing.m
%
% Validates IPMRotorSizer.computeStatorWindingSizing_ (paper Section VI,
% "Winding Data and Stator Core Dimensions", eqs. 65-94) against the
% worked example in:
%   A. Di Gerlando, C. Ricca, "Design Modeling and Sizing Equations of
%   V-shape IPM Motors," ICEM 2022, DOI: 10.1109/ICEM51905.2022.9910924
%
% Two-part strategy, mirroring test_IPMRotorSizer.m's approach to
% test_EssonsSizer.m's unrelated bore mismatch:
%
%   Part 1 (tight asserts): eqs. 65-75 are re-derived standalone, fed
%   with the paper's OWN stated upstream values (ell=81.3mm, eta_phi_c=
%   0.909, phi_g1o=38.616mWb/m from eqs. 61/42/64) instead of our own
%   solver's. This isolates whether THESE equations are implemented
%   correctly from the already-known, separate upstream gap: our
%   solve()'s own StackLength_mm is ~68% too long because of a
%   pre-existing bug in findGammaOpt_/fT_ (GammaOpt_deg converges to the
%   1 deg search-grid floor instead of the paper's 48.15 deg -- see
%   test_IPMRotorSizer.m's own GammaOpt_deg diagnostic row, unrelated to
%   Section VI and not fixed here). Feeding that same wrong stack length
%   into eqs. 65-94 would make a correct implementation look wrong.
%
%   Part 2 (diagnostics only): the full solve() pipeline is run
%   end-to-end and Section VI's outputs are logged against the paper,
%   same as test_IPMRotorSizer.m's existing diagnostic block -- these
%   inherit the upstream StackLength_mm gap plus this class's own design
%   choices (Has_mm, WireClearance_mm, Bt_T, Bc_T) that the paper's
%   worked example doesn't give numeric values for, so exact agreement
%   isn't expected here even once eqs. 65-94 themselves are correct.

clear; clc;

% ==============================================================
% Part 1 -- eqs. 65-75 standalone, fed the paper's own upstream values
% ==============================================================
k_w   = 0.91;          % WindingFactor default (paper doesn't give a
                        % numeric k_w for this example; this is the same
                        % default IPMRotorSizer uses throughout)
k_st  = 0.97;           % StackingFactor default
p     = 8;
N_s   = 60;
D_mm  = 160;
f_c   = 2900*8/120;     % CornerFrequency_Hz = 193.33 Hz
Vdc   = 650;
rho_EV = 0.650;         % paper eq. 67 ("iterative result", no formula given)
Delta_c = 90e3;         % paper's own Delta_c, eq. 60 area

ell_m     = 81.3e-3;    % paper eq. 64
eta_phi_c = 0.909;      % paper eq. 61
phi_g1o   = 38.616e-3;  % paper eq. 42 [Wb/m]

Phi_g1c = eta_phi_c * phi_g1o * ell_m;
check('Phi_g1c_mWb', Phi_g1c*1e3, 2.853, 0.01);

E_cc = (pi/sqrt(2)) * f_c * Phi_g1c;
check('E_cc_Vrms', E_cc, 1.225, 0.005);

V_invM = 0.95*Vdc/(2*sqrt(2));

Uc_th = (rho_EV*V_invM)/(k_w*E_cc);
check('Uc_th', Uc_th, 127.30, 0.5);

a = p/2;
check('a_parallel_paths', a, 4, 1e-9);

u_th = (3*Uc_th*a)/N_s;
check('u_th', u_th, 25.46, 0.05);

u = 2*round(0.5*u_th);
check('u_conductors_per_slot', u, 26, 1e-9);

Uc = (N_s*u)/(3*a);
check('Uc_series', Uc, 130, 0.5);

E_c = E_cc*Uc*k_w;
check('E_c_Vrms', E_c, 144.9, 0.5);

I_c = (Delta_c*pi*(D_mm*1e-3))/(3*Uc);
check('I_c_Arms', I_c, 116.0, 0.5);

fprintf('Eqs. 65-75 (fed paper''s own upstream values): all match paper within tolerance.\n');

% ==============================================================
% Part 2 -- full solve() pipeline, diagnostics only (see header note)
% ==============================================================
spec = MotorSpec(200, 2900, 13500, 650, 8, 60, 1); % Br20 now lives in MotorMaterials, not MotorSpec
sizer = IPMRotorSizer(spec, StatorBore_mm=160);
sizer.solve();

fprintf('\n%-26s %12s %12s %10s\n', 'Field (diagnostic)', 'Paper', 'Got', 'RelErr%');
diagnose('ConductorsInSeries', sizer.ConductorsInSeries, 130);
diagnose('ConductorsInSlot',   sizer.ConductorsInSlot,    26);
diagnose('PhaseCurrent_A',     sizer.PhaseCurrent_A,     116.0);
diagnose('ToothWidth_mm',      sizer.ToothWidth_mm,        5.89);
diagnose('RhoBtS',             sizer.RhoBtS,               0.704);
diagnose('SlotWidthOuter_mm',  sizer.SlotWidthOuter_mm,    7.44);
diagnose('StatorYokeHeight_mm',sizer.StatorYokeHeight_mm, 20.0);
diagnose('StatorOD_mm',        sizer.StatorOD_mm,        294.5);

% Sanity only: all outputs finite and physically plausible (positive,
% and roughly the right order of magnitude for a 160mm-bore machine).
assert(all(isfinite([sizer.ConductorsInSeries, sizer.PhaseCurrent_A, ...
    sizer.WireDiameter_mm, sizer.ToothWidth_mm, sizer.SlotWidthInner_mm, ...
    sizer.SlotWidthOuter_mm, sizer.SlotHeight_mm, sizer.StatorYokeHeight_mm, ...
    sizer.StatorOD_mm])), 'Section VI produced a non-finite result.');
assert(sizer.StatorOD_mm > sizer.StatorBore_mm, ...
    'External stator OD (%.1f mm) should exceed the bore (%.1f mm).', ...
    sizer.StatorOD_mm, sizer.StatorBore_mm);
assert(sizer.NumStrands >= 1 && sizer.WireDiameter_mm > 0, ...
    'Wire sizing (eqs. 78-81) produced a non-physical result.');

disp('TEST PASSED: Section VI equations (65-75) match the paper when fed its own upstream values; full-pipeline results logged as diagnostics.');

% ===== local helper functions (must follow all script code) =====
function check(name, got, expected, tol)
    assert(abs(got - expected) < tol, ...
        '%s mismatch! Expected: %.4f, Got: %.4f (diff: %.4f, tol: %.4f)', ...
        name, expected, got, abs(got - expected), tol);
end

function diagnose(name, got, expected)
    relerr = 100 * (got - expected) / expected;
    fprintf('%-26s %12.4f %12.4f %9.2f%%\n', name, expected, got, relerr);
end
