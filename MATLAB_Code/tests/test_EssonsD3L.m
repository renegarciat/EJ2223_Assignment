% test_EssonsD3L.m
%
% Validates EssonsSizer.solveD3L (the "D^3L output coefficient" method,
% Lecture 6 "Improving the Esson's rule", L. Peretti) -- the standalone
% second Esson-family sizing estimate: fix Dos (a packaging constraint)
% and find the Lagrangian/KKT-optimal Dis, then solve for le hitting the
% torque target. This is independent of solveD2L() (the D^2 formula),
% which goes the other direction (torque -> Dis, le, with the aspect
% ratio as an input rather than an output).
%
% Cross-check against the paper's own design point (Di Gerlando & Ricca,
% ICEM 2022): feeding the paper's actual Dos=294.5mm and T=200Nm should
% recover a Dis close to the paper's actual 160mm, even though solveD3L
% never sees Dis directly -- it only takes Dos as input.
%
% NOTE: this test used to also check self-consistency between solveD3L()
% and solve() (the D^2 formula) at a shared Dos -- solve() used to
% back-solve a self-consistent Dos from a given Dis via the same
% Ks,rms(Dis,Dos) relationship solveD3L() root-finds on. That crossover
% was removed when solveD2L() was simplified to the plain D^2 formula
% (Dis, le only, no Dos/slot-geometry output), so there is no longer a
% Dos to cross-check against here -- solveD2L() and solveD3L() are now
% fully independent methods, which is the intended simplification.

clear; clc;

spec = MotorSpec(200, 2900, 13500, 650, 8, 60, 1); % Br20 now lives in MotorMaterials, not MotorSpec
sizer = EssonsSizer(spec);
sizer.solveD3L(294.5e-3, MakePlot=false); % keep automated test runs headless

checkRel('D3L Dis (paper cross-check)', sizer.Dis_D3L_m*1e3, 160.0, 0.05);
assert(sizer.le_D3L_m > 0 && isfinite(sizer.le_D3L_m), ...
    'le_D3L_m must be a positive finite value, got %g', sizer.le_D3L_m);
assert(sizer.Dis_D3L_m > 0 && sizer.Dis_D3L_m < sizer.Dos_D3L_m, ...
    'Dis_D3L_m must lie strictly within (0, Dos).');
assert(sizer.FoAtOptimum_D3L > 0, 'fo(Dis/Dos) must be positive at the optimum.');
fprintf('D3L paper cross-check: Dis=%.2fmm (paper 160.0mm), le=%.2fmm\n', ...
    sizer.Dis_D3L_m*1e3, sizer.le_D3L_m*1e3);

disp('TEST PASSED: EssonsSizer.solveD3L matches the paper reference point.');

% ===== local helper functions (must follow all script code) =====
function checkRel(name, got, expected, tolRel)
    relerr = abs(got - expected) / abs(expected);
    assert(relerr < tolRel, ...
        '%s mismatch! Expected: %.4f, Got: %.4f (relErr: %.2f%%, tol: %.2f%%)', ...
        name, expected, got, relerr*100, tolRel*100);
end
