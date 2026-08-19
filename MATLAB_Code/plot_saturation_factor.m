%% plot_saturation_factor.m
% Plots sigma_sM(Mq) -- the q-axis saturation factor, paper eq. (7) --
% so it can be eye-compared against the paper's own Fig. 3 (saturation
% factor vs. peak q-axis MMF Mq, 0-10 kA).
%
% sigma_sM(Mq) is built inside IPMRotorSizer.computeSaturationModel_
% (eqs. 3-7 chained together) and exposed via the public
% IPMRotorSizer.saturationAt(Mq_A) method -- this script just sweeps it.
%
% Uses the paper's own reference design point (Table I) with the bore
% pinned directly (StatorBore_mm=160) -- same rationale as
% tests/test_IPMRotorSizer.m: isolates IPMRotorSizer's own equations
% from EssonsSizer's unrelated bore mismatch (README.md's "Known issues" section).
%
% Caveat: RhoBtS drifts from the paper's converged Table I value (0.704)
% to ~0.4847 during solve()'s iteration -- computeElectricalParameters_
% derives it as B_g1o/(Bt_T*k_st) (a flux-density-target ratio, see
% IPMRotorSizer.m:787-792), which is a known heuristic stage independent
% of eq. (3)/(7) themselves. This can still shift this curve somewhat
% from Fig. 3 even with eqs. (3) and (7) now both implemented per the
% paper (implicit root-solve, and the corrected sigma_sM formula).
clear; clc;

spec = MotorSpec(200, 2900, 13500, 650, 8, 60, 1); % Br20 now lives in MotorMaterials, not MotorSpec
sizer = IPMRotorSizer(spec, StatorBore_mm=160);
sizer.solve();

fprintf('Iterative params at convergence (paper Table I in parens):\n');
fprintf('  RhoBtS   = %.4f  (0.704)\n', sizer.RhoBtS);
fprintf('  RhoHteG  = %.4f  (52.4)\n', sizer.RhoHteG);
fprintf('  CarterFactor kC = %.4f  (1.071)\n', sizer.CarterFactor);

% Mq=0 is a removable-singularity edge case for the root-solve behind
% eq. (6)/(7) (both B(Mq) and the denominator vanish there); start just
% above zero instead of exactly at it.
Mq_kA = linspace(1e-3, 10, 300);
Mq_A  = Mq_kA * 1e3;
sigma_sM = arrayfun(@(m) sizer.saturationAt(m), Mq_A);

figure('Name', 'Saturation factor sigma_sM(Mq)', 'Position', [100 100 650 480]);
plot(Mq_kA, sigma_sM, '-', 'LineWidth', 1.5, 'Color', [0.85 0.1 0.1]);
grid on;
xlabel('M_{\rho q} [kA]');
ylabel('\sigma_{sM} [pu]');
title('Saturation factor \sigma_{sM}(M_{\rho q}) — eq. (7) (compare vs. paper Fig. 3)');
xlim([0, 10]);
ylim([0, 1]);

fprintf('\nsigma_sM at a few Mq values:\n');
for m_kA = [0, 0.5, 1, 2, 3, 5, 7, 10]
    fprintf('  Mq=%5.1f kA  sigma_sM=%.4f\n', m_kA, sizer.saturationAt(m_kA*1e3));
end
