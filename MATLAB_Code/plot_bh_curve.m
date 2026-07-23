%% plot_bh_curve.m
% Plots the M235-35A BH curve dataset used internally by
% IPMRotorSizer's saturation model, so it can be eye-compared against
% the paper's own Fig. 2 (references/fig_2_ricca.png). The dataset
% (IPMRotorSizer.BFE_DATA/HFE_DATA) is itself digitized directly from
% that figure -- see the comment above those constants for the
% digitization method.
%
% Left panel: mu_fe.pu vs B, same axes/scale as paper Fig. 2 -- put this
% side by side with references/fig_2_ricca.png for the 1:1 comparison.
% Right panel: H_fe(B_fe) -- H as a function of B, matching the
% orientation IPMRotorSizer.computeSaturationModel_ actually uses
% internally (Hfe = @(B) Hfe_interp_(B)). Not a curve shown in the paper.
clear; clc;

[B_T, H_Am] = IPMRotorSizer.bhCurveData();

% Same interpolant IPMRotorSizer actually uses internally (pchip).
bh_interp = griddedInterpolant(B_T, H_Am, 'pchip');
B_fine = linspace(0, max(B_T), 1000);
H_fine = bh_interp(B_fine);

% Paper Fig. 2 plots relative (pu) permeability mu_fe.pu = B/(mu0*H) vs
% B, not H vs B directly. Undefined at B=0 (H=0), so skip that point.
mu0 = 4*pi*1e-7; % [H/m]

fine_mask = B_fine > 0;
mu_fe_pu_fine = B_fine(fine_mask) ./ (mu0 * H_fine(fine_mask));

pts_mask = B_T > 0;
mu_fe_pu_pts = B_T(pts_mask) ./ (mu0 * H_Am(pts_mask));

figure('Name', 'M235-35A BH Curve', 'Position', [100 100 1100 480]);

subplot(1,2,1);
plot(B_fine(fine_mask), mu_fe_pu_fine, ...
    '-', 'LineWidth', 1.5, 'DisplayName', 'pchip interpolant (used by solver)');
hold on;
plot(B_T(pts_mask), mu_fe_pu_pts, 'o', 'MarkerSize', 6, ...
    'MarkerFaceColor', 'w', 'DisplayName', 'dataset points');
hold off;
grid on;
set(gca, 'YScale', 'log');
xlabel('B [T]');
ylabel('\mu_{fe.pu}');
title('Extended curve of pu permeability of M235-35A (paper Fig. 2)');
legend('Location', 'northeast');
ylim([1, 10000]);
xlim([0, 3]);

subplot(1,2,2);
plot(B_fine(fine_mask), H_fine(fine_mask), '-', 'LineWidth', 1.5, ...
    'DisplayName', 'pchip interpolant (used by solver)');
hold on;
plot(B_T(pts_mask), H_Am(pts_mask), 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'w', ...
    'DisplayName', 'dataset points');
hold off;
grid on;
set(gca, 'YScale', 'log');
xlabel('B_{fe} [T]');
ylabel('H_{fe} [A/m]');
title('H_{fe}(B_{fe})');
legend('Location', 'northwest');
xlim([0, 3]);

sgtitle('M235-35A BH curve — digitized from paper Fig. 2, used by IPMRotorSizer.computeSaturationModel\_');

fprintf('Dataset points (B [T], H [A/m]):\n');
for i = 1:numel(B_T)
    fprintf('  %5.2f  %8.1f\n', B_T(i), H_Am(i));
end
