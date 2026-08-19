%% rotor_bridge_stress_sweep.m
% Sweeps the inner bridge width (Wib_mm) and plots the resulting
% centrifugal stress at max speed against the lamination yield strength.
%
% Implements Di Gerlando & Ricca, ICEM 2022, Section IV (eqs. 19-22).
% See README.md's "Rotor bridge stress check" section for full sourcing
% and the known simplification (m_1p is magnet-only, pole-shoe iron
% neglected).
%
% Uses this project's actual main.m design point (9.8 Nm, 10-pole/12-slot,
% n_max=14,400 rpm, AlphaM=0.80, Whr_fraction=0.1, Hm_mm=3), not the
% reference paper's own worked example -- keep these six values in sync
% with main.m if that design point ever changes.
clear; clc;

torque_Nm       = 9.8;
maxSpeed_rpm    = 14400;
cornerSpeed_rpm = int32(round(maxSpeed_rpm*0.85));
vdcLink_V       = 515;
poles           = 10;
slots           = 12;
airgap_mm       = 1; % Br20 now lives in MotorMaterials (default 1.37T), not MotorSpec
AlphaM          = 0.80;
Whr_fraction    = 0.1;
Hm_mm           = 3;
AspectRatio     = 2.0; % must match main.m -- MotorSpec's default (1.0) gives a different bore/geometry

spec = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, ...
                  vdcLink_V, poles, slots, airgap_mm, AspectRatio = AspectRatio);
materials = MotorMaterials();

wib_sweep_mm = linspace(0.5, 6.0, 25);
sigma_ib_MPa = nan(size(wib_sweep_mm));

for i = 1:numel(wib_sweep_mm)
    sizer = IPMRotorSizer(spec, AlphaM=AlphaM, Whr_fraction=Whr_fraction, ...
                           Hm_mm=Hm_mm, Wib_mm=wib_sweep_mm(i), ...
                           Materials=materials);
    try
        sizer.solve();
        sigma_ib_MPa(i) = sizer.SigmaIb_MPa;
    catch ME
        % Infeasible geometry at this Wib_mm (e.g. h_ob <= 0) — leave NaN
        fprintf('Wib_mm=%.2f infeasible: %s\n', wib_sweep_mm(i), ME.message);
    end
end

% Current design point
sizer0 = IPMRotorSizer(spec, AlphaM=AlphaM, Whr_fraction=Whr_fraction, ...
                        Hm_mm=Hm_mm, Materials=materials);
sizer0.solve();

% Manufacturing-driven rule of thumb from the paper: w_ib = 5*w_ob,
% w_ob floored at w_lam = 0.35 mm  =>  w_ib_min ~ 1.75 mm
w_ib_manuf_min_mm = 5 * 0.35;

figure('Name', 'Rotor Inner Bridge Stress Sweep');
plot(wib_sweep_mm, sigma_ib_MPa, '-o', 'LineWidth', 1.5, 'DisplayName', '\sigma_{ib}(w_{ib})');
hold on;
yline(materials.sigma_y_lam * 1e-6, 'r--', 'LineWidth', 1.5, ...
      'DisplayName', sprintf('\\sigma_{y,lam} = %.0f MPa', materials.sigma_y_lam*1e-6));
xline(w_ib_manuf_min_mm, 'k:', 'LineWidth', 1.2, ...
      'DisplayName', sprintf('manufacturing floor (5\\times0.35mm) = %.2f mm', w_ib_manuf_min_mm));
plot(sizer0.Wib_mm, sizer0.SigmaIb_MPa, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'g', ...
     'DisplayName', sprintf('current design (w_{ib}=%.2f mm)', sizer0.Wib_mm));
hold off;
grid on;
xlabel('Inner bridge width w_{ib} [mm]');
ylabel('Inner bridge stress \sigma_{ib} [MPa]');
title(sprintf('Rotor bridge centrifugal stress vs. inner bridge width (n_{max} = %d rpm)', maxSpeed_rpm));
legend('Location', 'best');

out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'IPM_Design_Report', 'figures');
exportgraphics(gcf, fullfile(out_dir, 'rotor_bridge_stress_sweep.png'), 'Resolution', 150);
fprintf('Saved figures/rotor_bridge_stress_sweep.png\n');

fprintf('\nCurrent design: w_ib=%.2f mm -> sigma_ib=%.1f MPa, ratio=%.3f (safe=%d)\n', ...
    sizer0.Wib_mm, sizer0.SigmaIb_MPa, sizer0.SigmaIbRatio, sizer0.BridgeSafe);
