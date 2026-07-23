%% rotor_bridge_stress_sweep.m
% Sweeps the inner bridge width (Wib_mm) and plots the resulting
% centrifugal stress at max speed against the lamination yield strength.
%
% Implements Di Gerlando & Ricca, ICEM 2022, Section IV (eqs. 19-22).
% See doc/rotor_bridge_stress_notes.md for full sourcing and the known
% simplification (m_1p is magnet-only, pole-shoe iron neglected).
%
% NOTE: this currently uses the PAPER'S reference spec/design point
% (MotorSpec(200,2900,13500,650,8,60,1,1.37), default rotor geometry
% choices), NOT the project's actual main.m parameters. main.m's current
% values (poles=10, AlphaM=0.83, Whr_fraction=0.1) fail geometry
% feasibility (h_ob <= 0, see doc/bug_001.md) independent of Wib_mm —
% that's a separate, pre-existing issue that needs AlphaM/Whr_fraction
% (or pole count) revisited, not something to silently work around here.
clear; clc;

torque_Nm       = 200;
cornerSpeed_rpm = 2900;
maxSpeed_rpm    = 13500;
vdcLink_V       = 650;
poles           = 8;
slots           = 60;
airgap_mm       = 1;
br20_T          = 1.37;
AlphaM          = 0.754;
Whr_fraction    = 0.55;
Hm_mm           = 6;

spec = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, ...
                  vdcLink_V, poles, slots, airgap_mm, br20_T);
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
title('Rotor bridge centrifugal stress vs. inner bridge width (n_{max} = 12,000 rpm)');
legend('Location', 'best');

fprintf('\nCurrent design: w_ib=%.2f mm -> sigma_ib=%.1f MPa, ratio=%.3f (safe=%d)\n', ...
    sizer0.Wib_mm, sizer0.SigmaIb_MPa, sizer0.SigmaIbRatio, sizer0.BridgeSafe);
