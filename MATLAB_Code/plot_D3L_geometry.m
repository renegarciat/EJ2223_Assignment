%% plot_D3L_geometry.m
% Visualizes the D^3L output-coefficient method (EssonsSizer.solveD3L,
% Lecture 6 "Improving the Esson's rule") to make beta_t, beta_c, and
% fo(Dis/Dos) concrete instead of abstract, at our design's fixed P=10/
% Q=12 topology (no pole-count sweep -- an earlier version of this script
% compared several pole counts, which is no longer relevant now that the
% pole/slot combination is fixed):
%
%   Left:  fo(rho) at our design's flux-density targets (Bg1, Bt, Bc) and
%          P=10, marking its unconstrained optimum.
%
%   Right: an annotated cross-section schematic at that optimal ratio,
%          showing what beta_t (tooth width / slot pitch) and beta_c
%          (yoke depth / bore radius) physically carve out of the annulus
%          between Dis and Dos: teeth (iron), slots (copper), and yoke.
%
% Run from MATLAB_Code/ (or with it on the path); saves a PNG into
% ../IPM_Design_Report/figures/.
clear; clc;

spec = MotorSpec(9.8, 10000, 12000, 515, 10, 12, 1); % Br20 now lives in MotorMaterials, not MotorSpec
Bg1 = spec.Bg1_T; Bt = spec.Bt_T; Bc = spec.Bc_T;
kis = spec.IronFillFactor; kst = spec.StackingFactor;
P_actual = spec.Poles; Q = spec.Slots;
% Dos = 80mm was the current motor's IMB5 mounting-flange register
% diameter (front_technical_drawing.png), not its housing OD -- confirmed
% against the full AMK mechanical drawing (references/AMK motor
% mechanical drawings.pdf), which dimensions the actual housing body at
% Phi95.9mm (spans the full-length cylindrical casing) vs. the small
% flange disc at just the shaft end (Phi80mm register / Phi88mm bolt-boss
% envelope). Using the housing OD instead, per the packaging rationale in
% the report ("shouldn't be bigger than the old one").
Dos = 95.9e-3;

fig = figure('Name', 'D^3L output function and geometry', 'Position', [100 100 1100 480]);

[a, b, beta_t, beta_c] = EssonsSizer.computeAB_(Bg1, Bt, Bc, P_actual, kis, kst);
rho_u = fminbnd(@(r) -EssonsSizer.fo_(r, a, b), 1e-3, 1-1e-3);

% ---------------- Left panel: fo(rho) at P=10/Q=12 ----------------
subplot(1,2,1);
hold on; grid on;
rho = linspace(0.01, 0.99, 400);
fo = EssonsSizer.fo_(rho, a, b);
fo_max = EssonsSizer.fo_(rho_u, a, b);
plot(rho, fo, 'LineWidth', 1.8, 'Color', [0 0.45 0.74]);
plot(rho_u, fo_max, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0 0.45 0.74], 'MarkerEdgeColor', 'k');
text(rho_u+0.03, fo_max, sprintf('optimum \\rho=%.3f', rho_u), 'FontSize', 8);
xlabel('\rho = D_{is}/D_{os}');
ylabel('f_o(\rho)');
title({sprintf('Output function f_o(\\rho), P=%d/Q=%d', P_actual, Q), ...
    sprintf('(B_{g,1}=%.2fT, B_t=%.2fT, B_c=%.2fT -- our design''s targets)', Bg1, Bt, Bc)});
ylim([-0.1, 0.5]);

% ---------------- Right panel: annotated cross-section --------------
subplot(1,2,2);
Dis = rho_u * Dos;
d_cs = Dis * beta_c / 2;              % yoke depth (see computeAB_ derivation)
R_os = Dos/2; R_is = Dis/2; R_yoke = R_os - d_cs;

hold on; axis equal off;
th = linspace(0, 2*pi, 200);

% Draw back-to-front so each layer punches through the previous one:
% (1) solid iron disk out to Dos -- becomes the yoke ring once (2) punches
% an orange hole inside R_yoke; (3) teeth wedges then paint iron back on
% top of the orange in the tooth-pitch fraction only, leaving the
% remaining (1-beta_t) fraction of each slot pitch visibly orange (slots).
fill(R_os*cos(th), R_os*sin(th), [0.55 0.55 0.58], 'EdgeColor', 'k');       % (1) iron
fill(R_yoke*cos(th), R_yoke*sin(th), [0.85 0.5 0.15], 'EdgeColor', 'none'); % (2) slot region (orange)

slot_pitch_ang = 2*pi/Q;
tooth_ang = beta_t * slot_pitch_ang;
for k = 0:Q-1
    a0 = k*slot_pitch_ang - tooth_ang/2;
    a1 = a0 + tooth_ang;
    aw = linspace(a0, a1, 10);
    xw = [R_is*cos(aw), R_yoke*cos(fliplr(aw))];
    yw = [R_is*sin(aw), R_yoke*sin(fliplr(aw))];
    fill(xw, yw, [0.55 0.55 0.58], 'EdgeColor', 'k', 'LineWidth', 0.5);     % (3) tooth (iron)
end
fill(R_is*cos(th), R_is*sin(th), [0.75 0.85 0.95], 'EdgeColor', 'k');       % (4) bore/rotor cavity

title({sprintf('Cross-section at optimal \\rho=%.3f (P=%d, D_{os}=%.0fmm)', rho_u, P_actual, Dos*1e3), ...
    sprintf('\\beta_t=%.3f (tooth/slot-pitch), \\beta_c=%.3f (2\\cdotd_{cs}/D_{is})', beta_t, beta_c)});

% Annotations
text(0, 0, sprintf('bore\nD_{is}=%.1fmm', Dis*1e3), 'HorizontalAlignment', 'center', 'FontSize', 8);
text(R_os*0.75, R_os*0.75, sprintf('yoke\nd_{cs}=%.1fmm', d_cs*1e3), 'FontSize', 8, 'HorizontalAlignment', 'center');
text(-R_os*0.9, -R_os*0.55, sprintf('teeth (gray)\nslots (orange)'), 'FontSize', 8);
xlim([-R_os*1.15, R_os*1.15]); ylim([-R_os*1.15, R_os*1.15]);

sgtitle('D^3L output-coefficient method: \beta_t, \beta_c, and f_o(\rho)');

out_dir = fullfile('..', 'IPM_Design_Report', 'figures');
exportgraphics(fig, fullfile(out_dir, 'd3l_output_function.png'), 'Resolution', 150);
fprintf('Saved figures/d3l_output_function.png\n');
fprintf('Actual design: beta_t=%.4f, beta_c=%.4f, rho_u=%.4f, Dis=%.2fmm, d_cs=%.2fmm\n', ...
    beta_t, beta_c, rho_u, Dis*1e3, d_cs*1e3);
