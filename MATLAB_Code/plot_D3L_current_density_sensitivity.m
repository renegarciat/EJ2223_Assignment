%% plot_D3L_current_density_sensitivity.m
% Documents the investigation into why EssonsSizer.solveD3L(89.9mm) gives
% a stack length that overshoots the 110mm axial ceiling (Dis=32.8mm,
% le=123.5mm, le/Dis=3.77) for the actual design (9.8Nm, 10 poles,
% current motor's 89.9mm stator-core-OD/85.2mm-length housing).
%
% Dos = 89.9mm is the current motor's stator core OD: its housing OD
% (95.9mm, the cylinder containing the stator, confirmed against the
% full AMK mechanical drawing, references/AMK motor mechanical
% drawings.pdf) less the drawing's own 3mm minimal wall thickness --
% that wall is not part of the active stator core Formula 2 is sizing.
% An earlier version of this script (a) used Dos=80mm, the IMB5
% mounting-flange register diameter at the shaft end, an unrelated,
% much smaller feature -- see plot_D3L_geometry.m's header note -- and
% (b) after fixing that, used Dos=95.9mm directly (the raw housing OD,
% before the wall-thickness correction above).
%
% Finding at the corrected Dos: at the default Js,rms=8 A/mm^2, the
% Ks,rms,max=90kA/m cooling constraint is only marginally binding
% (the unconstrained geometry-optimal ratio alone would need
% Ks,rms just over 90kA/m), and loosening it further doesn't help --
% Dis relaxes back to the geometric optimum and le barely moves. The
% real lever is still Js,rms: sweeping it shows le drops to exactly the
% 85.2mm axial limit at Js,rms~12 A/mm^2 (Dis~39.5mm) -- still within
% the 7-10 A/mm^2 range the 8 A/mm^2 default was drawn from, so this is
% a real, not unreasonable, design lever. This does NOT mean Formula 2
% is wrong: it means our *default* Js,rms (borrowed from the much
% bigger reference paper machine) understates what this specific, much
% smaller, liquid-cooled racing motor can actually do.
%
% Run from MATLAB_Code/ (or with it on the path); saves a PNG into
% ../IPM_Design_Report/figures/.
clear; clc;

Dos = 89.9e-3;
axialLimit_mm = 85.2;
Js_sweep = linspace(4, 32, 60);

Dis_mm = zeros(size(Js_sweep));
le_mm  = zeros(size(Js_sweep));
active = false(size(Js_sweep));

for i = 1:numel(Js_sweep)
    spec = MotorSpec(9.8, 10000, 12000, 515, 10, 12, 1, ...
        CurrentDensity_Amm2 = Js_sweep(i)); % Br20 now lives in MotorMaterials, not MotorSpec
    sizer = EssonsSizer(spec);
    sizer.solveD3L(Dos, MakePlot=false); % this script builds its own figure below
    Dis_mm(i) = sizer.Dis_D3L_m * 1e3;
    le_mm(i)  = sizer.le_D3L_m * 1e3;
    active(i) = sizer.ConstraintActiveD3L;
end
ratio_leDis = le_mm ./ Dis_mm;

idxActive = find(active, 1, 'first');
fig = figure('Name', 'D^3L sensitivity to current density', 'Position', [100 100 1500 430]);

subplot(1,3,1);
hold on; grid on;
plot(Js_sweep, Dis_mm, '-', 'LineWidth', 1.8, 'Color', [0 0.45 0.74]);
if ~isempty(idxActive)
    xline(Js_sweep(idxActive), ':', 'Color', [0.3 0.3 0.3]);
end
xline(8, ':k');
xlabel('J_{s,rms} [A/mm^2]');
ylabel('D_{is} [mm]');
title('Optimal bore D_{is}');

subplot(1,3,2);
hold on; grid on;
plot(Js_sweep, le_mm, '-', 'LineWidth', 1.8, 'Color', [0.85 0.33 0.1]);
yline(axialLimit_mm, '--', 'Color', [0.7 0 0], 'LineWidth', 1.3);
text(Js_sweep(end)*0.35, axialLimit_mm*1.06, sprintf('%.1fmm axial limit', axialLimit_mm), ...
    'FontSize', 8, 'Color', [0.7 0 0]);
if ~isempty(idxActive)
    xline(Js_sweep(idxActive), ':', 'Color', [0.3 0.3 0.3]);
end
xline(8, ':k');
text(8.3, max(le_mm)*0.92, 'default J_{s,rms}=8', 'FontSize', 8);
xlabel('J_{s,rms} [A/mm^2]');
ylabel('l_e [mm]');
title({'Stack length l_e needed for T=9.8Nm', sprintf('(fixed D_{os}=%.0fmm)', Dos*1e3)});

subplot(1,3,3);
hold on; grid on;
plot(Js_sweep, ratio_leDis, '-', 'LineWidth', 1.8, 'Color', [0.1 0.4 0.1]);
if ~isempty(idxActive)
    xline(Js_sweep(idxActive), ':', 'Color', [0.3 0.3 0.3]);
    text(Js_sweep(idxActive)+0.3, max(ratio_leDis)*0.9, 'K_{s,rms} active', 'FontSize', 8);
end
yline(2.0, '--', 'Color', [0.6 0.6 0.6]);
text(Js_sweep(end)*0.4, 2.3, 'l_e/D_{is}=2 reference', 'FontSize', 8, 'Color', [0.4 0.4 0.4]);
xlabel('J_{s,rms} [A/mm^2]');
ylabel('l_e / D_{is}');
title('Aspect ratio l_e/D_{is}');

sgtitle(sprintf('Why Formula 2 needed a longer stack at D_{os}=%.0fmm, and what fixes it', Dos*1e3));

out_dir = fullfile('..', 'IPM_Design_Report', 'figures');
exportgraphics(fig, fullfile(out_dir, 'd3l_current_density_sensitivity.png'), 'Resolution', 150);
fprintf('Saved figures/d3l_current_density_sensitivity.png\n');

% Solved exactly at Js=8 (the default) and at the le=axialLimit_mm
% crossing point, rather than read off the nearest Js_sweep grid point,
% so these numbers match the report prose exactly.
[Dis8_mm, le8_mm] = local_solveD3L(8, Dos);
fprintf('\nJs=8.0:   Dis=%.2fmm le=%.2fmm le/Dis=%.2f\n', Dis8_mm, le8_mm, le8_mm/Dis8_mm);

Js_cross = fzero(@(Js) local_le_only(Js, Dos) - axialLimit_mm, [8, 20]);
[Dis_cross_mm, le_cross_mm] = local_solveD3L(Js_cross, Dos);
fprintf('Js=%.2f (le=axial limit): Dis=%.2fmm le=%.2fmm le/Dis=%.2f\n', ...
    Js_cross, Dis_cross_mm, le_cross_mm, le_cross_mm/Dis_cross_mm);

function [Dis_mm, le_mm] = local_solveD3L(Js, Dos)
    spec = MotorSpec(9.8, 10000, 12000, 515, 10, 12, 1, CurrentDensity_Amm2=Js);
    sizer = EssonsSizer(spec);
    sizer.solveD3L(Dos, MakePlot=false); % called repeatedly by fzero/the sweep above
    Dis_mm = sizer.Dis_D3L_m * 1e3;
    le_mm  = sizer.le_D3L_m * 1e3;
end

function le_mm = local_le_only(Js, Dos)
    [~, le_mm] = local_solveD3L(Js, Dos);
end
