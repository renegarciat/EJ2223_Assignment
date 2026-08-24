%% main.m
clear; clc; close all;
% Directives
RUN_COMSOL_PREBUILT = true;
RUN_COMSOL_PREBUILT_STUDY = true;  % runs the coupled EM/structural transient
                                    % study (~20 min). cheap to leave on:
                                    % results are cached (COMSOL_models/cache/)
                                    % and only re-solved when the design
                                    % point actually changes, or FORCE_-
                                    % COMSOL_RESOLVE is set below.
FORCE_COMSOL_RESOLVE = false;       % set true to ignore any matching cache
                                    % entry and re-solve regardless
RUN_EFFICIENCY_MAP = true;         % analytical torque-speed efficiency
                                    % map (seconds, no FEM) -- see
                                    % EfficiencyMapSizer.m
torque_Nm = 9.8; % [Nm] % Desired torque at corner speed
maxSpeed_rpm = 14400; % [rpm] Max. allowed by the inverter.
cornerSpeed_rpm = int32(round(maxSpeed_rpm*0.85)); % [rpm]. 14,400*0.85=12,240
vdcLink_V = 515; % [V]
poles = 10;
slots = 12;
airgap_mm = 1; % [mm]
AlphaM = 0.80; % [0.5  - 1] Magnet Embrace / Pole Arc Ratio.
Whr_fraction = 0.1; % [0.1 - 0.8] Half-width-rib fraction: how wide the ribs should be compared to the stator slot pitch
Hm_mm = 3; % [mm] Magnet height
AspectRatio = 2.0; % [1.1 - 2.0] le/tau_p
Dos_target_m = 0.0899; % [m] Target stator (core) outer diameter
CurrentDensity_Amm2 = 12; % [A/mm^2] Design J_s,rms -- raised from the 8
                           % A/mm^2 default so Formula 2's D^3L stack
                           % length clears the 110mm axial limit (see
                           % report's "Formula 2" section); reused below
                           % as Stage 1's bore, so this is now the single
                           % current-density assumption for both.
spec = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, ...
                vdcLink_V, poles, slots, ...
                airgap_mm, AspectRatio = AspectRatio, ...
                CurrentDensity_Amm2 = CurrentDensity_Amm2);
spec.summary()
materials = MotorMaterials();
materials.Br = 1.37; % [T] N48 NdFeB at 20degC

essonsSizer = EssonsSizer(spec);
% Run D^2L formula
disp('Running D^2L sizing...');
essonsSizer.solveD2L();      % run
essonsSizer.summary();       % print results
% Run D^3L formula
disp('Running D^3L sizing...');
essonsSizer.solveD3L(Dos_target_m);      % run
essonsSizer.summaryD3L();       % print results
% Run paper's calculations. Stage 1's bore is Formula 2's D^3L result
% (essonsSizer.Dis_D3L_m), not Formula 1's D^2L one: it is the only one
% of the two whose implied stator OD (Stage 4, eq. 94) actually respects
% the Dos_target_m housing constraint above -- see report's Stage 1
% discussion.
rotorSizer = IPMRotorSizer(spec, Materials = materials, AlphaM = AlphaM, Whr_fraction = Whr_fraction, Hm_mm = Hm_mm, ...
    StatorBore_mm = essonsSizer.Dis_D3L_m * 1e3);
rotorSizer.solve();          % run until convergence
rotorSizer.summary();        % print results

% Build MotorGeometry from sizing results
motorGeometry = MotorGeometry.fromSizingResults(spec, essonsSizer, rotorSizer);
motorGeometry.summary();

% mfilename('fullpath'), not pwd: MATLAB's run() changes the current
% folder to the script's own folder while it executes, so pwd is
% MATLAB_Code/ here regardless of where run('MATLAB_Code/main.m') was
% invoked from. This resolves the repo root robustly either way.
repoRoot = fileparts(mfilename('fullpath'));
reportFiguresDir = fullfile(repoRoot, '..', 'IPM_Design_Report', 'figures');

%% --- Analytical torque-speed efficiency map (no FEM required)
if RUN_EFFICIENCY_MAP
    effMap = EfficiencyMapSizer(spec, rotorSizer, materials);
    effMap.solve();
    effMap.summary();
    effMap.plotMap(SavePath = fullfile(reportFiguresDir, 'efficiency_map.png'));
end
return; % temporal
%% --- Push sizing results into the prebuilt COMSOL model
% Requires "comsol mphserver" running in a terminal.
if RUN_COMSOL_PREBUILT
    comsolPrebuilt = ComsolPrebuiltInterface();
    comsolPrebuilt.pushGeometry( ...
        Np = poles, ...
        Ns = slots, ...
        Airgap_mm = motorGeometry.Airgap_m * 1e3, ...
        StatorYokeHeight_mm = motorGeometry.StatorYokeHeight_m * 1e3, ...
        RotorDiameter_mm = motorGeometry.RotorOuterRadius_m * 2e3, ...
        ShaftDiameter_mm = motorGeometry.RotorInnerRadius_m * 2e3, ...
        L_mm = rotorSizer.StackLength_mm, ...
        MaxSpeed_rpm = maxSpeed_rpm);
    comsolPrebuilt.pushExcitation( ...
        Ipk_A = hypot(motorGeometry.Id_A, motorGeometry.Iq_A)); % Id=0 at our corner point, so this is just Iq
    comsolPrebuilt.rebuildGeometry();
    comsolPrebuilt.summary();

    if RUN_COMSOL_PREBUILT_STUDY
        % ~20 min on a cache miss (design point changed since the last
        % run); near-instant otherwise -- see runStudyCached's docstring.
        comsolPrebuilt.runStudyCached(Force = FORCE_COMSOL_RESOLVE, FigureDir = reportFiguresDir);
    end

    comsolPrebuilt.saveModelAs();
end