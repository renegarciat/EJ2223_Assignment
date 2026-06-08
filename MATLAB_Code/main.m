%% main.m
clear; clc;
% Dataset 1:

% Desired torque at corner speed
torque_Nm = 7; % [Nm] 
cornerSpeed_rpm = 10000; % [rpm]
maxSpeed_rpm = 13500; % [rpm] % Approx. 35% higher than corner speed
vdcLink_V = 334; % [V]
poles = 8;
slots = 12;
airgap_mm = 1; % [mm]
br20_T = 1.37; % [T]
% Magnet Embrace / Pole Arc Ratio.
% Default: 0.754
AlphaM = 0.83; % [0.5  - 1]
% Half-width-rib fraction: how wide the ribs should be compared to the stator slot pitch
% Default: 0.55
Whr_fraction = 0.1; % [0.1 - 0.9]
Hm_mm = 3; % [mm]

spec = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, ...
                vdcLink_V, poles, slots, ...
                airgap_mm, br20_T);
spec.summary()
essonsSizer = EssonsSizer(spec);
essonsSizer.solve();         % run
essonsSizer.summary();       % print results
rotorSizer = IPMRotorSizer(spec, AlphaM = AlphaM, Whr_fraction = Whr_fraction, Hm_mm = Hm_mm);
rotorSizer.solve();          % run until convergence
rotorSizer.summary();        % print results

% Build MotorGeometry from sizing results
motorGeometry = MotorGeometry.fromSizingResults(spec, essonsSizer, rotorSizer);

materials = MotorMaterials();

comsolInterface = ComsolInterface(motorGeometry, materials);
comsolInterface.drawStatorSector();
comsolInterface.drawRotorSector();
comsolInterface.createSelections();
comsolInterface.defineMaterials(materials);
%% --- Add stationary study
comsolInterface.addStationaryStudy();
%% --- Save the model
fprintf('Saving model...\n');
save_path = fullfile(pwd, 'COMSOL_models', 'motor_model.mph');
comsolInterface.saveModel(save_path);
fprintf('Model saved to: %s\n', save_path);
