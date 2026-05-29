%% main.m
clear; clc;
torque_Nm = 200; % [Nm]
cornerSpeed_rpm = 2900; % [rpm]
maxSpeed_rpm = 13500; % [rpm]
vdcLink_V = 650; % [V]
poles = 8;
slots = 60;
airgap_mm = 1; % [mm]
br20_T = 1.37; % [T]

spec = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, ...
                vdcLink_V, poles, slots, ...
                airgap_mm, br20_T);
spec.summary()
essonsSizer = EssonsSizer(spec);
essonsSizer.solve();         % run
essonsSizer.summary();       % print results
rotorSizer = IPMRotorSizer(spec);
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
% %% --- Save the model
save_path = fullfile(pwd, 'COMSOL_models', 'motor_model.mph');
comsolInterface.saveModel(save_path);