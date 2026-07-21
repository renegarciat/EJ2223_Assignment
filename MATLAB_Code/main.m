%% main.m
clear; clc;
% Dataset 1:

% Desired torque at corner speed
torque_Nm = 7; % [Nm] 
cornerSpeed_rpm = 10000; % [rpm]
maxSpeed_rpm = 12000; % [rpm] % Approx. 35% higher than corner speed
vdcLink_V = 515; % [V] %Changed!
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

%% --- Run stationary study
comsolInterface.runStudy();

%% --- Save the model
fprintf('Saving model...\n');
save_path = fullfile(pwd, 'COMSOL_models', 'motor_model.mph');
comsolInterface.saveModel(save_path);
fprintf('Model saved to: %s\n', save_path);
%% --- Extract the Results
% Pull the torque array into MATLAB workspace for plotting or saving
torque_results = comsolInterface.extractTorqueFromTable();

% Optional: Plot the torque if you did a parametric sweep of the rotor angle
if length(torque_results) > 1
    figure('Name', 'Motor Torque Output');
    plot(torque_results, '-o', 'LineWidth', 2);
    title('Axial Torque over Sweep');
    xlabel('Step');
    ylabel('Torque (Nm)');
    grid on;
end