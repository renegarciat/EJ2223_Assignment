% 1. Create script to use the equations in the article. Hadrien. 
% 2. Create functions.
% 3. Integrate the geometry drawing into the script.
% 4. Results: Establish different sets of simulations. 1. Magnetostatic simulation. Obtain torque 
% out of the current. Evaluate the torque ripple by ... The rotor side one rotates it a little bit and
% re-run the simulation to get the torque. After an entire rotation, get the plot toruqe ripple vs. position.
% Back EMF: Get flux linkage in the windings as we expect. Get the efficiency.  3 things.
% Call amount of degrees. Call using a for loop. 
% Have a first visualization of the heat transfer using COMSOL.
% 
%% KTH Formula Student - IPM Motor Design Automation
% Author: IPM Design Team
% Description: Modular script to interface MATLAB with COMSOL for 
% IPM motor performance validation (Torque & Ripple).
% with Username: admin and Password: admin before executing this script.
clear; clc;
% 1. Add the COMSOL LiveLink folder to MATLAB's search path
% Adjust 'comsol62' to your version (e.g., comsol61 or comsol60)
addpath('/home/renet/opt/comsol64/multiphysics/mli');

%% 1. CONFIGURATION & DESIGN TARGETS
% Data sourced from AMK Inverter/Motor Datasheets and Phase 1 Sizing
params.Poles = 10;                  % Fixed for 1200Hz Inverter Limit
params.I_max = 107;                 % Inverter Peak Limit (Amps)
params.V_dc  = 515;                 % Nominal Bus Voltage
params.Target_Torque = 18.0;        % Team "Clip" Limit (Nm)

% Dimensions Sizing Script
params.Stack_Length = 0.061;        % 61 mm baseline
params.Rotor_OD = 0.080;            % 80 mm baseline
params.AirGap = 0.001;              % 1 mm 


%% 2. INITIALIZE COMSOL CONNECTION
try
    fprintf('Checking for COMSOL Server connection...\n');
    mphstart; 
    fprintf('Successfully connected to COMSOL Server.\n');
catch ME
    if contains(ME.message, 'Already connected')
        fprintf('COMSOL connection already active. Skipping handshake.\n');
    elseif contains(ME.message, 'Connection refused') || contains(ME.message, 'ConnectException')
        error(['[COMSOL ERROR]: Server not found. Run "comsol mphserver" in terminal.']);
    else
        error(['Unexpected connection error: ' ME.message]);
    end
end

%% 2.1 LOAD THE MOTOR MODEL
modelPath = 'pm_motor_2d_introduction.mph'; 

try
    fprintf('Loading COMSOL model: %s...\n', modelPath);
    model = mphload(modelPath); 
    fprintf('Model loaded successfully.\n');
catch ME
    error(['Could not find or load the file: %s. ' ...
           'Ensure the .mph file is in the same folder as this script. ' ...
           'Error: %s'], modelPath, ME.message);
end

%% 3. UPDATE MODEL PARAMETERS
fprintf('Updating COMSOL geometry and electrical parameters...\n');

try
    mphsetparam(model, 'I_peak', [num2str(params.I_max), '[A]']);
    mphsetparam(model, 'L_stack', [num2str(params.Stack_Length), '[m]']);
    mphsetparam(model, 'D_rotor', [num2str(params.Rotor_OD), '[m]']);
    mphsetparam(model, 'num_poles', num2str(params.Poles));
    
    fprintf('Parameters successfully updated to: %dA, %dm stack.\n', ...
            params.I_max, params.Stack_Length*1000);
catch ME
    % This error triggers if 'model' exists but parameters are named differently.
    error(['Parameter Update Failed: Check names in COMSOL Global Parameters. ' ...
           'Error: ' ME.message]);
end

% Initialize Java classes
import com.comsol.model.util.*
ModelUtil.showProgress(true);

%% 3. UPDATE MODEL PARAMETERS
% Using mphsetparam is more robust than direct Java calls
fprintf('Updating COMSOL geometry and electrical parameters...\n');

% Ensure the parameters exist in your .mph file before setting them
try
    mphsetparam(model, 'I_peak', [num2str(params.I_max), '[A]']);
    mphsetparam(model, 'L_stack', [num2str(params.Stack_Length), '[m]']);
    mphsetparam(model, 'D_rotor', [num2str(params.Rotor_OD), '[m]']);
    
    % For dimensionless parameters like pole count, num2str is still safest
    mphsetparam(model, 'num_poles', num2str(params.Poles));
    
    fprintf('Parameters successfully updated to: %dA, %dm stack.\n', ...
            params.I_max, params.Stack_Length*1000);
catch ME
    error(['Parameter Update Failed: Ensure that I_peak, L_stack, D_rotor, ' ...
           'and num_poles are defined in the Global Parameters of your .mph file. ' ...
           'Error: ' ME.message]);
end

%% 4. RUN SIMULATION (Phase 2: 2D Magnetic FEA)
% Compute the time-dependent study (e.g., 'std1')
fprintf('Starting COMSOL Solver... this may take a few minutes.\n');
tic;
model.study('std1').run;
simTime = toc;
fprintf('Simulation complete. Time: %.2f seconds.\n', simTime);

%% 5. RETRIEVE RESULTS & POST-PROCESSING
% Extracting Torque vs Time data
% 'rot_torque' must be defined in COMSOL Global Evaluations
torque_data = mphtable(model, 'tbl1'); % Assuming results are in table 1
t = torque_data.data(:,1);             % Time steps
torque_raw = torque_data.data(:,2);    % Electromagnetic Torque (Nm)

% Calculations for your report
avg_torque = mean(torque_raw);
torque_ripple = (max(torque_raw) - min(torque_raw)) / avg_torque * 100;

fprintf('--- Results ---\n');
fprintf('Average Torque: %.2f Nm (Target: %.2f Nm)\n', avg_torque, params.Target_Torque);
fprintf('Torque Ripple: %.2f %%\n', torque_ripple);

%% 6. VISUALIZATION
figure(1);
plot(t, torque_raw, 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
hold on;
yline(params.Target_Torque, '--r', 'Clipped Target (18Nm)');
grid on;
title(['IPM Performance: ', num2str(params.Poles), ' Poles at ', num2str(params.I_max), 'A']);
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('FEA Result', 'Design Requirement');

% Save results to a structure for later comparison
results.avg_torque = avg_torque;
results.ripple = torque_ripple;
save('Simulation_Results.mat', 'results');