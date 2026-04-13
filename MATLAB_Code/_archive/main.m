%% main.m
% Main script to build the motor rotor in COMSOL.
%% --- Call the first dimensioning script (Esson's rule)

%% --- Call the second dimensioning script (Article)

%% --- Input parameters (normally obtained with previous scripts)
D_r     = 55e-3;        % [m]   Rotor outer radius (= stator inner radius - air gap)
D_ir    = 15e-3;        % [m]   Inner radius of rotor lamination
air_gap = 2e-3;         % [m]   Air-gap radius
p       = 4;            % [-]   Number of pole pairs
b_m     = 20e-3;        % [m]   Magnet length
h_m     = 5e-3;         % [m]   Magnet width
w_ib    = 5e-3;         % [m]   Magnet spacing
h_ry    = 10e-3;        % [m]   Magnet spacing with inner radius
angle_m = 25*2*pi/360;  % [rad] Magnet angle

draw_only_sector = true;   % [bool]    Specify if only one sector of the
                            %           motor has to be drawn

r_si = D_r + air_gap;
r_so = r_si + 10e-3;
Qs = 10;
slot_depth = 5e-3;  % [m]   Depth of the stator slots
slot_width = 5e-3;   % [m]   Width of the stator slots

mu_r_shaft  = 1;        % [-]   Relative permeability (non-magnetic shaft)
sigma_shaft = 1.4e6;    % [S/m] Electrical conductivity of stainless steel
epsilon_r_shaft = 0.8;
mu_r_iron   = 5000;     % [-]   Relative permeability of silicon steel (linear approximation)
sigma_iron  = 2e6;      % [S/m] Electrical conductivity of silicon steel lamination
epsilon_r_iron = 0.8;
mu_r_air    = 1;        % [-]   Relative permeability of air (= vacuum)
sigma_air   = 0;        % [S/m] Air is a perfect electrical insulator
mu_r_magnets  = 1.05;   % [-]   Relative permeability of NdFeB (close to 1)
sigma_magnets = 6.25e5; % [S/m] Electrical conductivity of NdFeB
Br            = 1.3;    % [T]   Remanent flux density of NdFeB (N42 grade)
mesh_size = 5;          % [-]   Mesh refinement level: 1 (extremely fine) to 9 (extremely coarse)
%% --- Establish COMSOL connection
comsolInterface = ComsolInterface();
comsolInterface.start();
% comsolInterface.runComsolSimulation(params, config);
% %% --- Create model and geometry node
% import com.comsol.model.*
% import com.comsol.model.util.*

% model    = ModelUtil.create('MotorModel');  % create a new COMSOL model
% comp_tag = 'comp1';
% comp = model.component.create(comp_tag, true);
% geom_tag = 'geom1';
% geom     = model.geom.create(geom_tag, 2);  % 2D geometry
% phys_tag = 'mf';
% phys = comp.physics.create(phys_tag, 'InductionCurrents', geom_tag);

% %% --- Call the stator drawing function
% draw_stator_sector(model, geom_tag, ...
%                             r_si, ...
%                             r_so, ...
%                             Qs, ...
%                             p, ...
%                             slot_depth, ...
%                             slot_width, ...
%                             draw_only_sector)

% %% --- Call the rotor drawing function
% [left_magnet_points, right_magnet_points] = draw_rotor_sector(model, geom_tag, ...
%                                               D_r, D_ir, air_gap, p, ...
%                                               b_m, h_m, w_ib, h_ry, angle_m, ...
%                                               draw_only_sector);

% %% --- Call the selection definition function
% create_selections(model, geom_tag, draw_only_sector, left_magnet_points, right_magnet_points, ...
%     D_ir, D_r, r_si, r_so, p)

% %% --- Call the materials function
% define_materials(   model, comp_tag, phys_tag, draw_only_sector, mesh_size, ...
%                     mu_r_shaft, sigma_shaft, epsilon_r_shaft, ...
%                     mu_r_iron, sigma_iron, epsilon_r_iron, ...
%                     mu_r_air, sigma_air, ...
%                     mu_r_magnets, sigma_magnets, Br);

% %% --- Save the model
% save_path = fullfile(pwd, 'motor_model.mph');
% model.save(save_path);
% fprintf('Model saved to: %s\n', save_path);

%% --- Call the material function
% It should maybe be call before the model is saved

%% Main 2
% %% IPM Design Workflow Entry Point
% % Runs: Essen sizing estimate -> pushes geometry into COMSOL -> extracts torque.
% %
% % Edit the "USER INPUTS" section
% % We need a function to calculate stacking ratio, the iron lenght will be smaller than the active length due to lamination.
% % We need to calculate this instead of hard coding it. stackingFactor = 1.05.
% clc;
% clearvars -except RUN_COMSOL Ipk_A;

% %% Paths
% thisFile = mfilename('fullpath');
% thisDir = fileparts(thisFile);
% projectRoot = fullfile(thisDir, '..');
% addpath(thisDir);

% %% 1) USER INPUTS (EDIT IF NEEDED)

% % Include this definitions before the run in case you don't want to run COMSOL
% % (e.g. RUN_COMSOL=false; run('main.m'))
% if ~exist('RUN_COMSOL', 'var')
%     RUN_COMSOL = true;
% end

% % Topology
% P = 10;              % poles
% Q = 12;              % slots
% m = 3;               % phases

% % Operating point
% % Sweep electrical frequency. (pole-pairs relation)
% w_r_rpm = 20000;     % mechanical speed [rpm]

% % Magnetic / electric loading assumptions (Essens input)
% Bg1 = 0.9;           % [T] airgap flux density (0.85..1.05 typical)
% Bt  = 1.8;           % [T] tooth flux density (1.6..2.0 typical)
% Bc  = 1.8;           % [T] back iron flux density (often ~1.0..1.5 typical; using test value)
% A1  = 90000;         % [A/m] linear current density
% % Sweep this value and store results.
% J_rms_A_per_mm2 = 5; % [A/mm^2] RMS current density Cannot remove. Needed for geometrical size calculation.

% % COMSOL excitation (phase peak current)
% % EssonsEstimation uses A1 and J_rms to size the geometry, but the COMSOL
% % model is excited via Ipk. We don't have winding turns/parallel paths here,
% % so we default to scaling Ipk with A1 relative to the legacy baseline.
% % Override without editing this file by running e.g.:
% %   Ipk_A = 150; RUN_COMSOL=true; run('main.m')
% if ~exist('Ipk_A', 'var') || isempty(Ipk_A)
%     Ipk_ref_A = 107;   % legacy baseline used previously in the COMSOL model
%     A1_ref = 30000;    % legacy baseline for linear current density [A/m]
%     Ipk_A = Ipk_ref_A * (A1 / A1_ref);
% end

% % Geometry assumptions
% AspectRatio = 5;     % [-] le/tau_p
% eta_estimation = 0.95;
% cos_phi = 0.95;
% airgap_m = 0.7e-3;   % [m]
% stackingfactor = 1.05; % [-]
% kis = 0.95;          % [-] effective iron factor in stator
% slot_opening_m = 0.5e-3; % [m] (dos)
% kcu = 0.45;          % [-] copper fill ratio

% % Torque target for sizing
% T_target_Nm = 15; %(nominal)

% % Stack-length adjustment (used when pushing into COMSOL)
% windingHead_m = 0.0238;

% % IPM rotor parameters (COMSOL)
% w_m_mm = 14;
% t_m_mm = 3.5;
% alpha_v_deg = 130;
% b_t_mm = 1.0;

% %% 2) ESSEN SIZING ESTIMATE
% % EssonsEstimation is the source of truth for the motor envelope + effective length.
% [Dis, le, tau_p, tau_s, t_s, h_slot, h_cs, Dro, Dso, ratio, q_spp] = EssonsEstimation( ...
%     T_target_Nm, w_r_rpm, P, Q, m, Bg1, Bt, Bc, A1, J_rms_A_per_mm2, ...
%     AspectRatio, eta_estimation, cos_phi, airgap_m, stackingfactor, kis, ...
%     slot_opening_m, kcu);

% L_for_comsol_m = le + windingHead_m;

% fprintf('\n--- EssonsEstimation Results (used for COMSOL) ---\n');
% fprintf('Dis: %.2f mm | Dro: %.2f mm | Dso: %.2f mm\n', Dis*1e3, Dro*1e3, Dso*1e3);
% fprintf('le:  %.2f mm | L_total(with winding head): %.2f mm\n', le*1e3, L_for_comsol_m*1e3);
% fprintf('h_slot: %.2f mm | h_cs: %.2f mm\n', h_slot*1e3, h_cs*1e3);
% fprintf('ratio Dis/Dso: %.3f | q_spp: %.3f\n', ratio, q_spp);

% if h_slot <= 0
%     warning('EssonsEstimation returned non-positive slot height h_slot=%.3f mm. Check Bg1/Bt/Bc, A1, J_rms, kis, kcu, dos.', h_slot*1e3);
% end

% fprintf('Ipk used for COMSOL excitation: %.2f A (Irms=%.2f A)\n', Ipk_A, Ipk_A/sqrt(2));

% %% 3) BUILD COMSOL PARAMETER STRUCT
% params = struct();

% % Geometry
% params.Np = sprintf('%d', P);
% params.Ns = sprintf('%d', Q);
% params.d_r = sprintf('%.3f[mm]', Dro*1e3);
% params.d_st = sprintf('%.3f[mm]', Dso*1e3);
% params.airgap = sprintf('%.3f[mm]', airgap_m*1e3);
% params.d_cont = sprintf('%.3f[mm]', Dro*1e3 + airgap_m*1e3); % matches existing convention
% params.L = sprintf('%.4f[m]', L_for_comsol_m);

% % Keep existing model parameters unless you want to parameterize them too
% params.mag_h = '2.5[mm]';
% params.d_s = '10[mm]';

% % Operating point / excitation
% params.w_rot = sprintf('%d[rpm]', w_r_rpm);
% params.Ipk = sprintf('%.3f[A]', Ipk_A);
% params.init_ang = '200[deg]';

% % V-shape IPM Specifics
% params.w_m = sprintf('%.3f[mm]', w_m_mm);
% params.t_m = sprintf('%.3f[mm]', t_m_mm);
% params.alpha_v = sprintf('%.3f[deg]', alpha_v_deg);
% params.b_t = sprintf('%.3f[mm]', b_t_mm);

% %% 4) RUN COMSOL
% config = struct();
% config.studyTag = 'std1';
% config.resultTableTag = 'tbl1';
% config.makePlot = true;
% config.saveResults = true;
% config.savePath = fullfile(projectRoot, 'Simulation_Results.mat');

% if RUN_COMSOL
%     results = run_comsol_simulation(params, config);
%     fprintf('\nSaved results to: %s\n', config.savePath);
% else
%     fprintf('\nRUN_COMSOL=false, skipping COMSOL run.\n');
%     results = struct();
% end