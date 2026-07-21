%% main.m
% Main script to build the motor rotor in COMSOL.
% Run this script after fisrt_start.m has been called.

%% --- Call the first dimensioning script (Esson's rule)

%% --- Call the second dimensioning script (Article)

%% --- Input parameters (normally obtained with previous scripts)
mot_length  = 20e-2;        % [m]   Motor length
rotor_angle = 10*2*pi/360;  % [rad] Rotor angle 
Iq          = 100;         % [A]   Current Iq
r_ro        = 35e-3;        % [m]   Rotor outer radius
r_ri        = 15e-3;        % [m]   Rotor inner radius
p           = 4;            % [-]   Number of pole pairs
b_m         = 10e-3;        % [m]   Magnet length
h_m         = 3e-3;         % [m]   Magnet width
w_ib        = 5e-3;         % [m]   Magnet spacing
h_ry        = 5e-3;         % [m]   Magnet spacing with inner radius
angle_m     = 35*2*pi/360;  % [rad] Magnet angle
w_pocket    = 2e-3;         % [m]   Pocket length
h_pocket    = 1e-3;         % [m]   Pocket height

air_gap     = 1e-4;         % [m]   Air-gap radius
draw_only_sector = false;   % [bool]    Specify if only one sector of the
                            %           motor has to be drawn

r_si    = r_ro + air_gap;   % [m]   Stator inner radius
r_so    = r_si + 25e-3;     % [m]   Stator outer radius
Qs      = p*2*3;            % [-]   Number of slots
d_os    = 3e-3;             % [m]   Spacing with airgap
d_s     = 20e-3;            % [m]   Slot length
b1      = 2e-3;             % [m]   Slot bottom width
b2      = 4e-3;             % [m]   Slot top width
r1      = 0.5e-3;           % [m]   Radius of rounded slot edge

mu_r_shaft      = 1;        % [-]   Relative permeability (non-magnetic shaft)
sigma_shaft     = 1.4e6;    % [S/m] Electrical conductivity of stainless steel
epsilon_r_shaft = 0.8;
mu_r_iron       = 5000;     % [-]   Relative permeability of silicon steel (linear approximation)
sigma_iron      = 2e6;      % [S/m] Electrical conductivity of silicon steel lamination
epsilon_r_iron  = 0.8;
mu_r_air        = 1;        % [-]   Relative permeability of air (= vacuum)
sigma_air       = 0;        % [S/m] Air is a perfect electrical insulator
epsilon_r_air = 1;
mu_r_copper     = 1;        % [-]   Relative permeability of copper
sigma_copper    = 5.8e7;    % [S/m] Electrical conductivity of copper
epsilon_r_copper = 1;
mu_r_magnets    = 1.05;     % [-]   Relative permeability of NdFeB (close to 1)
sigma_magnets   = 6.25e5;   % [S/m] Electrical conductivity of NdFeB
epsilon_r_magnets = 1;
Br              = 1.3;      % [T]   Remanent flux density of NdFeB (N42 grade)
mesh_size       = 9;        % [-]   Mesh refinement level: 1 (extremely fine) to 9 (extremely coarse)

%% --- Create model and geometry node
import com.comsol.model.*
import com.comsol.model.util.*

model    = ModelUtil.create('MotorModel');  % create a new COMSOL model
comp_tag = 'comp1';
comp = model.component.create(comp_tag, true);
geom_tag = 'geom1';
geom     = model.geom.create(geom_tag, 2);  % 2D geometry
phys_tag = 'mf';
phys = comp.physics.create(phys_tag, 'InductionCurrents', geom_tag);
phys.prop('d').set('d',mot_length);

%% --- Call the stator drawing function
slot_points = draw_stator_sector(model, geom_tag, ...
                            r_si, r_so, air_gap, ...
                            Qs, d_os, d_s, b1, b2, r1, ...
                            p, draw_only_sector);

%% --- Call the rotor drawing function
[left_magnet_points, right_magnet_points, tip_pocket_points] = draw_rotor_sector(model, geom_tag, ...
                                              rotor_angle, r_ro, r_ri, p, ...
                                              b_m, h_m, w_ib, h_ry, angle_m, ...
                                              w_pocket, h_pocket, ...
                                              draw_only_sector);

%% --- Call the selection definition function
create_selections(model, geom_tag, draw_only_sector, rotor_angle, ...
    left_magnet_points, right_magnet_points, tip_pocket_points, slot_points, ...
    r_ri, r_ro, r_si, r_so, p)

%% --- Call the materials function
define_materials(   model, comp_tag, phys_tag, draw_only_sector, mesh_size, ...
                    mu_r_shaft, sigma_shaft, epsilon_r_shaft, ...
                    mu_r_iron, sigma_iron, epsilon_r_iron, ...
                    mu_r_air, sigma_air, epsilon_r_air, ...
                    mu_r_copper, sigma_copper, epsilon_r_copper, Iq, ...
                    mu_r_magnets, sigma_magnets, epsilon_r_magnets, Br);

%% --- Add stationary study
fprintf('Adding study... ');
std = model.study.create('std1');
std.create('stat', 'Stationary');
std.createAutoSequences('all');
fprintf('Done!\n');

fprintf('Evaluating torque... ');
fcall = model.component('comp1').physics('mf').create('force_calculation', 'ForceCalculation', 2);
fcall.selection.named('sel_rotor');
fprintf('Done!\n');

%% --- Save the model
fprintf('Saving model...\n');
save_path = fullfile(pwd, 'motor_model.mph');
model.save(save_path);
fprintf('Model saved to: %s\n', save_path);
