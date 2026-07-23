classdef MotorMaterials
   % MotorMaterials Typed container for COMSOL material constants.
   %
   % This class intentionally defines properties only (no methods).

   properties
      mesh_size (1,1) double {mustBeFinite, mustBePositive} = 5 % [-]   Mesh refinement level: 1 (extremely fine) to 9 (extremely coarse)

      mu_r_shaft (1,1) double {mustBeFinite, mustBePositive} = 1
      sigma_shaft (1,1) double {mustBeFinite, mustBeNonnegative} = 1.4e6
      epsilon_r_shaft (1,1) double {mustBeFinite, mustBeNonnegative} = 0.8

      mu_r_iron (1,1) double {mustBeFinite, mustBePositive} = 5000
      sigma_iron (1,1) double {mustBeFinite, mustBeNonnegative} = 2e6
      epsilon_r_iron (1,1) double {mustBeFinite, mustBeNonnegative} = 0.8

      mu_r_air (1,1) double {mustBeFinite, mustBePositive} = 1
      sigma_air (1,1) double {mustBeFinite, mustBeNonnegative} = 0
      epsilon_r_air (1,1) double {mustBeFinite, mustBeNonnegative} = 1

      mu_r_magnets (1,1) double {mustBeFinite, mustBePositive} = 1.05
      sigma_magnets (1,1) double {mustBeFinite, mustBeNonnegative} = 6.25e5
      epsilon_r_magnets (1,1) double {mustBeFinite, mustBeNonnegative} = 1
      Br (1,1) double {mustBeFinite, mustBePositive} = 1.3

      epsilon_r_copper  (1,1) double {mustBeFinite, mustBeNonnegative} = 1
      mu_r_copper     (1,1) double {mustBeFinite, mustBePositive} = 1;        % [-]   Relative permeability of copper
      sigma_copper    (1,1) double {mustBeFinite, mustBeNonnegative} = 5.8e7;    % [S/m] Electrical conductivity of copper

      % --- Mechanical (rotor bridge stress check — see doc/rotor_bridge_stress_notes.md) ---
      % Defaults are for the paper's named grades (M235-35A lamination,
      % N48UZ-SGR PM), pulled from manufacturer datasheets, not the paper.
      rho_lam     (1,1) double {mustBeFinite, mustBePositive} = 7650   % [kg/m^3] M235-35A lamination density
      sigma_y_lam (1,1) double {mustBeFinite, mustBePositive} = 460e6  % [Pa] M235-35A yield strength (0.2% proof, rolling dir.)
      rho_pm      (1,1) double {mustBeFinite, mustBePositive} = 7500   % [kg/m^3] sintered NdFeB density
      Kt_ib       (1,1) double {mustBeFinite, mustBePositive} = 1.66   % [-] inner bridge stress concentration factor (Di Gerlando & Ricca, eq. 21)

   end
end
