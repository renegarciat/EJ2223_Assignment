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
      % PM remanent flux density at 20degC (the magnet grade's datasheet
      % value -- e.g. 1.37T for the project's N48 choice, see the design
      % report's Magnet Material Selection section). Deliberately lives
      % here rather than in MotorSpec: which magnet grade to use is a
      % decision for the rotor-sizing stage (IPMRotorSizer.Br_T
      % temperature-corrects this using MotorSpec.PMTemp_C/kBr_pctPerC),
      % not something the initial torque/geometry sizing (MotorSpec,
      % EssonsSizer) needs to commit to.
      Br (1,1) double {mustBeFinite, mustBePositive} = 1.37

      epsilon_r_copper  (1,1) double {mustBeFinite, mustBeNonnegative} = 1
      mu_r_copper     (1,1) double {mustBeFinite, mustBePositive} = 1;        % [-]   Relative permeability of copper
      sigma_copper    (1,1) double {mustBeFinite, mustBeNonnegative} = 5.8e7;    % [S/m] Electrical conductivity of copper

      % --- Mechanical (rotor bridge stress check — see README.md's "Rotor bridge stress check" section) ---
      % Defaults are for the paper's named grades (M235-35A lamination,
      % N48UZ-SGR PM), pulled from manufacturer datasheets, not the paper.
      rho_lam     (1,1) double {mustBeFinite, mustBePositive} = 7650   % [kg/m^3] M235-35A lamination density
      sigma_y_lam (1,1) double {mustBeFinite, mustBePositive} = 460e6  % [Pa] M235-35A yield strength (0.2% proof, rolling dir.)
      rho_pm      (1,1) double {mustBeFinite, mustBePositive} = 7500   % [kg/m^3] sintered NdFeB density
      Kt_ib       (1,1) double {mustBeFinite, mustBePositive} = 1.66   % [-] inner bridge stress concentration factor (Di Gerlando & Ricca, eq. 21)

      % --- Core loss (used by EfficiencyMapSizer) ---
      % M235-35A's EN 10106 grade name is literally its specific total
      % loss rating: "235" = P_1.5/50 = 2.35 W/kg at 1.5T/50Hz, 0.35mm
      % lamination thickness. No manufacturer split of that figure into
      % hysteresis/eddy components is available to this project, so
      % CoreLossHystFrac_50Hz is a representative assumption for thin
      % (0.35mm) non-oriented electrical steel at 50Hz, not a datasheet
      % value -- flagged the same way as the other calibrated
      % approximations in this codebase (see README known issues).
      CoreLoss_Wkg_1p5T_50Hz (1,1) double {mustBeFinite, mustBePositive} = 2.35
      CoreLossHystFrac_50Hz  (1,1) double {mustBeInRange(CoreLossHystFrac_50Hz, 0, 1)} = 0.65

   end
end
