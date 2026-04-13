classdef MotorMaterials
   % MotorMaterials Typed container for COMSOL material constants.
   %
   % This class intentionally defines properties only (no methods).

   properties
      mesh_size (1,1) double {mustBeFinite, mustBePositive} = 5

      mu_r_shaft (1,1) double {mustBeFinite, mustBePositive} = 1
      sigma_shaft (1,1) double {mustBeFinite, mustBeNonnegative} = 1.4e6
      epsilon_r_shaft (1,1) double {mustBeFinite, mustBeNonnegative} = 0.8

      mu_r_iron (1,1) double {mustBeFinite, mustBePositive} = 5000
      sigma_iron (1,1) double {mustBeFinite, mustBeNonnegative} = 2e6
      epsilon_r_iron (1,1) double {mustBeFinite, mustBeNonnegative} = 0.8

      mu_r_air (1,1) double {mustBeFinite, mustBePositive} = 1
      sigma_air (1,1) double {mustBeFinite, mustBeNonnegative} = 0

      mu_r_magnets (1,1) double {mustBeFinite, mustBePositive} = 1.05
      sigma_magnets (1,1) double {mustBeFinite, mustBeNonnegative} = 6.25e5
      Br (1,1) double {mustBeFinite, mustBePositive} = 1.3
   end
end
