classdef MotorGeometry
% MotorGeometry  Plain value object containing COMSOL-ready geometry inputs.
%
%   This class stores the physical parameters needed by the COMSOL drawing
%   scripts.
%
%   End goal:
%   - IPMRotorSizer + EssonsSizer populate a MotorGeometry instance
%
%   Units
%   -----
%   All lengths are stored in meters (_m). Angles in radians (_rad).
%   Radii are used where the COMSOL scripts expect radii.

	% =====================================================================
	% Stator inputs (draw_stator_sector)
	% =====================================================================
	properties
		StatorInnerRadius_m (1,1) double = NaN   % r_si [m]
		StatorOuterRadius_m (1,1) double = NaN   % r_so [m]
		Slots               (1,1) double = NaN   % Qs  [-]
		PolePairs           (1,1) double = NaN   % p   [-]
		SlotDepth_m         (1,1) double = NaN   % slot_depth [m]
		SlotWidth_m         (1,1) double = NaN   % slot_width [m]
		RadialOffset_m	  	(1,1) double = 3e-3  % d_os [m]. Radial offset from Stator Inner radius to slot opening.
		SlotRadialDepth_m   (1,1) double = 20e-3 % d_s [m]. 
		SlotWidthInner_m    (1,1) double = 2e-3  % b_1 [m]. Slot bottom width
		SlotWidthOuter_m    (1,1) double = 4e-3  % b_2 [m]. slot_width_outer [m]
		ChamferRadius_m     (1,1) double = 0.5e-3% r_1 [m]. Radius of rounded slot edge
		DrawOnlySector      (1,1) logical = true
	end

	methods (Static)
		function obj = fromSizingResults(spec, essonsSizer, rotorSizer)
			% fromSizingResults  Build a MotorGeometry from sizing objects.
			%   obj = MotorGeometry.fromSizingResults(spec, essonsSizer, rotorSizer)
			arguments
				spec MotorSpec
				essonsSizer EssonsSizer
				rotorSizer IPMRotorSizer
			end

			if ~essonsSizer.Solved
				error('MotorGeometry:essonsNotSolved', 'EssonsSizer.solve() must be called before building MotorGeometry.');
			end
			if ~rotorSizer.Converged
				error('MotorGeometry:rotorNotSolved', 'IPMRotorSizer.solve() must be called before building MotorGeometry.');
			end

			% Convert and assemble fields (units: m, rad)
			r_si = essonsSizer.Dis_m / 2;
			r_so = essonsSizer.Dso_m / 2;
			Qs = spec.Slots;
			p = spec.Poles / 2;
			slot_depth = essonsSizer.h_slot_m;
			slot_width = essonsSizer.tau_s_m - essonsSizer.t_s_m;
			draw_only = true;

			D_r = rotorSizer.RotorOD_mm / 2000;    % mm -> m radius
			D_ir = rotorSizer.RotorID_mm / 2000;   % mm -> m radius
			air_gap = spec.Airgap_mm / 1000;       % mm -> m
			b_m = rotorSizer.Bm_mm / 1000;         % mm -> m
			h_m = rotorSizer.Hm_mm / 1000;         % mm -> m
			w_ib = rotorSizer.Wib_mm / 1000;      % mm -> m
			h_ry = rotorSizer.Hry_mm / 1000;      % mm -> m
			angle_m = rotorSizer.Vtilt_deg * pi/180;

			obj = MotorGeometry( ...
				StatorInnerRadius_m = r_si, ...
				StatorOuterRadius_m = r_so, ...
				Slots = Qs, ...
				PolePairs = p, ...
				SlotDepth_m = slot_depth, ...
				SlotWidth_m = slot_width, ...
				DrawOnlySector = draw_only, ...
				RotorOuterRadius_m = D_r, ...
				RotorInnerRadius_m = D_ir, ...
				Airgap_m = air_gap, ...
				MagnetLength_m = b_m, ...
				MagnetWidth_m = h_m, ...
				MagnetSpacing_m = w_ib, ...
				MagnetRibHeight_m = h_ry, ...
				MagnetAngle_rad = angle_m ...
				);
		end
	end
	% =====================================================================
	% Rotor inputs (draw_rotor_sector)
	% =====================================================================
	properties
		RotorOuterRadius_m  (1,1) double = NaN   % D_r [m] (naming from script)
		RotorInnerRadius_m  (1,1) double = NaN   % D_ir [m]
		Airgap_m            (1,1) double = NaN   % air_gap [m]

		MagnetLength_m      (1,1) double = NaN   % b_m [m]
		MagnetWidth_m       (1,1) double = NaN   % h_m [m]
		MagnetSpacing_m     (1,1) double = NaN   % w_ib [m]
		MagnetRibHeight_m   (1,1) double = NaN   % h_ry [m]
		MagnetAngle_rad     (1,1) double = NaN   % angle_m [rad]
	end

	% =====================================================================
	% Constructor
	% =====================================================================
	methods
		function obj = MotorGeometry(options)
			% MotorGeometry Construct with Name=Value overrides.
			arguments
				options.StatorInnerRadius_m (1,1) double = NaN
				options.StatorOuterRadius_m (1,1) double = NaN
				options.Slots               (1,1) double = NaN
				options.PolePairs           (1,1) double = NaN
				options.SlotDepth_m         (1,1) double = NaN
				options.SlotWidth_m         (1,1) double = NaN
				options.DrawOnlySector      (1,1) logical = true

				options.RotorOuterRadius_m  (1,1) double = NaN
				options.RotorInnerRadius_m  (1,1) double = NaN
				options.Airgap_m            (1,1) double = NaN
				options.MagnetLength_m      (1,1) double = NaN
				options.MagnetWidth_m       (1,1) double = NaN
				options.MagnetSpacing_m     (1,1) double = NaN
				options.MagnetRibHeight_m   (1,1) double = NaN
				options.MagnetAngle_rad     (1,1) double = NaN
			end

			obj.StatorInnerRadius_m = options.StatorInnerRadius_m;
			obj.StatorOuterRadius_m = options.StatorOuterRadius_m;
			obj.Slots               = options.Slots;
			obj.PolePairs           = options.PolePairs;
			obj.SlotDepth_m         = options.SlotDepth_m;
			obj.SlotWidth_m         = options.SlotWidth_m;
			obj.DrawOnlySector      = options.DrawOnlySector;

			obj.RotorOuterRadius_m  = options.RotorOuterRadius_m;
			obj.RotorInnerRadius_m  = options.RotorInnerRadius_m;
			obj.Airgap_m            = options.Airgap_m;
			obj.MagnetLength_m      = options.MagnetLength_m;
			obj.MagnetWidth_m       = options.MagnetWidth_m;
			obj.MagnetSpacing_m     = options.MagnetSpacing_m;
			obj.MagnetRibHeight_m   = options.MagnetRibHeight_m;
			obj.MagnetAngle_rad     = options.MagnetAngle_rad;
		end

		function s = toStruct(obj)
			% toStruct  Export to a plain struct (for logging/serialization).
			s = struct();

			% Stator
			s.StatorInnerRadius_m = obj.StatorInnerRadius_m;
			s.StatorOuterRadius_m = obj.StatorOuterRadius_m;
			s.Slots               = obj.Slots;
			s.PolePairs           = obj.PolePairs;
			s.SlotDepth_m         = obj.SlotDepth_m;
			s.SlotWidth_m         = obj.SlotWidth_m;
			s.DrawOnlySector      = obj.DrawOnlySector;

			% Rotor
			s.RotorOuterRadius_m  = obj.RotorOuterRadius_m;
			s.RotorInnerRadius_m  = obj.RotorInnerRadius_m;
			s.Airgap_m            = obj.Airgap_m;
			s.MagnetLength_m      = obj.MagnetLength_m;
			s.MagnetWidth_m       = obj.MagnetWidth_m;
			s.MagnetSpacing_m     = obj.MagnetSpacing_m;
			s.MagnetRibHeight_m   = obj.MagnetRibHeight_m;
			s.MagnetAngle_rad     = obj.MagnetAngle_rad;
		end
	end
end
