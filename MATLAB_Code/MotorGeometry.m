classdef MotorGeometry
% MOTORGEOMETRY
%
%   This class assembles the physical geometry (stator/rotor dimensions,
%   corner-point currents) from the sizing pipeline's results
%   (EssonsSizer, IPMRotorSizer). ComsolPrebuiltInterface.pushGeometry()
%   pulls individual scalar fields off it to push into the prebuilt
%   COMSOL model -- see main.m.

	% =====================================================================
	% Stator inputs
	% =====================================================================
	properties
		% Paper | COMSOL | Description
		StatorInnerRadius_m (1,1) double = NaN   % r_si [m]
		StatorOuterRadius_m (1,1) double = NaN   % r_so [m]
		Slots               (1,1) double = NaN   % Qs  [-]
		PolePairs           (1,1) double = NaN   % p   [-]
		SlotDepth_m         (1,1) double = NaN   % slot_depth [m]. Full radial depth from the bore:
		                                          % slot opening (h_as) + slot body (h).
		SlotWidth_m         (1,1) double = NaN   % slot_width [m]
		ToothHeight_m       (1,1) double = NaN   % [m]. Radial tooth height. Teeth and slots occupy
		                                          % the same radial band (bore to yoke), just
		                                          % different angular sectors, so this always equals
		                                          % SlotDepth_m. NOT the saturation model's h_teq
		                                          % (IPMRotorSizer, eq. 90) -- that's a proxy that
		                                          % also folds in b_1/2 and the rotor's half-rib
		                                          % height, not a drawable dimension.
		StatorYokeHeight_m  (1,1) double = NaN   % h_sy [m]. Back-iron thickness beyond the
		                                          % slots, i.e. (StatorOuterRadius_m -
		                                          % StatorInnerRadius_m) - SlotDepth_m (eq. 93).
		Id_A                (1,1) double = NaN   % direct-axis current (for logging)
		Iq_A                (1,1) double = NaN   % quadrature
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
				error('MotorGeometry:essonsNotSolved', 'EssonsSizer.solveD2L() must be called before building MotorGeometry.');
			end
			if ~rotorSizer.Converged
				error('MotorGeometry:rotorNotSolved', 'IPMRotorSizer.solve() must be called before building MotorGeometry.');
			end

			% Convert and assemble fields (units: m, rad). Stator OD/slot
			% geometry come from rotorSizer (IPMRotorSizer's own eqs.
			% 65-94 winding/core sizing), not essonsSizer: EssonsSizer's
			% D^2L formula (solveD2L()) was simplified to just Dis/le/
			% tau_p/D_r and no longer derives Dos or slot dimensions --
			% see EssonsSizer.m's header note.
			r_si = rotorSizer.StatorBore_mm / 2000; % mm -> m radius
			r_so = rotorSizer.StatorOD_mm / 2000;   % mm -> m radius
			Qs = spec.Slots;
			p = spec.Poles / 2;
			slot_depth = (rotorSizer.Has_mm + rotorSizer.SlotHeight_mm) / 1000;  % mm -> m, h_as+h, bore to yoke
			slot_width = mean([rotorSizer.SlotWidthInner_mm, rotorSizer.SlotWidthOuter_mm]) / 1000; % mm -> m, trapezoidal slot averaged to a single width
			tooth_height = slot_depth; % teeth and slots share the same radial band, just different angular sectors
			yoke_height = rotorSizer.StatorYokeHeight_mm / 1000; % mm -> m (eq. 93)

			D_r = rotorSizer.RotorOD_mm / 2000;    % mm -> m radius
			D_ir = rotorSizer.RotorID_mm / 2000;   % mm -> m radius
			air_gap = spec.Airgap_mm / 1000;       % mm -> m
			b_m = rotorSizer.Bm_mm / 1000;         % mm -> m
			h_m = rotorSizer.Hm_mm / 1000;         % mm -> m
			w_ib = rotorSizer.Wib_mm / 1000;      % mm -> m
			h_ry = rotorSizer.Hry_mm / 1000;      % mm -> m
			angle_m = rotorSizer.Vtilt_deg * pi/180;
			Iq_A = rotorSizer.Iq_A;
			Id_A = rotorSizer.Id_A;

			obj = MotorGeometry( ...
				StatorInnerRadius_m = r_si, ...
				StatorOuterRadius_m = r_so, ...
				Slots = Qs, ...
				PolePairs = p, ...
				SlotDepth_m = slot_depth, ...
				SlotWidth_m = slot_width, ...
				ToothHeight_m = tooth_height, ...
				StatorYokeHeight_m = yoke_height, ...
				RotorOuterRadius_m = D_r, ...
				RotorInnerRadius_m = D_ir, ...
				Airgap_m = air_gap, ...
				MagnetLength_m = b_m, ...
				MagnetWidth_m = h_m, ...
				MagnetSpacing_m = w_ib, ...
				MagnetRibHeight_m = h_ry, ...
				MagnetAngle_rad = angle_m, ...
				Iq_A = Iq_A, Id_A = Id_A ...
				);
		end
	end
	% =====================================================================
	% Rotor inputs
	% =====================================================================
	properties
		RotorOuterRadius_m  (1,1) double = NaN   % D_r [m]
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
				options.ToothHeight_m       (1,1) double = NaN
				options.StatorYokeHeight_m  (1,1) double = NaN

				options.RotorOuterRadius_m  (1,1) double = NaN
				options.RotorInnerRadius_m  (1,1) double = NaN
				options.Airgap_m            (1,1) double = NaN
				options.MagnetLength_m      (1,1) double = NaN
				options.MagnetWidth_m       (1,1) double = NaN
				options.MagnetSpacing_m     (1,1) double = NaN
				options.MagnetRibHeight_m   (1,1) double = NaN
				options.MagnetAngle_rad     (1,1) double = NaN
				options.Id_A                (1,1) double = NaN
				options.Iq_A                (1,1) double = NaN
			end

			obj.StatorInnerRadius_m = options.StatorInnerRadius_m;
			obj.StatorOuterRadius_m = options.StatorOuterRadius_m;
			obj.Slots               = options.Slots;
			obj.PolePairs           = options.PolePairs;
			obj.SlotDepth_m         = options.SlotDepth_m;
			obj.SlotWidth_m         = options.SlotWidth_m;
			obj.ToothHeight_m       = options.ToothHeight_m;
			obj.StatorYokeHeight_m  = options.StatorYokeHeight_m;

			obj.RotorOuterRadius_m  = options.RotorOuterRadius_m;
			obj.RotorInnerRadius_m  = options.RotorInnerRadius_m;
			obj.Airgap_m            = options.Airgap_m;
			obj.MagnetLength_m      = options.MagnetLength_m;
			obj.MagnetWidth_m       = options.MagnetWidth_m;
			obj.MagnetSpacing_m     = options.MagnetSpacing_m;
			obj.MagnetRibHeight_m   = options.MagnetRibHeight_m;
			obj.MagnetAngle_rad     = options.MagnetAngle_rad;
			obj.Id_A                = options.Id_A;
			obj.Iq_A                = options.Iq_A;
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
			s.ToothHeight_m       = obj.ToothHeight_m;
			s.StatorYokeHeight_m  = obj.StatorYokeHeight_m;

			% Rotor
			s.RotorOuterRadius_m  = obj.RotorOuterRadius_m;
			s.RotorInnerRadius_m  = obj.RotorInnerRadius_m;
			s.Airgap_m            = obj.Airgap_m;
			s.MagnetLength_m      = obj.MagnetLength_m;
			s.MagnetWidth_m       = obj.MagnetWidth_m;
			s.MagnetSpacing_m     = obj.MagnetSpacing_m;
			s.MagnetRibHeight_m   = obj.MagnetRibHeight_m;
			s.MagnetAngle_rad     = obj.MagnetAngle_rad;
			s.Id_A                = obj.Id_A;
			s.Iq_A                = obj.Iq_A;
		end

		function summary(obj)
			% summary  Print a formatted overview of the assembled geometry.
			fprintf('\nMotorGeometry summary\n');
			fprintf('%s\n', repmat('-', 1, 52));
			fprintf('  %-30s %.2f mm\n', 'Stator inner diameter (D_si):', obj.StatorInnerRadius_m*2e3);
			fprintf('  %-30s %.2f mm\n', 'Stator outer diameter (D_os):', obj.StatorOuterRadius_m*2e3);
			fprintf('  %-30s %d\n',      'Slots:',                     obj.Slots);
			fprintf('  %-30s %d\n',      'Pole pairs:',                obj.PolePairs);
			fprintf('  %-30s %.2f mm\n', 'Slot depth (bore to yoke):', obj.SlotDepth_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'Slot width (avg):',          obj.SlotWidth_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'Tooth height (= slot depth):', obj.ToothHeight_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'Stator yoke height (h_sy):', obj.StatorYokeHeight_m*1e3);
			fprintf('\n');
			fprintf('  %-30s %.2f mm\n', 'Rotor outer diameter (D_or):',  obj.RotorOuterRadius_m*2e3);
			fprintf('  %-30s %.2f mm\n', 'Rotor inner diameter (D_ir):', obj.RotorInnerRadius_m*2e3);
			fprintf('  %-30s %.2f mm\n', 'Airgap:',                    obj.Airgap_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'Magnet length (b_m):',       obj.MagnetLength_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'Magnet width (h_m):',        obj.MagnetWidth_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'Magnet spacing (w_ib):',     obj.MagnetSpacing_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'Magnet rib height (h_ry):',  obj.MagnetRibHeight_m*1e3);
			fprintf('  %-30s %.2f deg\n','Magnet angle:',              obj.MagnetAngle_rad*180/pi);
			fprintf('  %-30s %.2f A\n',  'Id (corner):',               obj.Id_A);
			fprintf('  %-30s %.2f A\n',  'Iq (corner):',               obj.Iq_A);
			fprintf('%s\n\n', repmat('-', 1, 52));
		end
	end
end
