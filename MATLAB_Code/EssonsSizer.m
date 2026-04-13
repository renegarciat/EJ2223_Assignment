classdef EssonsSizer < handle
% EssonsSizer  Esson sizing estimate (class-based port of EssonsEstimation).
%
%   Provides the same computations as EssonsEstimation.m, but packaged as a
%   handle class with a solve()/summary() workflow similar to IPMRotorSizer.
%
%   Usage
%   -----
%     spec  = MotorSpec(...);
%     sizer = EssonsSizer(spec);
%     sizer.solve();
%     sizer.summary();
%
%   Notes
%   -----
%   - All calculations follow EssonsEstimation.m exactly (including the
%     same warning conditions and root-bracketing behavior).
%   - Length results are stored in meters (suffix _m), matching the
%     original function outputs.

	% =====================================================================
	% Public configuration
	% =====================================================================
	properties
		% Threshold used for warning (matches D_o_old in EssonsEstimation).
		StatorODWarnLimit_m (1,1) double {mustBePositive} = 120e-3
	end

	% =====================================================================
	% Read-only results — written by solve()
	% =====================================================================
	properties (SetAccess = private)

		% --- Results (same as EssonsEstimation outputs) ---
		Dis_m    (1,1) double = NaN   % inner stator diameter
		le_m     (1,1) double = NaN   % effective length
		tau_p_m  (1,1) double = NaN   % pole pitch
		tau_s_m  (1,1) double = NaN   % stator pitch
		t_s_m    (1,1) double = NaN   % stator tooth distance
		h_slot_m (1,1) double = NaN   % slot height
		h_cs_m   (1,1) double = NaN   % back iron height
		Dro_m    (1,1) double = NaN   % outer rotor diameter
		Dso_m    (1,1) double = NaN   % outer stator diameter
		Ratio    (1,1) double = NaN   % Dis/Dso
		q_spp    (1,1) double = NaN   % slots per pole per phase

		Solved   (1,1) logical = false
	end

	% =====================================================================
	% Convenience aliases (computed) — for compatibility with older code
	% =====================================================================
	properties (Dependent, SetAccess = private)
		StatorBore_mm   % Alias for Dis_m in millimeters
	end

	% =====================================================================
	% Private — reference to MotorSpec
	% =====================================================================
	properties (Access = private)
		spec_ MotorSpec
	end

	% =====================================================================
	% Constructor
	% =====================================================================
	methods
		function obj = EssonsSizer(spec, options)
			% EssonsSizer  Create sizer bound to a MotorSpec.
			arguments
				spec (1,1) MotorSpec
				options.StatorODWarnLimit_m (1,1) double = 120e-3
			end

			obj.spec_ = spec;
			obj.StatorODWarnLimit_m = options.StatorODWarnLimit_m;
		end
	end

	% =====================================================================
	% Public methods
	% =====================================================================
	methods (Access = public)
		function solve(obj)
			% solve  Run the Esson sizing estimate and store results.
			s = obj.spec_;

			% Map MotorSpec → EssonsEstimation inputs.
			T_Nm = s.Torque_Nm;
			P    = s.Poles;
			Q    = s.Slots;
			m    = s.Phases;
			Bg1  = s.Bg1_T;
			Bt   = s.Bt_T;
			Bc   = s.Bc_T;
			A1   = s.LinearCurrentDensity_Am;
			J_rms_Amm2 = s.CurrentDensity_Amm2;
			AspectRatio = s.AspectRatio;
			eta_estimation = s.EfficiencyEstimate;
			cos_phi = s.PowerFactor;
			airgap_estimation_m = s.Airgap_mm * 1e-3;
			stackingfactor = s.StackingFactor;
			kis = s.IronFillFactor;
			dos_m = s.SlotOpening_mm * 1e-3;
			kcu = s.CopperFillFactor;

			[Dis, le, tau_p, tau_s, t_s, h_slot, h_cs, Dro, Dso, ratio, q_spp_val] = ...
				obj.compute_(T_Nm, P, Q, m, Bg1, Bt, Bc, A1, J_rms_Amm2, ...
							 AspectRatio, eta_estimation, cos_phi, airgap_estimation_m, ...
							 stackingfactor, kis, dos_m, kcu);

			% Geometry sanity checks that require the computed bore.
			if ~(isfinite(Dis) && Dis > 0)
				error('EssonsSizer:invalidBore', ...
					'Computed stator bore (Dis) must be finite and > 0. Got: %g m', Dis);
			end
			if airgap_estimation_m >= Dis/2
				error('EssonsSizer:invalidGeometry', ...
					['Airgap (g=%g m) must be much smaller than the computed stator bore/2 (Dis/2=%g m). ' ...
					 'Check MotorSpec.Airgap_mm and/or the Esson sizing inputs.'], ...
					airgap_estimation_m, Dis/2);
			end
			if ~(isfinite(Dro) && Dro > 0)
				error('EssonsSizer:invalidRotorDiameter', ...
					'Computed rotor outer diameter (Dro) must be finite and > 0. Got: %g m', Dro);
			end

			obj.Dis_m    = Dis;
			obj.le_m     = le;
			obj.tau_p_m  = tau_p;
			obj.tau_s_m  = tau_s;
			obj.t_s_m    = t_s;
			obj.h_slot_m = h_slot;
			obj.h_cs_m   = h_cs;
			obj.Dro_m    = Dro;
			obj.Dso_m    = Dso;
			obj.Ratio    = ratio;
			obj.q_spp    = q_spp_val;
			obj.Solved   = true;
		end

		function summary(obj)
			% summary  Print a formatted overview of the Esson estimate.
			obj.requireSolved_('summary');

			s = obj.spec_;
			fprintf('\nEssonsSizer results\n');
			fprintf('%s\n', repmat('-', 1, 52));
			fprintf('  %-30s %g Nm\n', 'Torque target:', s.Torque_Nm);
			fprintf('  %-30s %d\n',   'Poles (P):',     s.Poles);
			fprintf('  %-30s %g\n',   'Slots (Q):',     s.Slots);
			fprintf('  %-30s %d\n',   'Phases (m):',    s.Phases);
			fprintf('\n');
			fprintf('  %-30s %.2f mm\n', 'Dis:',    obj.Dis_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'Dso:',    obj.Dso_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'Dro:',    obj.Dro_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'le:',     obj.le_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'tau_p:',  obj.tau_p_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'tau_s:',  obj.tau_s_m*1e3);
			fprintf('  %-30s %.2f mm\n', 't_s:',    obj.t_s_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'h_slot:', obj.h_slot_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'h_cs:',   obj.h_cs_m*1e3);
			fprintf('  %-30s %.4f\n',    'Dis/Dso:', obj.Ratio);
			fprintf('  %-30s %.4f\n',    'q_spp:',   obj.q_spp);
			fprintf('%s\n\n', repmat('-', 1, 52));
		end

		function s = toStruct(obj)
			% toStruct  Export inputs (from spec) and computed results.
			s = struct();

			specStruct = obj.spec_.toStruct();
			s.spec = specStruct;

			s.results = struct();
			s.results.Dis_m    = obj.Dis_m;
			s.results.le_m     = obj.le_m;
			s.results.tau_p_m  = obj.tau_p_m;
			s.results.tau_s_m  = obj.tau_s_m;
			s.results.t_s_m    = obj.t_s_m;
			s.results.h_slot_m = obj.h_slot_m;
			s.results.h_cs_m   = obj.h_cs_m;
			s.results.Dro_m    = obj.Dro_m;
			s.results.Dso_m    = obj.Dso_m;
			s.results.Ratio    = obj.Ratio;
			s.results.q_spp    = obj.q_spp;
			s.results.Solved   = obj.Solved;
		end
	end

	% =====================================================================
	% Dependent property getters
	% =====================================================================
	methods
		function Dmm = get.StatorBore_mm(obj)
			obj.requireSolved_('StatorBore_mm');
			Dmm = obj.Dis_m * 1e3;
		end
	end

	% =====================================================================
	% Private — computation (exact port of EssonsEstimation.m)
	% =====================================================================
	methods (Access = private)
		function [Dis, le, tau_p, tau_s, t_s, h_slot, h_cs, Dro, Dso, ratio, q_spp_val] = ...
				compute_(obj, T_Nm, P, Q, m, Bg1, Bt, Bc, A1, J_rms_Amm2, ...
						 AspectRatio, eta_estimation, cos_phi, airgap_estimation, ...
						 stackingfactor, kis, dos, kcu)

			% ---------- Calculation constants (as in EssonsEstimation) -----
			D_o_old = obj.StatorODWarnLimit_m;
			sigma_m = A1 * Bg1 / sqrt(2); % FIXME: A1 Peak or RMS?
			q_spp_val = Q / (m * P);
			J_rms = J_rms_Amm2 * 1e6;

			% ---------- Calculation inner diameter of stator ---------------
			Volume = T_Nm / (2 * sigma_m * eta_estimation * cos_phi);
			A = (Volume * 4*pi*sqrt(AspectRatio) / sqrt(P))^(2/3);      % 6.33 Lipo Book
			Dis = 1/pi * sqrt(P * A / AspectRatio);                      % 6.32 Lipo Book
			tau_p = pi * Dis / P;
			le = AspectRatio * tau_p;

			% ---------- Calculation outer Diameter and Slots ---------------
			tau_s = pi * Dis / Q;
			t_s = tau_s * 1/kis * (Bg1/Bt) * stackingfactor;
			h_cs = Dis/(P*kis) * (Bg1/Bc) * stackingfactor;

			% Calculate the fraction of the stator bore that is available
			% for the slot opening (i.e., not taken up by the teeth)
			a = (Bg1/(kis*Bt)*stackingfactor + 2/P*Bg1/(kis*Bc)*stackingfactor)^2 - ...
				(1 - Bg1/(Bt*kis)*stackingfactor)^2;
			b = (Bg1/(kis*Bt) + 2/P*Bg1/(kis*Bc)) * stackingfactor;

			% NOTE: This formula takes A1, if its peak, convert to RMS by dividing by sqrt(2)
			fun = @(Dos) EssonsSizer.calc_Dis_from_Dos_(Dos, a, b, A1, kcu, J_rms) - Dis;
			Dso_min = Dis / 0.8;
			Dso_max = Dis / 0.2;
			Dso = EssonsSizer.find_root_bracketed_(fun, Dso_min, Dso_max);
			if Dso > D_o_old
				warning('Outer diameter stator is bigger than old design');
			end

			ratio = Dis / Dso;
			zeta = (3*a*ratio^2 - 4*b*ratio + 1) / ((kcu*J_rms/4) * ((1/ratio)^2 - a));
			if zeta > 0
				warning('Constraint inactive. Increase k_cu or reduce J_rms');
			end

			b1 = pi/Q * (Dis * (1 - Bg1/(kis*Bt)*stackingfactor) + 2*dos);
			b2 = pi/Q * (Dso - Dis * (Bg1/(kis*Bt)*stackingfactor + 2/P*Bg1/(Bc*kis)*stackingfactor));
			h_slot = Q/(2*pi) * (b2 - b1);

			Dos_test = Dis + 2*(h_cs + dos + h_slot); %#ok<NASGU> % kept for parity with EssonsEstimation.m

			Dro = Dis - 2*airgap_estimation;
		end

		function requireSolved_(obj, caller)
			if ~obj.Solved
				error('EssonsSizer:notSolved', ...
					'EssonsSizer.%s() called before solve().', caller);
			end
		end
	end

	% =====================================================================
	% Private static helpers (ported from EssonsEstimation.m)
	% =====================================================================
	methods (Static, Access = private)
		function Dis_calc = calc_Dis_from_Dos_(Dos, a, b, A1, kcu, J_rms)
			x = b/a + 2*A1/(a*kcu*J_rms*Dos);
			rad = x^2 - 1/a;
			if rad < 0
				warning('sqare wave term is negative');
				Dis_calc = NaN;
				return
			end
			r = x - sqrt(rad);
			if r <= 0 || r >= 1
				Dis_calc = NaN;
				warning('Wrong ratio');
				return
			end
			Dis_calc = Dos * r;
		end

		function root = find_root_bracketed_(fun, xmin, xmax)
			% FIND_ROOT_BRACKETED Robust wrapper around fzero.
			% Ensures the interval contains finite values and a sign change.

			if ~(isfinite(xmin) && isfinite(xmax) && xmax > xmin)
				error('Invalid root search interval.');
			end

			% Sample the interval and locate a finite sign change.
			xs = linspace(xmin, xmax, 300);
			ys = arrayfun(fun, xs);

			finiteMask = isfinite(ys);
			xs = xs(finiteMask);
			ys = ys(finiteMask);

			if numel(xs) < 2
				error('Root finding failed: function is not finite on the interval.');
			end

			s = sign(ys);

			idxZero = find(s == 0, 1, 'first');
			if ~isempty(idxZero)
				root = xs(idxZero);
				return;
			end

			for i = 1:(numel(xs)-1)
				if s(i) * s(i+1) < 0
					root = fzero(fun, [xs(i), xs(i+1)]);
					return;
				end
			end

			% Fallback: try from best finite initial guess.
			[~, idx] = min(abs(ys));
			root = fzero(fun, xs(idx));
		end
	end
end

