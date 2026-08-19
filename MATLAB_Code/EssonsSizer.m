classdef EssonsSizer < handle
% 	ESSONSSIZER Calculate sizing estimates according to 2 Esson's rules.
%   Usage:
%     spec  = MotorSpec(...);
%     sizer = EssonsSizer(spec);
%     sizer.solveD2L();               % D^2 formula
%     sizer.summary();
%     sizer.solveD3L(0.150);       % D^3L formula, Dos = 150mm target
%     sizer.summaryD3L();
%
%   Notes
%   -----
%   - Length results are stored in meters (suffix _m).
%   - MotorSpec.LinearCurrentDensity_Am is Ks,rms/Ks,1's RMS value (see
%     MotorSpec.m) and is reused as Ks,rms,max in solveD3L().
%   - solveD3L() shows the fo(rho)/cross-section optimum plot (see
%     plotD3LOptimum_) every time it is called, by default. Pass
%     MakePlot=false (e.g. sizer.solveD3L(Dos, MakePlot=false)) to
%     suppress it -- needed when solveD3L() is called repeatedly, e.g.
%     inside a sweep or an fzero root-find, to avoid a figure per call.

	% =====================================================================
	% Public configuration
	% =====================================================================
	properties
		% Winding factor k_1, used only by solveD3L() (Lecture 6's D^3L
		% output equation is the only Esson formula here that needs it
		% explicitly -- solveD2L()'s D^2 formula folds it into A1/Ks,1).
		% Default matches IPMRotorSizer.WindingFactor.
		WindingFactor (1,1) double {mustBeInRange(WindingFactor, 0.5, 1.0)} = 0.91
	end

	% =====================================================================
	% Read-only results — written by solveD2L()
	% =====================================================================
	properties (SetAccess = private)

		% --- D^2L Results ---
		Dis_m    (1,1) double = NaN   % inner stator diameter
		le_m     (1,1) double = NaN   % effective length
		tau_p_m  (1,1) double = NaN   % pole pitch
		Dro_m    (1,1) double = NaN   % outer rotor diameter

		Solved   (1,1) logical = false

		% --- D^3L results (see solveD3L()) ---
		Dis_D3L_m         (1,1) double = NaN   % optimal inner stator diameter for the given Dos
		le_D3L_m          (1,1) double = NaN   % effective length hitting the torque target
		Dos_D3L_m         (1,1) double = NaN   % target outer diameter (echoed input)
		tau_p_D3L_m       (1,1) double = NaN   % pole pitch at Dis_D3L_m
		Ratio_D3L         (1,1) double = NaN   % Dis_D3L_m / Dos_D3L_m
		AspectRatio_D3L   (1,1) double = NaN   % le_D3L_m / tau_p_D3L_m
		FoAtOptimum_D3L   (1,1) double = NaN   % fo(Dis/Dos) at the found ratio
		KsRmsAtOptimum_D3L(1,1) double = NaN   % Ks,rms delivered at the found ratio [A/m]
		ConstraintActiveD3L (1,1) logical = false % true if the Ks,rms,max constraint is binding

		SolvedD3L (1,1) logical = false
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
				options.WindingFactor       (1,1) double = 0.91
			end

			obj.spec_ = spec;
			obj.WindingFactor       = options.WindingFactor;
		end
	end

	% =====================================================================
	% Public methods
	% =====================================================================
	methods (Access = public)
		function solveD2L(obj)
			% SOLVE Runs the Esson's sizing estimate and stores results.
			%
			% Inputs:
			%   obj - (MotorSpec) The motor's expected configuration.
			s = obj.spec_;

			% Map MotorSpec -> EssonsEstimation inputs.
			T_Nm = s.Torque_Nm;
			P    = s.Poles;
			AspectRatio = s.AspectRatio;
			Airgap_m = s.Airgap_mm * 1e-3;


			[Dis, le, tau_p, Dro] = obj.compute_D2L(T_Nm, P, Airgap_m, AspectRatio);

			% --- property validation ---
			if ~(isfinite(Dis) && Dis > 0)
				error('EssonsSizer:invalidBore', ...
					'Computed stator bore (Dis) must be finite and > 0. Got: %g m', Dis);
			end
			if Airgap_m >= Dis/2
				error('EssonsSizer:invalidGeometry', ...
					['Airgap (g=%g m) must be much smaller than the computed stator bore/2 (Dis/2=%g m). ' ...
					 'Check MotorSpec.Airgap_mm and/or the Esson sizing inputs.'], ...
					Airgap_m, Dis/2);
			end
			if ~(isfinite(Dro) && Dro > 0)
				error('EssonsSizer:invalidRotorDiameter', ...
					'Computed rotor outer diameter (Dro) must be finite and > 0. Got: %g m', Dro);
			end
			obj.Dis_m    = Dis;
			obj.le_m     = le;
			obj.tau_p_m  = tau_p;
			obj.Dro_m    = Dro;
			obj.Solved   = true;
		end

		function solveD3L(obj, Dos_target_m, options)
			% SOLVED3L  D^3L sizing: fix Dos, find the Lagrangian-optimal
			% Dis subject to a Ks,rms,max cooling constraint, then solve
			% for the le that hits the torque target.
			%	Inputs:
			%   obj - (EssonsSizer) The Esson sizing object.
			%   Dos_target_m - (double) The target rotor outer diameter [m].
			% Reference: Lecture 6, "Improving the Esson's rule" --
			%   fo(Dis/Dos) = a*(Dis/Dos)^3 - 2b*(Dis/Dos)^2 + Dis/Dos
			%   Ks,rms = (kcu*Js,rms/4)*(a*Dis - 2b*Dos + Dos^2/Dis)
			%   Pout/wr[rpm] = (sqrt(2)*pi^2/480)*k1*kcu*ηgap*cos(φgap)*
			%                  fo(Dis/Dos)*Dos^3*le*Bg1*Js,rms
			%   (converted below to torque directly: see derivation notes)
			arguments
				obj
				Dos_target_m (1,1) double {mustBePositive}
				options.MakePlot (1,1) logical = true
			end

			s = obj.spec_;
			P    = s.Poles;
			Q    = s.Slots;
			Bg1  = s.Bg1_T;
			Bt   = s.Bt_T;
			Bc   = s.Bc_T;
			kis  = s.IronFillFactor;
			stackingfactor = s.StackingFactor;
			kcu  = s.CopperFillFactor;
			k1   = obj.WindingFactor;
			eta_estimation = s.EfficiencyEstimate;
			cos_phi = s.PowerFactor;
			Js_SI = s.CurrentDensity_Amm2 * 1e6;      % [A/m^2]
			KsRmsMax = s.LinearCurrentDensity_Am;     % [A/m], RMS
			T_Nm = s.Torque_Nm;
			Dos = Dos_target_m;

			[a, b, beta_t, beta_c] = EssonsSizer.computeAB_(Bg1, Bt, Bc, P, kis, stackingfactor);

			% ---- Unconstrained optimum: maximize fo(rho) over rho in (0,1) ----
			foNeg = @(rho) -EssonsSizer.fo_(rho, a, b);
			rho_u = fminbnd(foNeg, 1e-3, 1 - 1e-3);
			Dis_u = rho_u * Dos;
			Ks_u  = EssonsSizer.ksRms_(Dis_u, Dos, a, b, kcu, Js_SI);

			if Ks_u <= KsRmsMax
				% Constraint inactive: the geometry-optimal ratio is already
				% within the cooling limit.
				Dis = Dis_u;
				constraintActive = false;
			else
				% Constraint active: Ks,rms(Dis,Dos) is monotonically
				% decreasing in Dis (for Dis<Dos), so the feasible/binding
				% Dis lies between the unconstrained optimum and Dos.
				fun = @(Dis) EssonsSizer.ksRms_(Dis, Dos, a, b, kcu, Js_SI) - KsRmsMax;
				Dis = EssonsSizer.find_root_bracketed_(fun, Dis_u, Dos*(1 - 1e-6));
				constraintActive = true;
			end

			if ~(isfinite(Dis) && Dis > 0 && Dis < Dos)
				error('EssonsSizer:invalidD3LBore', ...
					'solveD3L failed to find a valid Dis in (0, Dos=%g m). Got: %g m', Dos, Dis);
			end

			ratio = Dis / Dos;
			fo_val = EssonsSizer.fo_(ratio, a, b);
			Ks_val = EssonsSizer.ksRms_(Dis, Dos, a, b, kcu, Js_SI);

			if fo_val <= 0
				error('EssonsSizer:invalidD3LOutputFn', ...
					['fo(Dis/Dos)=%.4g is not positive at the found ratio (%.4f). ' ...
					 'The torque equation cannot be solved for a positive le. ' ...
					 'Check Bt_T/Bc_T/Bg1_T or the requested Dos.'], fo_val, ratio);
			end

			% Torque form of the lecture's Pout/wr[rpm] = (...) equation
			% (dividing out the rpm->rad/s conversion, same manipulation
			% used to go from Esson's boxed P_mecc equation to its tau
			% form): tau = (sqrt(2)*pi/16)*k1*kcu*eta*cosphi*fo(rho)*
			%              Dos^3*le*Bg1*Js,rms
			k_out = (sqrt(2)*pi/16) * k1 * kcu * eta_estimation * cos_phi * fo_val * Bg1 * Js_SI;
			le = T_Nm / (k_out * Dos^3);

			tau_p = pi * Dis / P;

			obj.Dis_D3L_m          = Dis;
			obj.le_D3L_m           = le;
			obj.Dos_D3L_m          = Dos;
			obj.tau_p_D3L_m        = tau_p;
			obj.Ratio_D3L          = ratio;
			obj.AspectRatio_D3L    = le / tau_p;
			obj.FoAtOptimum_D3L    = fo_val;
			obj.KsRmsAtOptimum_D3L = Ks_val;
			obj.ConstraintActiveD3L = constraintActive;
			obj.SolvedD3L          = true;

			if options.MakePlot
				obj.plotD3LOptimum_(a, b, beta_t, beta_c, rho_u, Dis, Dos, P, Q, constraintActive);
			end
		end

		function summary(obj)
			% summary  Print a formatted overview of the Esson estimate.
			obj.requireSolved_('summary');

			s = obj.spec_;
			fprintf('\nEssonsSizer D^2L results\n');
			fprintf('%s\n', repmat('-', 1, 52));
			fprintf('  %-30s %.2f mm\n', 'Dis:',    obj.Dis_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'Dro:',    obj.Dro_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'le:',     obj.le_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'tau_p:',  obj.tau_p_m*1e3);
			fprintf('  %-30s %.2f\n', 'lambda (Aspect ratio):',  obj.le_m/obj.tau_p_m);
			fprintf('%s\n\n', repmat('-', 1, 52));
		end

		function summaryD3L(obj)
			% summaryD3L  Print a formatted overview of the D^3L estimate.
			if ~obj.SolvedD3L
				error('EssonsSizer:notSolved', ...
					'EssonsSizer.summaryD3L() called before solveD3L().');
			end

			s = obj.spec_;
			fprintf('\nEssonsSizer D^3L results (fixed Dos = %.2f mm)\n', obj.Dos_D3L_m*1e3);
			fprintf('%s\n', repmat('-', 1, 52));
			fprintf('  %-30s %g Nm\n', 'Torque target:', s.Torque_Nm);
			fprintf('  %-30s %d\n',   'Poles (P):',     s.Poles);
			fprintf('\n');
			fprintf('  %-30s %.2f mm\n', 'Dis (optimal):', obj.Dis_D3L_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'Dos (fixed input):', obj.Dos_D3L_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'le (solved for torque):', obj.le_D3L_m*1e3);
			fprintf('  %-30s %.2f mm\n', 'tau_p:', obj.tau_p_D3L_m*1e3);
			fprintf('  %-30s %.4f\n',    'Dis/Dos:', obj.Ratio_D3L);
			fprintf('  %-30s %.4f  (typical 1.0-2.0)\n', 'le/tau_p:', obj.AspectRatio_D3L);
			fprintf('  %-30s %.4f\n',    'fo(Dis/Dos):', obj.FoAtOptimum_D3L);
			fprintf('  %-30s %.0f A/m\n', 'Ks,rms delivered:', obj.KsRmsAtOptimum_D3L);
			fprintf('  %-30s %s\n', 'Ks,rms,max constraint:', ...
				string(obj.ConstraintActiveD3L));
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
			s.results.Dro_m    = obj.Dro_m;
			s.results.Solved   = obj.Solved;

			if obj.SolvedD3L
				s.resultsD3L = struct();
				s.resultsD3L.Dis_m               = obj.Dis_D3L_m;
				s.resultsD3L.le_m                = obj.le_D3L_m;
				s.resultsD3L.Dos_m               = obj.Dos_D3L_m;
				s.resultsD3L.tau_p_m             = obj.tau_p_D3L_m;
				s.resultsD3L.Ratio               = obj.Ratio_D3L;
				s.resultsD3L.AspectRatio         = obj.AspectRatio_D3L;
				s.resultsD3L.FoAtOptimum         = obj.FoAtOptimum_D3L;
				s.resultsD3L.KsRmsAtOptimum      = obj.KsRmsAtOptimum_D3L;
				s.resultsD3L.ConstraintActive    = obj.ConstraintActiveD3L;
			end
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
	% Private - computation
	% =====================================================================
	methods (Access = private)
		function [Dis, le, tau_p, Dro] = compute_D2L(obj, T_Nm, P, Airgap_m, AspectRatio)
			sigma_m = 70e3; % [N/m^2] (Shear stress)
			k = AspectRatio * pi / P; % [-] (Mechanical ratio)
			% ---------- Calculation inner diameter of stator ---------------
			Dis = (2 * T_Nm / (pi* sigma_m * k ))^(1/3);
			le = k * Dis; % [m] (Effective length)
			tau_p = pi * Dis / P;
			Dro = Dis - 2 * Airgap_m;
		end

		function plotD3LOptimum_(obj, a, b, beta_t, beta_c, rho_u, Dis, Dos, P, Q, constraintActive)
			% plotD3LOptimum_  Visualize the D^3L output function fo(rho)
			% and the resulting stator cross-section at the solved point
			% -- ported from plot_D3L_geometry.m so solveD3L() shows the
			% same picture on every call instead of needing a separate
			% script. Called from solveD3L() unless MakePlot=false.
			rho = linspace(0.01, 0.99, 400);
			fo = EssonsSizer.fo_(rho, a, b);
			fo_u = EssonsSizer.fo_(rho_u, a, b);
			ratio = Dis / Dos;
			fo_ratio = EssonsSizer.fo_(ratio, a, b);

			fig = figure('Name', sprintf('D^3L optimum at Dos=%.1fmm', Dos*1e3), ...
				'Position', [100 100 1100 480]);

			% ---------------- Left panel: fo(rho) ----------------
			subplot(1,2,1);
			hold on; grid on;
			plot(rho, fo, 'LineWidth', 1.8, 'Color', [0 0.45 0.74], 'DisplayName', 'f_o(\rho)');
			plot(rho_u, fo_u, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0 0.45 0.74], ...
				'MarkerEdgeColor', 'k', 'DisplayName', 'unconstrained optimum');
			if constraintActive
				plot(ratio, fo_ratio, 's', 'MarkerSize', 8, 'MarkerFaceColor', [0.85 0.1 0.1], ...
					'MarkerEdgeColor', 'k', 'DisplayName', 'K_{s,rms,max}-bound solution');
				legend('Location', 'best');
			end
			xlabel('\rho = D_{is}/D_{os}');
			ylabel('f_o(\rho)');
			title({sprintf('Output function f_o(\\rho), P=%d/Q=%d', P, Q), ...
				sprintf('D_{os}=%.1fmm, solved \\rho=%.3f', Dos*1e3, ratio)});
			ylim([-0.1, 0.5]);

			% ---------------- Right panel: annotated cross-section --------------
			subplot(1,2,2);
			d_cs = Dis * beta_c / 2;
			R_os = Dos/2; R_is = Dis/2; R_yoke = R_os - d_cs;

			hold on; axis equal off;
			th = linspace(0, 2*pi, 200);

			% Draw back-to-front, same layering as plot_D3L_geometry.m:
			% solid iron disk -> orange slot-region hole -> teeth wedges
			% painted back over the orange -> bore/rotor cavity on top.
			fill(R_os*cos(th), R_os*sin(th), [0.55 0.55 0.58], 'EdgeColor', 'k');
			fill(R_yoke*cos(th), R_yoke*sin(th), [0.85 0.5 0.15], 'EdgeColor', 'none');

			slot_pitch_ang = 2*pi/Q;
			tooth_ang = beta_t * slot_pitch_ang;
			for k = 0:Q-1
				a0 = k*slot_pitch_ang - tooth_ang/2;
				a1 = a0 + tooth_ang;
				aw = linspace(a0, a1, 10);
				xw = [R_is*cos(aw), R_yoke*cos(fliplr(aw))];
				yw = [R_is*sin(aw), R_yoke*sin(fliplr(aw))];
				fill(xw, yw, [0.55 0.55 0.58], 'EdgeColor', 'k', 'LineWidth', 0.5);
			end
			fill(R_is*cos(th), R_is*sin(th), [0.75 0.85 0.95], 'EdgeColor', 'k');

			title({sprintf('Cross-section at \\rho=%.3f (P=%d, D_{os}=%.0fmm)', ratio, P, Dos*1e3), ...
				sprintf('\\beta_t=%.3f (tooth/slot-pitch), \\beta_c=%.3f (2\\cdotd_{cs}/D_{is})', beta_t, beta_c)});

			text(0, 0, sprintf('bore\nD_{is}=%.1fmm', Dis*1e3), 'HorizontalAlignment', 'center', 'FontSize', 8);
			text(R_os*0.75, R_os*0.75, sprintf('yoke\nd_{cs}=%.1fmm', d_cs*1e3), 'FontSize', 8, 'HorizontalAlignment', 'center');
			text(-R_os*0.9, -R_os*0.55, sprintf('teeth (gray)\nslots (orange)'), 'FontSize', 8);
			xlim([-R_os*1.15, R_os*1.15]); ylim([-R_os*1.15, R_os*1.15]);

			sgtitle(sprintf('D^3L optimum: P=%d/Q=%d, D_{os}=%.1fmm \\rightarrow D_{is}=%.1fmm, l_e=%.1fmm', ...
				P, Q, Dos*1e3, Dis*1e3, obj.le_D3L_m*1e3));
			drawnow;
		end

		function requireSolved_(obj, caller)
			if ~obj.Solved
				error('EssonsSizer:notSolved', ...
					'EssonsSizer.%s() called before solveD2L().', caller);
			end
		end
	end

	% =====================================================================
	% Public static helpers
	% =====================================================================
	methods (Static)
		function [a, b, beta_t, beta_c] = computeAB_(Bg1, Bt, Bc, P, kis, stackingfactor)
			% computeAB_  D^3L output-function coefficients a, b: depend
			% only on flux-density targets and pole count, not on slot
			% count or bore size.
			%   beta_t = ts/tau_s, the fraction of one slot pitch taken up
			%            by a tooth driven to its target flux density Bt.
			%   beta_c = 2*d_cs/Dis, the yoke depth as a fraction of the
			%            bore RADIUS, driven to its target flux density Bc.
			beta_t = Bg1/(kis*Bt) * stackingfactor;
			beta_c = (2/P)*Bg1/(kis*Bc) * stackingfactor;
			a = (beta_t + beta_c)^2 - (1 - beta_t)^2;
			b = beta_t + beta_c;
		end

		function fo = fo_(rho, a, b)
			% fo_  D^3L output function fo(Dis/Dos) = a*rho^3-2b*rho^2+rho.
			fo = a.*rho.^3 - 2*b.*rho.^2 + rho;
		end

		function Ks = ksRms_(Din, Dout, a, b, kcu, Js_SI)
			% ksRms_  RMS surface current density delivered by a slot
			% geometry sized for (Din, Dout), per Lecture 6:
			%   Ks,rms = (kcu*Js,rms/4)*(a*Din - 2b*Dout + Dout^2/Din)
			Ks = (kcu*Js_SI/4) .* (a.*Din - 2*b.*Dout + Dout.^2./Din);
		end
	end

	% =====================================================================
	% Private static helpers (ported from EssonsEstimation.m)
	% =====================================================================
	methods (Static, Access = private)
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

