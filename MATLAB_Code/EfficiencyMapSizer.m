classdef EfficiencyMapSizer < handle
   % EfficiencyMapSizer  Rough analytical loss and efficiency-map estimate
   % over the motor's torque-speed operating envelope, built entirely from
   % a converged IPMRotorSizer -- no extra FEM runs needed. Produces the
   % "energy efficiency map over the whole torque-speed range" deliverable
   % the assignment brief asks for (Part B).
   %
   % Deliberately "rough" (the assignment's own language for this design
   % stage), with every simplifying assumption flagged here and in
   % summary()/the design report:
   %   - Copper loss: phase resistance from a geometric mean-turn-length
   %     estimate (2x stack length + a semicircular end-turn bridging one
   %     slot pitch on each end -- appropriate for this design's
   %     non-overlapping, single-tooth-wound coils), evaluated at an
   %     assumed hot winding temperature (CuOperatingTemp_C).
   %   - Core loss: a two-term (hysteresis + eddy) Steinmetz fit
   %     calibrated to M235-35A's single datasheet point (P_1.5/50 =
   %     2.35 W/kg -- literally what "235" means in the EN 10106 grade
   %     name), split by an assumed hyst/eddy ratio at 50Hz and
   %     extrapolated up to this design's much higher electrical
   %     frequency. That extrapolation is large (see summary()) and is
   %     this map's single biggest source of uncertainty.
   %   - Mechanical (windage/bearing) losses: excluded -- no validated
   %     model available at this design stage.
   %   - Flux-weakening above corner speed: simplified steady-state
   %     voltage-limit solve (Id from V_ph = w_e*|Psi_pm + j*Ld*Id +
   %     ... | = V_ph,max at fixed Iq), neglecting the resistive volt
   %     drop -- standard first-pass approximation.
   %
   % Usage:
   %   em = EfficiencyMapSizer(spec, rotorSizer, materials);
   %   em.solve();
   %   em.summary();
   %   em.plotMap(SavePath=fullfile('..','IPM_Design_Report','figures','efficiency_map.png'));

   properties (SetAccess = private)
      PhaseResistance_Ohm  (1,1) double = NaN   % per phase, at CuOperatingTemp_C
      MassTeeth_kg         (1,1) double = NaN
      MassYoke_kg          (1,1) double = NaN

      % --- Rated (corner) operating point, for sanity-checking against FEM ---
      RatedCopperLoss_W    (1,1) double = NaN
      RatedCoreLoss_W      (1,1) double = NaN
      RatedEfficiency_pct  (1,1) double = NaN

      % --- Map data ---
      Speed_rpm         (1,:) double = []   % [1 x nSpeed]
      Torque_Nm         (:,1) double = []   % [nTorque x 1]
      Efficiency_pct    double = []         % [nTorque x nSpeed], NaN outside the achievable envelope
      EnvelopeTorque_Nm (1,:) double = []   % [1 x nSpeed] max achievable continuous torque

      Solved (1,1) logical = false
   end

   properties
      CuOperatingTemp_C (1,1) double = 100    % [degC] assumed hot winding temperature
      CuTempCoeff_perC  (1,1) double = 0.00393 % [1/degC] copper resistivity temp. coefficient (from 20degC)
   end

   properties (Access = private)
      spec_
      rotor_
      mat_
      coreLossPerKg_   % @(f_Hz, B_T) -> W/kg, cached Steinmetz fit
   end

   methods
      function obj = EfficiencyMapSizer(spec, rotorSizer, materials)
         arguments
            spec        MotorSpec
            rotorSizer  IPMRotorSizer
            materials   MotorMaterials = MotorMaterials()
         end
         if ~rotorSizer.Converged
            error('EfficiencyMapSizer:notConverged', ...
                  'rotorSizer must be solve()d and converged before building an efficiency map.');
         end
         obj.spec_   = spec;
         obj.rotor_  = rotorSizer;
         obj.mat_    = materials;
      end

      function solve(obj, options)
         %solve  Compute phase resistance + core-loss model, then sweep
         %the torque-speed grid. SpeedPoints/TorquePoints control grid
         %resolution (this is all closed-form/root-finding, so even a
         %fine grid solves in well under a second).
         arguments
            obj
            options.SpeedPoints  (1,1) double {mustBeInteger, mustBePositive} = 60
            options.TorquePoints (1,1) double {mustBeInteger, mustBePositive} = 60
         end
         obj.computeResistance_();
         obj.computeCoreLossModel_();
         obj.computeMap_(options.SpeedPoints, options.TorquePoints);
         obj.computeRatedPoint_();
         obj.Solved = true;
      end

      function summary(obj)
         if ~obj.Solved
            error('EfficiencyMapSizer:notSolved', 'Call solve() first.');
         end
         s = obj.spec_;
         fprintf('\nEfficiencyMapSizer summary (rough analytical map, not FEM)\n');
         fprintf('%s\n', repmat('-', 1, 60));
         fprintf('  Phase resistance (@%.0f degC):     %8.2f mOhm\n', obj.CuOperatingTemp_C, obj.PhaseResistance_Ohm*1e3);
         fprintf('  %-34s %8.3f kg\n',   'Stator core mass (teeth+yoke):', obj.MassTeeth_kg + obj.MassYoke_kg);
         fprintf('  --- Rated point (%.3g Nm @ %.0f rpm corner) ---\n', s.Torque_Nm, s.CornerSpeed_rpm);
         fprintf('  %-34s %8.2f W\n', 'Copper loss:', obj.RatedCopperLoss_W);
         fprintf('  %-34s %8.2f W\n', 'Core loss:',   obj.RatedCoreLoss_W);
         fprintf('  %-34s %8.2f %%\n','Efficiency:',  obj.RatedEfficiency_pct);
         fprintf('  Note: core loss extrapolates a 50Hz datasheet point out to f_c=%.0fHz\n', s.CornerFrequency_Hz);
         fprintf('  (%.0fx) -- treat as an order-of-magnitude estimate, not a precise figure.\n', s.CornerFrequency_Hz/50);
         fprintf('%s\n\n', repmat('-', 1, 60));
      end

      function fig = plotMap(obj, options)
         %plotMap  Efficiency contour over the torque-speed grid, with
         %the achievable envelope, the rated design point, and the
         %assignment's peak-torque/max-speed targets (Table 1) marked
         %for context. Saves a PNG if SavePath is given.
         arguments
            obj
            options.SavePath (1,:) char = ''
         end
         if ~obj.Solved
            error('EfficiencyMapSizer:notSolved', 'Call solve() first.');
         end
         s = obj.spec_;

         fig = figure('Name', 'Efficiency map', 'Position', [100 100 760 560]);
         hold on;
         levels = 40:2:98;
         contourf(obj.Speed_rpm, obj.Torque_Nm, obj.Efficiency_pct, levels, 'LineColor', 'none');
         cb = colorbar; cb.Label.String = 'Efficiency [%]';
         colormap(turbo(numel(levels)));
         clim([levels(1) levels(end)]);

         plot(obj.Speed_rpm, obj.EnvelopeTorque_Nm, 'k-', 'LineWidth', 2);
         plot(s.CornerSpeed_rpm, s.Torque_Nm, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y');
         text(s.CornerSpeed_rpm, s.Torque_Nm, '  rated point', 'FontSize', 9, 'FontWeight', 'bold');

         % Assignment Table 1 targets, for context only (not evaluated here):
         % peak torque is a 5s overload rating, max speed exceeds this
         % design's inverter-limited flux-weakening range (see report,
         % "Inverter" section).
         yline(21.5, 'r--', 'peak torque, 5s (spec)', 'LabelHorizontalAlignment', 'left');
         xline(20000, 'b--', 'spec max speed', 'LabelVerticalAlignment', 'bottom');

         xlabel('Speed [rpm]');
         ylabel('Torque [Nm]');
         title('Analytical efficiency map (I^2R + core loss; FEM validates torque at the rated point only)');
         xlim([0, max(obj.Speed_rpm(end), 20500)]);
         ylim([0, max(obj.Torque_Nm(end), 22)]);
         box on; grid on;

         if ~isempty(options.SavePath)
            saveDir = fileparts(options.SavePath);
            if ~isempty(saveDir) && ~exist(saveDir, 'dir')
               mkdir(saveDir);
            end
            exportgraphics(fig, options.SavePath, 'Resolution', 150);
            fprintf('[EfficiencyMapSizer] Saved %s\n', options.SavePath);
         end
      end
   end

   methods (Access = private)
      function computeResistance_(obj)
         r = obj.rotor_;
         Uc      = r.ConductorsInSeries;
         n_w     = r.NumStrands;
         A_strand = pi/4 * (r.WireDiameter_mm*1e-3)^2;
         tau_s_m  = pi * r.StatorBore_mm*1e-3 / obj.spec_.Slots;
         L_m      = r.StackLength_mm*1e-3;

         % Mean turn length: two straight passes through the stack, plus
         % a semicircular end-turn loop on each end sized to just bridge
         % one slot pitch -- a reasonable first-order estimate for this
         % design's non-overlapping, single-tooth-wound coils (q=0.4 spp,
         % see Part A "Pole/Slot Number"), which have much shorter end
         % turns than an overlapping/distributed winding.
         l_turn_m = 2*L_m + pi*tau_s_m;

         rho20 = 1/obj.mat_.sigma_copper;
         rhoT  = rho20 * (1 + obj.CuTempCoeff_perC*(obj.CuOperatingTemp_C - 20));
         obj.PhaseResistance_Ohm = rhoT * Uc * l_turn_m / (n_w * A_strand);
      end

      function computeCoreLossModel_(obj)
         r = obj.rotor_;
         s = obj.spec_;
         k_st = s.StackingFactor;
         L_m = r.StackLength_mm*1e-3;

         OD_m       = r.StatorOD_mm*1e-3;
         ID_yoke_m  = OD_m - 2*r.StatorYokeHeight_mm*1e-3;
         V_yoke     = pi/4*(OD_m^2 - ID_yoke_m^2)*L_m*k_st;
         h_tooth_m  = (r.SlotHeight_mm + r.Has_mm)*1e-3;
         V_teeth    = s.Slots * (r.ToothWidth_mm*1e-3) * h_tooth_m * L_m * k_st;

         obj.MassYoke_kg  = V_yoke  * obj.mat_.rho_lam;
         obj.MassTeeth_kg = V_teeth * obj.mat_.rho_lam;

         P_ref = obj.mat_.CoreLoss_Wkg_1p5T_50Hz;
         f_ref = 50; B_ref = 1.5;
         hFrac = obj.mat_.CoreLossHystFrac_50Hz;
         kh = hFrac      * P_ref / (f_ref   * B_ref^2);
         ke = (1 - hFrac)* P_ref / (f_ref^2 * B_ref^2);
         obj.coreLossPerKg_ = @(f, B) kh.*f.*B.^2 + ke.*f.^2.*B.^2;
      end

      function [Id, Iq, feasible] = solveOperatingPoint_(obj, T_Nm, n_rpm)
         %solveOperatingPoint_  Id/Iq (peak, dq) for a target torque at a
         %given speed.
         %
         % Below/at corner speed, this forces Id=0 unconditionally --
         % matching IPMRotorSizer.computeElectricalParameters_'s own
         % "no field weakening at the base torque point" assumption
         % (the same Id_A/Iq_A that actually get pushed into the FEM
         % model as Ipk). This map does NOT re-litigate whether that
         % point's terminal voltage is actually reachable: the converged
         % PsiPM1 is well below the target flux linkage (a known,
         % separately-documented gap -- see README "Known issues", and
         % the design report's "Corner Speed and Required Flux Linkage"
         % section), which on its own already makes a naive voltage
         % check fail right at the design's own declared corner point.
         % Applying this map's voltage-limit model there would just
         % substitute one large, non-physical Id for a problem this map
         % isn't the place to resolve.
         %
         % Above corner speed (genuine flux-weakening territory), Id is
         % solved from the voltage limit at fixed Iq: V_ph = w_e*|Psi_pm
         % + j*Ld*Id + ...| = V_ph,max (resistive drop neglected).
         % feasible=false if even Id can't bring the voltage under the
         % limit -- the Lq*Iq cross term alone already exceeds it, so
         % this (T,n) point is outside the achievable envelope.
         s = obj.spec_; r = obj.rotor_;
         p = s.Poles;
         PsiPM1 = r.PsiPM1_Wb;
         Ld = r.Ld_mH*1e-3; Lq = r.Lq_mH*1e-3;
         Vph_max = 0.95*s.VdcLink_V/(2*sqrt(2));   % same convention as computeStatorWindingSizing_'s V_invM

         Iq = T_Nm / (1.5*(p/2)*PsiPM1);

         if n_rpm <= s.CornerSpeed_rpm
            Id = 0; feasible = true;
            return;
         end

         we = 2*pi*(p/2)*n_rpm/60;
         rhs = (Vph_max/we)^2 - (Lq*Iq)^2;
         if rhs < 0
            Id = NaN; feasible = false;   % voltage limit unreachable at this (T,n)
            return;
         end
         Id = (sqrt(rhs) - PsiPM1) / Ld;
         Id = min(Id, 0);   % field-weakening only ever reduces Id, never adds to it
         feasible = true;
      end

      function computeMap_(obj, nSpeed, nTorque)
         s = obj.spec_; r = obj.rotor_;
         PsiPM1 = r.PsiPM1_Wb; Ld = r.Ld_mH*1e-3;
         p = s.Poles;

         speeds  = linspace(0, s.MaxSpeed_rpm, nSpeed);
         torques = linspace(0, s.Torque_Nm, nTorque)';

         eta   = NaN(nTorque, nSpeed);
         envT  = zeros(1, nSpeed);

         for i = 1:nSpeed
            n = speeds(i);
            f_e = (p/2)*n/60;
            for j = 1:nTorque
               T = torques(j);
               [Id, Iq, feasible] = obj.solveOperatingPoint_(T, n);
               if ~feasible
                  continue;   % leave NaN -- outside the achievable envelope
               end
               envT(i) = T;   % highest feasible T so far in this column (torques is ascending)

               if T <= 0
                  continue;   % P_mech=0 -> efficiency undefined
               end

               I_pk  = hypot(Id, Iq);
               I_rms = I_pk/sqrt(2);
               P_cu  = 3 * I_rms^2 * obj.PhaseResistance_Ohm;

               Bscale = max((PsiPM1 + Ld*Id)/PsiPM1, 0.05);   % flux-weakening reduces core flux too
               P_fe = obj.coreLossPerKg_(f_e, s.Bt_T*Bscale)*obj.MassTeeth_kg ...
                    + obj.coreLossPerKg_(f_e, s.Bc_T*Bscale)*obj.MassYoke_kg;

               P_mech = T * (2*pi*n/60);
               P_elec = P_mech + P_cu + P_fe;
               eta(j,i) = 100 * P_mech/P_elec;
            end
         end

         obj.Speed_rpm = speeds;
         obj.Torque_Nm = torques;
         obj.Efficiency_pct = eta;
         obj.EnvelopeTorque_Nm = envT;
      end

      function computeRatedPoint_(obj)
         s = obj.spec_; r = obj.rotor_;
         PsiPM1 = r.PsiPM1_Wb; Ld = r.Ld_mH*1e-3;
         [Id, Iq] = obj.solveOperatingPoint_(s.Torque_Nm, s.CornerSpeed_rpm);
         I_rms = hypot(Id, Iq)/sqrt(2);
         obj.RatedCopperLoss_W = 3 * I_rms^2 * obj.PhaseResistance_Ohm;

         f_c = s.CornerFrequency_Hz;
         Bscale = max((PsiPM1 + Ld*Id)/PsiPM1, 0.05);
         obj.RatedCoreLoss_W = obj.coreLossPerKg_(f_c, s.Bt_T*Bscale)*obj.MassTeeth_kg ...
                              + obj.coreLossPerKg_(f_c, s.Bc_T*Bscale)*obj.MassYoke_kg;

         P_mech = s.Torque_Nm * (2*pi*s.CornerSpeed_rpm/60);
         obj.RatedEfficiency_pct = 100 * P_mech / (P_mech + obj.RatedCopperLoss_W + obj.RatedCoreLoss_W);
      end
   end
end
