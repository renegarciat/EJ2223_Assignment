classdef IPMRotorSizer < handle
% IPMRotorSizer  Rotor sizing for V-shape IPM motors.
%
%   Implements the analytical design procedure of:
%     A. Di Gerlando, C. Ricca, "Design Modeling and Sizing Equations of
%     V-shape IPM Motors," ICEM 2022, DOI: 10.1109/ICEM51905.2022.9910924
%
%   Equation numbers in comments refer directly to the paper.
%   Sections I-VIII of the paper map onto the private compute methods.
%
%   Usage
%   -----
%     spec  = MotorSpec(200, 2900, 13500, 650, 8, 60, 1, 1.37);
%     sizer = IPMRotorSizer(spec);
%     sizer.solve();          % run until convergence
%     sizer.summary();        % print results vs. paper Table I values
%
%   Rotor design choices (IPMRotorSizer properties, not MotorSpec)
%   --------------------------------------------------------------
%     AlphaM          Magnet embrace / pole-arc ratio      (default 0.754)
%     Whr_fraction    w_hr = Whr_fraction * tau_s          (default 0.55)
%     Wob_mm          Depth rotor OD → pocket top          (default 0.5 mm)
%     Hm_mm           PM segment height                    (default 6 mm)
%     Vtilt_deg       Magnet tilt angle v                  (default 78 deg)
%     Wib_mm          Inner bridge / magnet spacing        (default 2.5 mm)
%     HryFraction     h_ry = HryFraction * w_hr            (default 1.5)
%     WindingFactor   k_w  (2-layer winding factor)        (default 0.91)
%     Has_mm          h_as, slot opening (wedge) height    (default 1.5 mm)
%     WireClearance_mm d_clearance, wire insertion margin  (default 0.3 mm)
%     RhoEV           rho_EV, EMF/max-voltage utilization  (default 0.650)
%
%   Iterative parameters (updated by solve(), seeded from MotorSpec)
%   ----------------------------------------------------------------
%     RhoBtS          rho_bt_s  = b_ts / tau_s             (default 0.704)
%     RhoHteG         rho_hte_g = h_te / g                 (default 52.4)
%     SigmaAnis       sigma_an,o (anisotropy ratio)        (default 4.11)
%     Cd              c_d (d-axis reaction coefficient)     (default 0.201)
%
%   Read-only results (set by solve())
%   -----------------------------------
%   Rotor geometry:
%     RotorOD_mm, RotorID_mm, RotorPolePitch_mm
%     Bps_mm, Hob_mm, Zeta_deg, Hib_mm, Hhr_mm
%     D12_mm, D23_mm, D24_mm, Dps_mm
%     Bm_mm, Whr_mm, Hry_mm
%   Saturation model:
%     CarterFactor, PhiGo_Wbm, Bg1o_T, PhiG1o_Wbm
%   Torque sizing:
%     StackLength_mm, GammaOpt_deg, SpecificTorque_kNmm
%     EtaPhi_c, SigmaS_c
%   Winding data & stator core dimensions (paper Section VI, eqs. 65-94):
%     ConductorsInSeries, ConductorsInSlot, PhaseCurrent_A, NumStrands
%     WireDiameter_mm, ToothWidth_mm, SlotWidthInner_mm, SlotWidthOuter_mm
%     SlotHeight_mm, SlotArea_mm2
%     StatorYokeHeight_mm, StatorOD_mm
%   Electrical parameters:
%     Cq, Ld_mH, Lq_mH, PsiPM1_Wb, Id, Iq
%     LambdaIs_uHm
%   Rotor bridge stress check (paper Section IV, eqs. 19-22):
%     FMaxSpecific_Nm, SigmaIbIdeal_MPa, SigmaIb_MPa, SigmaIbRatio, BridgeSafe
%   Convergence:
%     Converged, Iterations
%
%   See also: MotorSpec, EssonSizer, MotorGeometry, MotorMaterials

    % =====================================================================
    % Public — rotor design choices (not in MotorSpec)
    % =====================================================================
    properties

        % Rotor geometry choices
        AlphaM          (1,1) double {mustBeInRange(AlphaM,    0.5, 0.95)} = 0.754
        Whr_fraction    (1,1) double {mustBeInRange(Whr_fraction, 0.1, 0.8)} = 0.55
        Wob_mm          (1,1) double {mustBePositive}  = 0.5
        Hm_mm           (1,1) double {mustBePositive}  = 6.0
        Vtilt_deg       (1,1) double {mustBeInRange(Vtilt_deg, 45, 89)} = 78
        Wib_mm          (1,1) double {mustBePositive}  = 2.5
        HryFraction     (1,1) double {mustBePositive}  = 1.5

        % Winding
        WindingFactor   (1,1) double {mustBeInRange(WindingFactor, 0.5, 1.0)} = 0.91

        % Stator slot / winding design choices (paper Section VI, eqs. 65-94)
        Has_mm          (1,1) double {mustBePositive}  = 1.5    % h_as, slot opening (wedge) height
        WireClearance_mm (1,1) double {mustBePositive} = 0.3    % d_clearance, wire insertion clearance
        RhoEV           (1,1) double {mustBeInRange(RhoEV, 0.3, 0.95)} = 0.650  % rho_EV, EMF/max-voltage utilization target

        % Iterative parameters — seeded here, updated by solve()
        RhoBtS          (1,1) double {mustBePositive}  = 0.704  % b_ts/tau_s
        RhoHteG         (1,1) double {mustBePositive}  = 52.4   % h_te/g
        SigmaAnis       (1,1) double {mustBePositive}  = 4.11   % anisotropy ratio
        Cd              (1,1) double {mustBePositive}  = 0.201  % d-axis reaction coeff

        % Solver settings
        MaxIterations   (1,1) double {mustBePositive, mustBeInteger} = 20
        Tolerance       (1,1) double {mustBePositive} = 1e-4

    end

    % =====================================================================
    % Public — stator bore actually used (from EssonsSizer, or overridden)
    % =====================================================================
    properties (SetAccess = private)
        StatorBore_mm     (1,1) double = NaN   % D — from EssonsSizer, or the constructor override
    end

    % =====================================================================
    % Public — magnet material (moved out of MotorSpec: the magnet grade
    % is a design decision that only matters once rotor sizing starts,
    % not at the initial torque/geometry-sizing stage MotorSpec covers)
    % =====================================================================
    properties (Dependent, SetAccess = private)
        Materials   % the MotorMaterials instance driving this sizer (bridge stress + PM remanence)
        Br_T        % PM remanence, temperature-corrected to spec.PMTemp_C (Materials.Br is the 20degC datasheet value)
    end

    % =====================================================================
    % Public — rotor bridge stress check results (eqs. 19-22)
    % =====================================================================
    properties (SetAccess = private)
        FMaxSpecific_Nm   (1,1) double = NaN   % f_max, per unit stack length [N/m]
        SigmaIbIdeal_MPa  (1,1) double = NaN   % sigma_ib.i (no concentration factor)
        SigmaIb_MPa       (1,1) double = NaN   % sigma_ib = Kt * sigma_ib.i
        SigmaIbRatio      (1,1) double = NaN   % sigma_ib / sigma_y_lam  (must be < 1)
        BridgeSafe        (1,1) logical = false
    end

    % =====================================================================
    % Read-only results — written exclusively by solve_()
    % =====================================================================
    properties (SetAccess = private)

        % --- Rotor geometry (eqs. 8–18) ---
        RotorOD_mm       (1,1) double = NaN   % D_r
        RotorID_mm       (1,1) double = NaN   % D_ir
        RotorPolePitch_mm (1,1) double = NaN  % tau_r
        Bps_mm           (1,1) double = NaN   % pole-shoe extension
        Hob_mm           (1,1) double = NaN   % outer bridge length
        Zeta_deg         (1,1) double = NaN   % side PM angle
        Hib_mm           (1,1) double = NaN   % inner bridge length
        Hhr_mm           (1,1) double = NaN   % half-rib radial length
        D12_mm           (1,1) double = NaN   % construction length d_12
        D23_mm           (1,1) double = NaN   % construction length d_23
        D24_mm           (1,1) double = NaN   % construction length d_24
        Dps_mm           (1,1) double = NaN   % pole-shoe radial depth
        Bm_mm            (1,1) double = NaN   % PM segment width
        Whr_mm           (1,1) double = NaN   % half-rib tooth width
        Hry_mm           (1,1) double = NaN   % rotor yoke height

        % --- Saturation model (eqs. 3–7, 38–42) ---
        CarterFactor     (1,1) double = NaN   % k_C
        PhiGo_Wbm        (1,1) double = NaN   % phi_go (no-load, per unit length)
        Bg1o_T           (1,1) double = NaN   % fundamental air-gap flux density
        PhiG1o_Wbm       (1,1) double = NaN   % no-load fundamental specific flux

        % --- Torque sizing (eqs. 57–64) ---
        StackLength_mm   (1,1) double = NaN   % ell
        GammaOpt_deg     (1,1) double = NaN   % optimal phase advance at corner
        SpecificTorque_kNmm (1,1) double = NaN % T_ell [kNm/m]
        EtaPhi_c         (1,1) double = NaN   % eta_phiM at corner point
        SigmaS_c         (1,1) double = NaN   % sigma_sM at corner point

        % --- Winding data & stator core dimensions (eqs. 65–94) ---
        ConductorsInSeries       (1,1) double = NaN   % U_c
        ConductorsInSlot         (1,1) double = NaN   % u
        PhaseCurrent_A           (1,1) double = NaN   % I_c [A rms]
        NumStrands               (1,1) double = NaN   % n_w
        WireDiameter_mm          (1,1) double = NaN   % d_wcu
        ToothWidth_mm            (1,1) double = NaN   % b_ts
        SlotWidthInner_mm        (1,1) double = NaN   % b_1 (minor, bore side)
        SlotWidthOuter_mm        (1,1) double = NaN   % b_2 (major, yoke side)
        SlotHeight_mm            (1,1) double = NaN   % h
        SlotArea_mm2             (1,1) double = NaN   % A_slot
        StatorYokeHeight_mm      (1,1) double = NaN   % h_sy
        StatorOD_mm              (1,1) double = NaN   % D_es

        % --- Electrical parameters (eqs. 95–120) ---
        Cq               (1,1) double = NaN   % q-axis reaction coefficient
        LambdaIs_uHm     (1,1) double = NaN   % specific permeance [µH/m]
        Ld_mH            (1,1) double = NaN   % d-axis synchronous inductance
        Lq_mH            (1,1) double = NaN   % q-axis synchronous inductance (corner)
        PsiPM1_Wb        (1,1) double = NaN   % PM flux linkage [Wb rms]
        Id_A             (1,1) double = NaN   % direct-axis current
        Iq_A             (1,1) double = NaN   % quadrature-axis current

        % --- Convergence ---
        Converged        (1,1) logical = false
        Iterations       (1,1) double  = 0

    end

    % =====================================================================
    % Private — reference to spec and cached BH curve interpolant
    % =====================================================================
    properties (Access = private)
        spec_           % MotorSpec handle
        materials_      % MotorMaterials handle (mechanical properties for bridge check)
        Hfe_interp_     % griddedInterpolant for M235-35A BH curve

        % Cached derived geometry (computed from EssonsSizer at solve() start,
        % unless statorBoreOverride_mm_ is set — see constructor StatorBore_mm)
        statorBore_mm_          (1,1) double = NaN
        statorBoreOverride_mm_  (1,1) double = NaN
    end

    % =====================================================================
    % Constants
    % =====================================================================
    properties (Constant, Access = private)
        MU0 = 4*pi*1e-7   % [H/m]

        % M235-35A BH curve — derived from the paper's own Fig. 2
        % (references/fig_2_ricca.png), not a generic manufacturer
        % datasheet. The paper only plots relative permeability
        % mu_fe,pu(B) = mu_fe/mu0 = B/(mu0*H_fe) (dimensionless, log
        % y-axis), not H_fe in A/m. We pixel-traced (B, mu_fe,pu) pairs
        % off the screenshot (calibrated against the plot's axis tick
        % labels; see MATLAB_Code/plot_bh_curve.m for the 1:1 comparison
        % plot), then inverted point-by-point:
        %   H_fe = B / (mu0 * mu_fe,pu)
        % to get HFE_DATA below in the A/m units this class's Hfe(B)
        % interpolant needs. B=0 forced to H=0 as the physical anchor.
        % Cross-check: at B=3.0T, H_fe=684000 A/m back-converts to
        % mu_fe,pu = 3.0/(mu0*684000) = 3.49, matching the paper's
        % traced curve endpoint.
        BFE_DATA = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, ...
                    1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, ...
                    2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0]  % [T]
        HFE_DATA = [0, 36, 47, 55, 63, 70, 78, 86, 95, 106, ...
                    122, 147, 184, 258, 473, 1230, 3110, 6350, 13000, 25500, ...
                    46000, 76700, 117000, 170000, 233000, 301000, 373000, 451000, 531000, 606000, 684000] % [A/m]
    end

    % =====================================================================
    % Constructor
    % =====================================================================
    methods

        function obj = IPMRotorSizer(spec, options)
            % IPMRotorSizer  Create sizer bound to a MotorSpec.
            %
            %   sizer = IPMRotorSizer(spec)
            %   sizer = IPMRotorSizer(spec, AlphaM=0.76, Hm_mm=5.5, ...)
            %   sizer = IPMRotorSizer(spec, Materials=myMaterials)
            %   sizer = IPMRotorSizer(spec, StatorBore_mm=160)
            %
            %   StatorBore_mm, if given, is used directly instead of
            %   deriving the bore from EssonsSizer — needed to reproduce a
            %   worked example (e.g. the paper's Fig. 4 point, D=160mm)
            %   where the bore is a given input, not an Esson-rule output.
            arguments
                spec                    MotorSpec
                options.Materials       MotorMaterials = MotorMaterials()
                options.StatorBore_mm   (1,1) double = NaN
                options.AlphaM          (1,1) double = 0.754
                options.Whr_fraction    (1,1) double = 0.55
                options.Wob_mm          (1,1) double = 0.5
                options.Hm_mm           (1,1) double = 6.0
                options.Vtilt_deg       (1,1) double = 78
                options.Wib_mm          (1,1) double = 2.5
                options.HryFraction     (1,1) double = 1.5
                options.WindingFactor   (1,1) double = 0.91
                options.Has_mm          (1,1) double = 1.5
                options.WireClearance_mm (1,1) double = 0.3
                options.RhoEV           (1,1) double = 0.650
                options.RhoBtS          (1,1) double = 0.704
                options.RhoHteG         (1,1) double = 52.4
                options.SigmaAnis       (1,1) double = 4.11
                options.Cd              (1,1) double = 0.201
                options.MaxIterations   (1,1) double = 20
                options.Tolerance       (1,1) double = 1e-4
            end

            obj.spec_                   = spec;
            obj.materials_              = options.Materials;
            obj.statorBoreOverride_mm_  = options.StatorBore_mm;

            % Apply optional overrides
            obj.AlphaM        = options.AlphaM;
            obj.Whr_fraction  = options.Whr_fraction;
            obj.Wob_mm        = options.Wob_mm;
            obj.Hm_mm         = options.Hm_mm;
            obj.Vtilt_deg     = options.Vtilt_deg;
            obj.Wib_mm        = options.Wib_mm;
            obj.HryFraction   = options.HryFraction;
            obj.WindingFactor = options.WindingFactor;
            obj.Has_mm        = options.Has_mm;
            obj.WireClearance_mm = options.WireClearance_mm;
            obj.RhoEV         = options.RhoEV;
            obj.RhoBtS        = options.RhoBtS;
            obj.RhoHteG       = options.RhoHteG;
            obj.SigmaAnis     = options.SigmaAnis;
            obj.Cd            = options.Cd;
            obj.MaxIterations = options.MaxIterations;
            obj.Tolerance     = options.Tolerance;

            % Build BH interpolant once (pchip, extrapolate flat at max H)
            obj.Hfe_interp_ = griddedInterpolant( ...
                IPMRotorSizer.BFE_DATA, IPMRotorSizer.HFE_DATA, 'pchip');
        end
    end

    % =====================================================================
    % Dependent property getters
    % =====================================================================
    methods

        function m = get.Materials(obj)
            m = obj.materials_;
        end

        function Br = get.Br_T(obj)
            % Temperature-corrected remanent flux density (linear model,
            % same formula MotorSpec.Br_T used before Br20 moved here).
            s = obj.spec_;
            Br = obj.materials_.Br * (1 + s.kBr_pctPerC/100 * (s.PMTemp_C - 20));
        end
    end

    % =====================================================================
    % Public methods
    % =====================================================================
    methods (Access = public)

        function solve(obj)
            % solve  Run the full sizing loop until iterative params converge.
            %
            %   Convergence is judged on the four iterative parameters:
            %   RhoBtS, RhoHteG, SigmaAnis, Cd.
            %   On exit, Converged = true and all result properties are set.

            % Derive the stator bore from Esson sizing, unless the caller
            % supplied an explicit override (e.g. to match a worked example).
            if isnan(obj.statorBoreOverride_mm_)
                ess = EssonsSizer(obj.spec_);
                ess.solveD2L();
                obj.statorBore_mm_ = ess.StatorBore_mm;
            else
                obj.statorBore_mm_ = obj.statorBoreOverride_mm_;
            end
            obj.StatorBore_mm = obj.statorBore_mm_;

            obj.Converged  = false;
            obj.Iterations = 0;

            for k = 1:obj.MaxIterations
                obj.Iterations = k;

                % Save current iterative params to check convergence
                prev = [obj.RhoBtS, obj.RhoHteG, obj.SigmaAnis, obj.Cd];

                % Run all computation stages in order
                obj.computeRotorGeometry_();
                obj.computeSaturationModel_();
                obj.computeTorqueSizing_();
                obj.computeStatorWindingSizing_();
                obj.computeElectricalParameters_();

                % Check convergence — max relative change across all four
                curr   = [obj.RhoBtS, obj.RhoHteG, obj.SigmaAnis, obj.Cd];
                relErr = max(abs(curr - prev) ./ (abs(prev) + 1e-12));

                if relErr < obj.Tolerance
                    obj.Converged = true;
                    break
                end
            end

            if ~obj.Converged
                warning('IPMRotorSizer:notConverged', ...
                    'solve() did not converge in %d iterations (relErr=%.2e).', ...
                    obj.MaxIterations, relErr);
            end

            % Rotor bridge stress check — not part of the convergence loop
            % (geometry-only, doesn't feed back into RhoBtS/RhoHteG/etc.)
            obj.computeBridgeStress_();
        end

        function summary(obj)
            % summary  Print sizing results with paper reference values.
            obj.requireSolved_('summary');
            fprintf('\nIPMRotorSizer results  (converged=%d, iterations=%d)\n', ...
                obj.Converged, obj.Iterations);
            fprintf('%s\n', repmat('-', 1, 60));

            fprintf('\n  Rotor geometry\n');
            fprintf('    %-28s %7.2f mm\n',    'Stator bore D:',    obj.StatorBore_mm);
            fprintf('    %-28s %7.2f mm\n',    'Rotor OD  D_r:',    obj.RotorOD_mm);
            fprintf('    %-28s %7.2f mm\n',    'Rotor ID  D_ir:',   obj.RotorID_mm);
            fprintf('    %-28s %7.3f mm\n',    'Rotor pole pitch:',  obj.RotorPolePitch_mm);
            fprintf('    %-28s %7.2f mm\n',  'Outer bridge h_ob:', obj.Hob_mm);
            fprintf('    %-28s %7.2f deg\n',   'Side PM angle zeta:', obj.Zeta_deg);
            fprintf('    %-28s %7.2f mm\n',  'Inner bridge h_ib:', obj.Hib_mm);
            fprintf('    %-28s %7.2f mm\n',  'Half-rib h_hr:',     obj.Hhr_mm);
            fprintf('    %-28s %7.2f mm\n',  'Pole-shoe depth d_ps:', obj.Dps_mm);
            fprintf('    %-28s %7.2f mm\n', 'PM width b_m:',      obj.Bm_mm);
            fprintf('    %-28s %7.2f mm\n','Rotor ID D_ir:',     obj.RotorID_mm);

            fprintf('\n  Saturation model\n');
            fprintf('    %-28s %7.4f\n',       'Carter factor k_C:',  obj.CarterFactor);
                fprintf('    %-28s %7.4f mWb/m\n', ...
                    'phi_go:', obj.PhiGo_Wbm*1e3);
                fprintf('    %-28s %7.4f T\n',  'B_go:',  ...
                    obj.PhiGo_Wbm / (obj.AlphaM * obj.RotorPolePitch_mm*1e-3));
                fprintf('    %-28s %7.4f T\n',  'B_g1o:', obj.Bg1o_T);
                fprintf('    %-28s %7.4f mWb/m\n', ...
                    'phi_g1o:', obj.PhiG1o_Wbm*1e3);

            fprintf('\n  Torque sizing\n');
                fprintf('    %-28s %7.2f deg\n', ...
                    'gamma_opt:', obj.GammaOpt_deg);
                fprintf('    %-28s %7.4f\n', ...
                    'eta_phi at corner:', obj.EtaPhi_c);
                fprintf('    %-28s %7.4f\n', ...
                    'sigma_s at corner:', obj.SigmaS_c);
                fprintf('    %-28s %7.4f kNm/m\n', ...
                    'Specific torque T_ell:', obj.SpecificTorque_kNmm);
                fprintf('    %-28s %7.1f mm\n', ...
                    'Stack length ell:', obj.StackLength_mm);

            fprintf('\n  Winding data & stator core dimensions (eqs. 65-94)\n');
                fprintf('    %-28s %7.1f\n',   'Conductors in series U_c:', obj.ConductorsInSeries);
                fprintf('    %-28s %7d\n',     'Conductors in slot u:',     obj.ConductorsInSlot);
                fprintf('    %-28s %7.2f A\n', 'Phase current I_c:',        obj.PhaseCurrent_A);
                fprintf('    %-28s %7d\n',     'Strands in hand n_w:',      obj.NumStrands);
                fprintf('    %-28s %7.3f mm\n','Wire diameter d_wcu:',      obj.WireDiameter_mm);
                fprintf('    %-28s %7.2f mm\n','Tooth width b_ts:',         obj.ToothWidth_mm);
                fprintf('    %-28s %7.2f mm\n','Slot width (minor) b_1:',   obj.SlotWidthInner_mm);
                fprintf('    %-28s %7.2f mm\n','Slot width (major) b_2:',   obj.SlotWidthOuter_mm);
                fprintf('    %-28s %7.2f mm\n','Slot height h:',            obj.SlotHeight_mm);
                fprintf('    %-28s %7.2f mm^2\n','Slot area A_slot:',       obj.SlotArea_mm2);
                fprintf('    %-28s %7.2f mm\n','Stator yoke height h_sy:',  obj.StatorYokeHeight_mm);
                fprintf('    %-28s %7.1f mm\n','External stator OD D_es:',  obj.StatorOD_mm);

            fprintf('\n  Electrical parameters\n');
                fprintf('    %-28s %7.4f\n', 'c_d:', obj.Cd);
                fprintf('    %-28s %7.4f\n', 'c_q:', obj.Cq);
                fprintf('    %-28s %7.4f\n',  ...
                    'sigma_an,o:', obj.SigmaAnis);
                fprintf('    %-28s %7.4f µH/m\n', ...
                    'lambda_is:', obj.LambdaIs_uHm);
            fprintf('    %-28s %7.4f mH\n', 'L_d:', obj.Ld_mH);
            fprintf('    %-28s %7.4f mH\n', 'L_q (corner):', obj.Lq_mH);
            fprintf('    %-28s %7.5f Wb\n', 'Psi_PM1:', obj.PsiPM1_Wb);
            fprintf('    %-28s %7.2f A\n', 'I_d (corner):', obj.Id_A);
            fprintf('    %-28s %7.2f A\n', 'I_q (corner):', obj.Iq_A);

            fprintf('\n  Rotor bridge stress check (n_max, eqs. 19-22)\n');
            fprintf('    %-28s %7.2f N/m\n', 'f_max:',        obj.FMaxSpecific_Nm);
            fprintf('    %-28s %7.2f MPa\n', 'sigma_ib.i:',   obj.SigmaIbIdeal_MPa);
            fprintf('    %-28s %7.2f MPa\n', 'sigma_ib:',     obj.SigmaIb_MPa);
            fprintf('    %-28s %7.3f  (safe=%d)\n', 'sigma_ib/sigma_y_lam:', ...
                obj.SigmaIbRatio, obj.BridgeSafe);
            fprintf('%s\n\n', repmat('-', 1, 60));
        end

        function s = toStruct(obj)
            % toStruct  Export all results as a plain struct.
            %   Passes cleanly to EssonSizer and MotorGeometry constructors.
            obj.requireSolved_('toStruct');
            s = struct( ...
                'StatorBore_mm',       obj.StatorBore_mm, ...
                'RotorOD_mm',          obj.RotorOD_mm, ...
                'RotorID_mm',          obj.RotorID_mm, ...
                'RotorPolePitch_mm',   obj.RotorPolePitch_mm, ...
                'Bps_mm',              obj.Bps_mm, ...
                'Hob_mm',              obj.Hob_mm, ...
                'Zeta_deg',            obj.Zeta_deg, ...
                'Hib_mm',              obj.Hib_mm, ...
                'Hhr_mm',              obj.Hhr_mm, ...
                'D12_mm',              obj.D12_mm, ...
                'D23_mm',              obj.D23_mm, ...
                'D24_mm',              obj.D24_mm, ...
                'Dps_mm',              obj.Dps_mm, ...
                'Bm_mm',               obj.Bm_mm, ...
                'Whr_mm',              obj.Whr_mm, ...
                'Hry_mm',              obj.Hry_mm, ...
                'AlphaM',              obj.AlphaM, ...
                'Hm_mm',               obj.Hm_mm, ...
                'Vtilt_deg',           obj.Vtilt_deg, ...
                'Wib_mm',              obj.Wib_mm, ...
                'Wob_mm',              obj.Wob_mm, ...
                'CarterFactor',        obj.CarterFactor, ...
                'PhiGo_Wbm',           obj.PhiGo_Wbm, ...
                'Bg1o_T',              obj.Bg1o_T, ...
                'PhiG1o_Wbm',          obj.PhiG1o_Wbm, ...
                'StackLength_mm',      obj.StackLength_mm, ...
                'GammaOpt_deg',        obj.GammaOpt_deg, ...
                'SpecificTorque_kNmm', obj.SpecificTorque_kNmm, ...
                'EtaPhi_c',            obj.EtaPhi_c, ...
                'SigmaS_c',            obj.SigmaS_c, ...
                'WindingFactor',       obj.WindingFactor, ...
                'Cd',                  obj.Cd, ...
                'Cq',                  obj.Cq, ...
                'SigmaAnis',           obj.SigmaAnis, ...
                'LambdaIs_uHm',        obj.LambdaIs_uHm, ...
                'Ld_mH',               obj.Ld_mH, ...
                'Lq_mH',               obj.Lq_mH, ...
                'PsiPM1_Wb',           obj.PsiPM1_Wb, ...
                'FMaxSpecific_Nm',     obj.FMaxSpecific_Nm, ...
                'SigmaIbIdeal_MPa',    obj.SigmaIbIdeal_MPa, ...
                'SigmaIb_MPa',         obj.SigmaIb_MPa, ...
                'SigmaIbRatio',        obj.SigmaIbRatio, ...
                'BridgeSafe',          obj.BridgeSafe, ...
                'Converged',           obj.Converged, ...
                'Iterations',          obj.Iterations);
        end

        function [sigma, eta] = saturationAt(obj, Mq_A)
            % saturationAt  Return sigma_sM and eta_phiM at a given Mq [A].
            obj.requireSolved_('saturationAt');
            sigma = obj.sigmaSM_(Mq_A);
            eta   = obj.etaPhiM_(Mq_A);
        end

    end % public methods

    % =====================================================================
    % Public static — data accessors
    % =====================================================================
    methods (Static, Access = public)

        function [B_T, H_Am] = bhCurveData()
            % bhCurveData  Return the M235-35A BH curve dataset used
            %   internally for the saturation model (paper Fig. 2), so it
            %   can be plotted/inspected without duplicating the numbers.
            %
            %   [B_T, H_Am] = IPMRotorSizer.bhCurveData();
            B_T  = IPMRotorSizer.BFE_DATA;
            H_Am = IPMRotorSizer.HFE_DATA;
        end

    end

    % =====================================================================
    % Private — four computation stages
    % =====================================================================
    methods (Access = private)

        function computeRotorGeometry_(obj)
            % Implements paper Section II / eqs. 8–18.
            s  = obj.spec_;
            p  = s.Poles;
            D  = obj.statorBore_mm_; % [mm]
            g  = s.Airgap_mm;       % [mm]
            N_s  = s.Slots;

            % Basic derived quantities
            D_r   = D - 2*g;                          % rotor OD  [mm]
            tau_s = pi * D / N_s;                      % stator slot pitch [mm]
            tau_r = pi * D_r / p;                     % rotor pole pitch  [mm]

            % Rotor design choices
            alpha_m = obj.AlphaM;
            w_hr    = obj.Whr_fraction * tau_s;       % half-rib width [mm]
            w_ob    = obj.Wob_mm;
            h_m     = obj.Hm_mm;
            v       = obj.Vtilt_deg;
            w_ib    = obj.Wib_mm;
            h_ry    = obj.HryFraction * w_hr;

            % Eq. (8)  pole-shoe extension
            b_ps = alpha_m * tau_r;

            % Eq. (9)  outer bridge tangential length
            h_ob = (tau_r - 2*w_hr - b_ps) / 2;
            if h_ob <= 0
                error('IPMRotorSizer:geometry', ...
                    'h_ob <= 0 (%.3f mm). Reduce AlphaM or Whr_fraction.', h_ob);
            end

            % Eq. (10) side PM angle zeta
            arg  = h_ob * (D_r - 2*w_ob) / D_r / h_m;
            if abs(arg) > 1
                error('IPMRotorSizer:geometry', ...
                    'acos argument out of range (%.4f) for zeta.', arg);
            end
            zeta = acosd(arg);                        % [deg]

            % Eq. (11) inner bridge length
            h_ib = h_m * sind(v);

            % Eq. (12) half-rib radial length
            h_hr = h_m * sind(zeta);

            % Eq. (13) construction length d_12
            d_12 = (D_r/2 - w_ob) * sin(alpha_m * pi/p);

            % Eq. (14) construction length d_23
            d_23 = (d_12 - w_ib/2) / tand(v);

            % Eq. (15) construction length d_24
            d_24 = (D_r/2 - w_ob) - d_12 / tan(alpha_m * pi/p);

            % Eq. (16) pole-shoe radial depth
            d_ps = d_23 + d_24 + w_ob;

            % Eq. (17) rotor inner diameter
            D_ir = D_r - 2*(d_ps + h_ib + h_ry);
            if D_ir <= 0
                error('IPMRotorSizer:geometry', ...
                    'D_ir = %.2f mm <= 0. Check rotor design choices.', D_ir);
            end

            % Eq. (18) PM segment width
            b_m = (d_12 - w_ib/2) / sind(v);

            % Store results
            obj.RotorOD_mm        = D_r;
            obj.RotorID_mm        = D_ir;
            obj.RotorPolePitch_mm = tau_r;
            obj.Bps_mm            = b_ps;
            obj.Hob_mm            = h_ob;
            obj.Zeta_deg          = zeta;
            obj.Hib_mm            = h_ib;
            obj.Hhr_mm            = h_hr;
            obj.D12_mm            = d_12;
            obj.D23_mm            = d_23;
            obj.D24_mm            = d_24;
            obj.Dps_mm            = d_ps;
            obj.Bm_mm             = b_m;
            obj.Whr_mm            = w_hr;
            obj.Hry_mm            = h_ry;
        end

        function computeSaturationModel_(obj)
            % Implements paper Section III (eqs. 3–7) and eqs. 38–42.
            %
            % Builds anonymous functions sigma_sM(Mq) and eta_phiM(Mq)
            % that are used by computeTorqueSizing_ and
            % computeElectricalParameters_.  Nothing is stored except
            % the three scalar outputs needed by the other stages.

            s      = obj.spec_;
            mu0    = IPMRotorSizer.MU0;
            g_m    = s.Airgap_mm * 1e-3;              % [m]
            b_as_m = s.SlotOpening_mm * 1e-3;         % [m]
            tau_s_m = pi * obj.statorBore_mm_*1e-3 / s.Slots;  % [m]
            k_st   = s.StackingFactor;
            rho_bt_s = obj.RhoBtS;
            rho_hte_g = obj.RhoHteG;

            % Carter's factor  (standard slot-opening formula)
            gamma_c  = (b_as_m/g_m)^2 / (5 + b_as_m/g_m);
            k_C      = tau_s_m / (tau_s_m - gamma_c*g_m);
            obj.CarterFactor = k_C;

            % BH interpolant (already built in constructor)
            Hfe = @(B) obj.Hfe_interp_(abs(B));       % [A/m]

            % Eq. (3)  tooth flux density from air-gap flux density
            % (implicit): 0 = BtG - BgI/(rho_bt_s*k_st)
            %                 + mu0*Hfe(BtG)/k_st * (1/rho_bt_s - 1)
            % Solved the same way as eq. (6) (root-find via
            % invertMonotone_/fzero). An earlier version dropped the
            % iron-MVD correction term entirely (Bt ≈ B_gI/(rho_bt_s*k_st)),
            % which tracks this solved version closely at low B but
            % increasingly overstates Bt (and so understates saturation)
            % as B grows -- consistent with the sigma_sM(Mq) knee in
            % plot_saturation_factor.m landing earlier/steeper than the
            % paper's Fig. 3 before this fix.
            %
            % Solved once on a BgI grid (matching eq. (6)'s own [0,4.0]
            % search domain below) and interpolated, rather than re-solved
            % on every call: eq. (6)'s root-search alone samples ~200 BgI
            % points, and it's itself called ~200 times per findGammaOpt_
            % grid search -- solving eq. (3) on demand at every one of
            % those points would nest one root-find inside another and
            % make solve() prohibitively slow.
            BgI_grid_for_Bt = linspace(0, 4.0, 200);
            Bt_grid = arrayfun(@(BgI) IPMRotorSizer.invertMonotone_( ...
                @(BtG) BtG - BgI ./ (rho_bt_s * k_st) ...
                       + mu0 .* Hfe(BtG) ./ k_st .* (1/rho_bt_s - 1), ...
                0, 0, 6.0), BgI_grid_for_Bt);
            Bt_of_BgI = griddedInterpolant(BgI_grid_for_Bt, Bt_grid, 'pchip');

            % Eq. (4)  saturation ratio rho_sat(B_gI)
            rho_sat = @(BgI) 1 + Hfe(Bt_of_BgI(BgI)) .* rho_hte_g ...
                                 ./ (BgI / mu0 / k_C);

            % Eq. (5)  peak MMF M_I producing B_gI
            M_of_BgI = @(BgI) (BgI/mu0) .* g_m .* k_C .* rho_sat(BgI);

            % Eq. (6)  invert: B_gI from q-axis MMF Mq
            Bg_of_Mq = @(Mq) IPMRotorSizer.invertMonotone_( ...
                              M_of_BgI, Mq, 0, 4.0);

            % Eq. (7)  saturation factor sigma_sM(Mq)
            % σ_sM = B_p(Mq) / (mu0 * Mq / (g*k_C))
            %       = B_p(Mq) * g * k_C / (mu0 * Mq)
            % NOTE: an earlier version of this line had mu0 multiplying
            % B_p instead of dividing it -- dimensionally wrong (gave
            % T^2/A^2, not pu) and off by ~6 orders of magnitude
            % numerically. Fixed 2026-07-22; see plot_saturation_factor.m
            % for the eq. (7) vs. paper Fig. 3 comparison.
            sigma_fn = @(Mq) Bg_of_Mq(Mq) .* g_m .* k_C ...
                             ./ (mu0 .* Mq + 1e-12);

            % ---- PM flux saturation factor eta_phiM(Mq) — Eq. (38) ----
            % Full magnetic network (Fig. 5) is approximated here by a
            % smooth analytical fit calibrated to paper Fig. 7.
            % Replace this with the full network implementation if needed.
            %
            % Left as a documented gap (2026-07-23): this and the leakage
            % approximation below are now the dominant source of the
            % remaining ~10-20% error in Bg1o_T/GammaOpt_deg/StackLength_mm
            % (see test_IPMRotorSizer.m's diagnostic block) -- eqs. 3-7,
            % 57, and 96-110 were all found and fixed this session and now
            % match the paper to <2.5%, isolating this as the one
            % remaining known-approximate stage. Deliberately not fixed:
            % the full nested-root-solve magnetic network (eqs. 23-37) is
            % a second-order correction relative to everything else now
            % implemented, and FEM will validate the final numbers anyway.
            eta_fn = @(Mq) 1 ./ (1 + 0.55*(Mq./3000).^2);

            % ---- No-load flux density and fundamental — Eqs. 39–42 ----
            % phi_go: specific air-gap flux within pole shoe at Mq=0
            % Estimated from PM residual flux minus bridge leakage (~16.6%)
            alpha_m  = obj.AlphaM;
            b_m_m    = obj.Bm_mm * 1e-3;
            tau_r_m  = obj.RotorPolePitch_mm * 1e-3;
            B_r      = obj.Br_T; % from Materials.Br (20degC), temp-corrected by spec.PMTemp_C/kBr_pctPerC

            phi_PM   = B_r * 2 * b_m_m;              % Eq. (24) [Wb/m]
            % Bridge leakage permeance (both bridges combined, per unit length)
            % — uses a simplified ratio matching paper Fig. 8 at Mq=0
            leakage_no_load = 0.166;
            phi_go   = phi_PM * (1 - leakage_no_load) * alpha_m; % [Wb/m]

            % Eq. (40) no-load flux density in pole shoe
            B_go  = phi_go / (alpha_m * tau_r_m);    % [T]

            % Eq. (41) fundamental component
            B_g1o = (4/pi) * sin(alpha_m * pi/2) * B_go;  % [T]

            % Eq. (42) no-load fundamental specific flux [Wb/m]
            phi_g1o = (2/pi) * B_g1o * tau_r_m;

            % Store scalar results
            obj.PhiGo_Wbm   = phi_go;
            obj.Bg1o_T      = B_g1o;
            obj.PhiG1o_Wbm  = phi_g1o;

            % Cache saturation functions for use in later stages
            % (stored as anonymous functions in private properties)
            obj.sigmaSM_  = sigma_fn;
            obj.etaPhiM_  = eta_fn;
        end

        function computeTorqueSizing_(obj)
            % Implements paper Section V — eqs. 57–64.
            s       = obj.spec_;
            mu0     = IPMRotorSizer.MU0;
            k_w     = obj.WindingFactor;
            tau_r_m = obj.RotorPolePitch_mm * 1e-3;  % [m]
            B_g1o   = obj.Bg1o_T;
            Delta   = s.LinearCurrentDensity_Am;      % peak [A/m]
            D_m     = obj.statorBore_mm_ * 1e-3;      % [m]

            % Specific permeance lambda_is — Eq. (51)
            g_m   = s.Airgap_mm * 1e-3;
            k_C   = obj.CarterFactor;
            lambda_is = mu0 * k_w^2 * (3/pi^2) * tau_r_m / (g_m * k_C);
            obj.LambdaIs_uHm = lambda_is * 1e6;       % [µH/m]

            % Optimal phase advance — Eq. (58), solved via line search
            gamma_opt = obj.findGammaOpt_(Delta);
            obj.GammaOpt_deg = gamma_opt;

            % Saturation function values at corner point
            Mq_c = (sqrt(2)/pi) * k_w * tau_r_m * Delta * cosd(gamma_opt);
            obj.EtaPhi_c = obj.etaPhiM_(Mq_c);
            obj.SigmaS_c = obj.sigmaSM_(Mq_c);

            % Optimal specific torque T_ell — Eq. (59) [Nm/m]
            fT_c    = obj.fT_(Delta, gamma_opt);
            T_ell   = fT_c * (pi * k_w / (2*sqrt(2))) * B_g1o * Delta * D_m^2;
            obj.SpecificTorque_kNmm = T_ell / 1e3;   % [kNm/m]

            % Stack length — Eq. (64) [mm]
            ell_m  = s.Torque_Nm / T_ell;
            obj.StackLength_mm = ell_m * 1e3;
        end

        function computeStatorWindingSizing_(obj)
            % Implements paper Section VI — eqs. 65-94: detailed stator
            % core and winding sizing (conductor count, wire gauge, slot
            % geometry, stator yoke, external diameter). These are the
            % dimensions a FEM cross-section actually needs -- previously
            % this was entirely missing, patched over by the unrelated
            % EssonsSizer estimate (README.md's "Known issues" section).
            %
            % Also updates RhoBtS (eq. 86) and RhoHteG (eq. 91), the two
            % iterative parameters this section owns.
            s        = obj.spec_;
            k_w      = obj.WindingFactor;
            k_st     = s.StackingFactor;
            p        = s.Poles;
            N_s      = s.Slots;
            D_mm     = obj.statorBore_mm_;         % [mm]
            g_mm     = s.Airgap_mm;                % [mm]
            f_c      = s.CornerFrequency_Hz;
            phi_g1o  = obj.PhiG1o_Wbm;              % [Wb/m]
            ell_m    = obj.StackLength_mm * 1e-3;   % [m]
            eta_phi_c = obj.EtaPhi_c;
            Delta_c  = s.LinearCurrentDensity_Am;   % [A/m]
            B_g1o    = obj.Bg1o_T;
            B_ts     = s.Bt_T;
            B_ys     = s.Bc_T;
            b_as     = s.SlotOpening_mm;            % [mm]
            tau_s_mm = pi * D_mm / N_s;              % slot pitch [mm]

            % Eq. (65) fundamental pole flux at corner [Wb]
            Phi_g1c = eta_phi_c * phi_g1o * ell_m;

            % Eq. (66) conductor EMF [V]
            E_cc = (pi/sqrt(2)) * f_c * Phi_g1c;

            % Eq. (68) max inverter phase voltage [V]
            V_invM = 0.95 * s.VdcLink_V / (2*sqrt(2));

            % Eq. (69) theoretical conductors in series (rho_EV, eq. 67,
            % is a design target -- the paper gives no closed-form for it
            % either, just an "iterative result" -- so it's a configurable
            % property here, not solved for)
            Uc_th = (obj.RhoEV * V_invM) / (k_w * E_cc);

            % Eq. (70) parallel paths
            a = p/2;

            % Eq. (71) theoretical conductors per slot
            u_th = (3 * Uc_th * a) / N_s;

            % Eq. (72) actual conductors per slot (rounded to an even number)
            u = 2 * round(0.5 * u_th);

            % Eq. (73) actual conductors in series per phase
            Uc = (N_s * u) / (3 * a);

            % Eq. (75) phase current [A rms]
            I_c = (Delta_c * pi * (D_mm*1e-3)) / (3 * Uc);

            % Eq. (76) path current [A]
            I_c_path = I_c / a;

            % Eq. (78) elementary conductor cross section [mm^2]
            S_cth = s.CurrentDensity_Amm2;
            A_u = I_c_path / S_cth;

            % Eq. (79) max single-wire diameter -- limited by the slot
            % opening b_as (the wire bundle must physically pass through
            % it during insertion), minus an insertion/insulation clearance
            d_wmax = b_as - obj.WireClearance_mm;

            % Eq. (80) strands in hand (split into thinner parallel
            % strands so each one still fits through the slot opening)
            n_w = ceil((4/pi) * A_u / d_wmax^2);

            % Eq. (81) wire diameter [mm]
            d_wcu = sqrt((4/pi) * (A_u / n_w));

            % Eq. (82) copper cross section in slot [mm^2]
            A_cu_slot = u * n_w * (pi/4) * d_wcu^2;

            % Eq. (84) slot cross section [mm^2]
            alpha_cu = s.CopperFillFactor;
            A_slot = A_cu_slot / alpha_cu;

            % Eq. (85) tooth width [mm]
            b_ts = (B_g1o / B_ts) * tau_s_mm / k_st;

            % Eq. (86) rho_bt.s ratio -- feeds back as the iterative RhoBtS
            rho_bt_s = b_ts / tau_s_mm;

            % Eq. (87) minor slot width (bottom, bore side) [mm]
            h_as = obj.Has_mm;
            b_1 = (pi*(D_mm + 2*h_as) - N_s*b_ts) / (N_s - pi);

            % Eq. (88) auxiliary slot taper parameter
            k_0 = tan(pi / N_s);

            % Eq. (89) slot height [mm] -- closed-form root of the
            % trapezoidal slot-area quadratic (slot sides taper outward
            % at angle theta = pi/N_s)
            h = (-b_1 + sqrt(b_1^2 + 2*k_0*(2*A_slot - b_1^2*pi/4))) / (2*k_0);

            % Eq. (90) total tooth equivalent height [mm] -- includes the
            % rotor half-rib height Hhr_mm, saturated by Mpq roughly the
            % same way as the stator teeth (paper's own remark under eq. 90)
            h_teq = h + b_1/2 + h_as + obj.Hhr_mm;

            % Eq. (91) rho_hte.g ratio -- feeds back into the saturation
            % model (eq. 4's rho_sat) as the iterative RhoHteG
            rho_hte_g = h_teq / g_mm;

            % Eq. (92) major slot width (top, yoke side) [mm]
            b_2 = b_1 + 2*h*k_0;

            % Eq. (93) stator yoke height [mm]
            h_sy = phi_g1o / (2 * B_ys * k_st) * 1e3;

            % Eq. (94) external stator diameter [mm]
            D_es = D_mm + 2*(h_as + h + h_sy);

            % Store results
            obj.ConductorsInSeries       = Uc;
            obj.ConductorsInSlot         = u;
            obj.PhaseCurrent_A           = I_c;
            obj.NumStrands               = n_w;
            obj.WireDiameter_mm          = d_wcu;
            obj.ToothWidth_mm            = b_ts;
            obj.SlotWidthInner_mm        = b_1;
            obj.SlotWidthOuter_mm        = b_2;
            obj.SlotHeight_mm            = h;
            obj.SlotArea_mm2             = A_slot;
            obj.StatorYokeHeight_mm      = h_sy;
            obj.StatorOD_mm              = D_es;

            obj.RhoBtS  = rho_bt_s;
            obj.RhoHteG = rho_hte_g;
        end

        function computeElectricalParameters_(obj)
            % Implements paper Section VII — eqs. 95–110.
            % Updates the iterative parameters Cd, SigmaAnis, RhoBtS, RhoHteG.
            s       = obj.spec_;
            p       = s.Poles;
            torque_Nm = s.Torque_Nm;
            mu0     = IPMRotorSizer.MU0;
            k_w     = obj.WindingFactor;
            k_C     = obj.CarterFactor;
            k_st    = s.StackingFactor;
            tau_r_m = obj.RotorPolePitch_mm * 1e-3;
            D_r_m   = obj.RotorOD_mm * 1e-3;
            g_m     = s.Airgap_mm * 1e-3;
            alpha_m = obj.AlphaM;
            h_ob_m  = obj.Hob_mm  * 1e-3;
            h_m_m   = obj.Hm_mm   * 1e-3;
            w_ib_m  = obj.Wib_mm  * 1e-3;
            ell_m   = obj.StackLength_mm * 1e-3;
            lambda_is = obj.LambdaIs_uHm * 1e-6;     % [H/m]
            phi_g1o   = obj.PhiG1o_Wbm;
            B_g1o     = obj.Bg1o_T;

            % ---- d-axis reaction coefficient c_d — Eq. (104)–(106) ----
            % Radial permeance of inner rotor to pole shoe  Eq. (99)
            b_m_m  = obj.Bm_mm * 1e-3;
            vtilt  = obj.Vtilt_deg;
            Lambda_ps_ir = mu0 * ell_m * (2*b_m_m + h_m_m*cosd(vtilt) ...
                           + w_ib_m) / h_m_m;         % Eq. (99)

            % Air-gap permeance in front of pole shoe — Eq. (102)
            Lambda_g = mu0 * alpha_m * (tau_r_m * ell_m) / (k_C * g_m);

            % Pole-shoe potential fraction — Eq. (104) simplified
            % U_psd / M_pd = sin(alpha_m*pi/2) / (alpha_m*pi/2)
            %                 * 1/(1 + Lambda_ps_ir/Lambda_g)
            sin_ratio = sin(alpha_m*pi/2) / (alpha_m*pi/2);
            Upsd_frac = sin_ratio / (1 + Lambda_ps_ir/Lambda_g);

            % theta_e,rib — Eq. (98)/(107): alpha_m*pi/2 + (p/2)*h_ob/(D_r/2)
            % NOTE: an earlier version divided by D_r_m (diameter) instead
            % of D_r_m/2 (radius), which is what the paper's formula
            % literally uses -- that halved the (p/2)*h_ob/(D_r/2) term.
            % Fixed 2026-07-23.
            e_rib = alpha_m*pi/2 + (p/2)*h_ob_m/(D_r_m/2);
            a_pole = alpha_m*pi/2;

            % Fundamental of b_pd(theta_e) — Eq. (105), then c_d — Eq. (106)
            % b_pd(theta_e) is piecewise (Eq. 98): the d-axis reaction MVD
            % term M_pd*cos(theta_e)-U_pspd in [0, a_pole], zero across the
            % rib [a_pole, e_rib], and the isotropic term B_pd.is*cos(theta_e)
            % beyond the rib [e_rib, pi/2]. Integrating (4/pi)*int(b_pd*cos)
            % analytically (int cos^2 = x/2+sin(2x)/4, int cos = sin(x)) and
            % dividing by B_pd.is = mu0*M_pd/(kC*g) gives this closed form,
            % using U = Upsd_frac (eq. 104) for U_pspd/M_pd:
            %
            % An earlier version replaced this integral with an invented
            % approximation, (1-Upsd_frac)*(1-h_ob/(D_r/2)), that doesn't
            % correspond to integrating Eq. (98) at all. This closed form,
            % combined with the e_rib fix above, reproduces the paper's
            % Table I value (c_d=0.201) to <1%.
            c_d_new = (4/pi) * ( a_pole/2 + sin(2*a_pole)/4 - Upsd_frac*sin(a_pole) ...
                       + pi/4 - e_rib/2 - sin(2*e_rib)/4 );

            % Fundamental of b_pq(theta_e) — Eq. (108), then c_q — Eq. (109)
            % b_pq(theta_e) = B_pq.is*sin(theta_e) everywhere except zero
            % across the rib (Eq. 107) -- no unknown potential here, unlike
            % the d-axis case. Integrating (4/pi)*int(b_pq*sin) analytically
            % (int sin^2 = x/2-sin(2x)/4) over [0,a_pole] union [e_rib,pi/2]
            % gives this closed form. Reproduces the paper's Table I value
            % (c_q=0.825) to <1%. An earlier version used a differently-
            % structured formula (single-angle cos/sin terms, consistent
            % with assuming a uniform rather than sin(theta_e)-distributed
            % flux density) that didn't match eq. 107's stated distribution.
            c_q_new = (4/pi) * ( a_pole/2 - sin(2*a_pole)/4 ...
                       + pi/4 - e_rib/2 + sin(2*e_rib)/4 );

            % Anisotropy ratio — Eq. (53/110)
            sigma_anis_new = c_q_new / c_d_new;

            % ---- Inductances — Eq. (120) ----
            % Uc (actual conductors in series per phase) comes from
            % computeStatorWindingSizing_ (eq. 73), which runs earlier in
            % the same solve() iteration -- no need to re-estimate it here.
            Uc = obj.ConductorsInSeries;

            L_pdo  = c_d_new  * lambda_is * (Uc^2/p) * ell_m;   % [H]
            L_pqo  = c_q_new  * lambda_is * (Uc^2/p) * ell_m;
            L_pq_c = L_pqo * obj.SigmaS_c;

            % PM flux linkage estimate — Eq. (48) at corner
            PsiPM1 = k_w * Uc * phi_g1o * obj.EtaPhi_c / (2*sqrt(2)) * ell_m;
            Iq = torque_Nm / (1.5 * p/2 * PsiPM1); % Simplified
            Id = 0; % Assuming no field weakening for the base torque point

            % ---- Update iterative parameters ----
            % RhoBtS and RhoHteG are updated in computeStatorWindingSizing_
            % (eqs. 86, 91), which runs earlier in the same solve()
            % iteration -- not duplicated here.
            obj.Cd        = c_d_new;
            obj.Cq        = c_q_new;
            obj.SigmaAnis = sigma_anis_new;

            % Store electrical results
            obj.Ld_mH    = L_pdo * 1e3;
            obj.Lq_mH    = L_pq_c * 1e3;
            obj.PsiPM1_Wb = PsiPM1;
            obj.Iq_A = Iq;
            obj.Id_A = Id;
        end

        function computeBridgeStress_(obj)
            % Implements paper Section IV — eqs. 19-22 (rotor bridge
            % centrifugal stress check). See README.md's "Rotor bridge
            % stress check" section for full sourcing.
            %
            % The paper does not spell out m_1p or R_av explicitly; this
            % uses the same first-order approximation already sketched in
            % the report (magnet-only mass, pole-shoe iron neglected —
            % non-conservative simplification, flagged for a future pass):
            %   m_1p ≈ 2 * rho_pm * b_m * h_m   (per unit stack length)
            %   R_av ≈ Dr/2 - Dps/2             (centroid ~ mid pole-shoe depth)
            s   = obj.spec_;
            mat = obj.materials_;

            Omega_max = 2*pi * s.MaxSpeed_rpm / 60;   % [rad/s]

            b_m_m = obj.Bm_mm  * 1e-3;                % [m]
            h_m_m = obj.Hm_mm  * 1e-3;                % [m]
            R_av  = (obj.RotorOD_mm/2 - obj.Dps_mm/2) * 1e-3;  % [m]

            m1p_specific = 2 * mat.rho_pm * b_m_m * h_m_m;      % [kg/m]

            % Eq. (19) specific max centrifugal force [N/m]
            f_max = m1p_specific * R_av * Omega_max^2;

            % Eq. (20) ideal (nominal) inner bridge stress [Pa]
            w_ib_m = obj.Wib_mm * 1e-3;
            k_st   = s.StackingFactor;
            sigma_ib_ideal = f_max / (w_ib_m * k_st);

            % Eq. (21) actual stress with concentration factor [Pa]
            sigma_ib = mat.Kt_ib * sigma_ib_ideal;

            % Eq. (22) check against lamination yield strength
            ratio = sigma_ib / mat.sigma_y_lam;

            obj.FMaxSpecific_Nm  = f_max;
            obj.SigmaIbIdeal_MPa = sigma_ib_ideal * 1e-6;
            obj.SigmaIb_MPa      = sigma_ib * 1e-6;
            obj.SigmaIbRatio     = ratio;
            obj.BridgeSafe       = ratio < 1;
        end

    end % private compute methods

    % =====================================================================
    % Private — saturation function handles (set by computeSaturationModel_)
    % =====================================================================
    properties (Access = private)
        sigmaSM_    % function handle: sigma_sM(Mq)
        etaPhiM_    % function handle: eta_phiM(Mq)
    end

    % =====================================================================
    % Private — internal helpers
    % =====================================================================
    methods (Access = private)

        function fT = fT_(obj, Delta, gamma_deg)
            % fT_  Torque pu function f_T(Delta, gamma) — Eq. (57).
            %
            %   f_T = f_{T,al} + f_{T,an}
            %       = eta_phi * cos(gamma)
            %         + (sqrt(2)*pi/6) * (c_d*lambda_is)/(k_w*B_g1o)
            %           * Delta * (sigma_an * sigma_s - 1) * sin(2*gamma)
            %
            % NOTE: two bugs fixed here 2026-07-23, found while chasing
            % findGammaOpt_ collapsing to the 1 deg floor of its search
            % grid instead of tracking the paper's ~48 deg:
            %   1. The denominator used phi_g1o (PhiG1o_Wbm, specific
            %      flux [Wb/m]) instead of B_g1o (Bg1o_T, flux density
            %      [T]) -- paper's eq. (57) is explicit it's k_w*B_g1o.
            %      phi_g1o is ~25x smaller than B_g1o, which inflated the
            %      anisotropy term ~25x.
            %   2. An extra tau_r (pole pitch) factor was multiplied in
            %      that doesn't appear in the paper's eq. (57) at all --
            %      also confirmed by dimensional analysis: (c_d*lambda_is)
            %      /(k_w*B_g1o)*Delta is already dimensionless on its own
            %      ([H/m]*[A/m]/[T] = [T]/[T]), so multiplying by tau_r
            %      [m] broke that and made the term ~16x too small
            %      (tau_r ~ 0.062 m).
            % Together these made the anisotropy term ~400x too small,
            % so cos(gamma)'s monotonic decay always won and gamma_opt
            % collapsed to the search grid's lower bound. Verified by
            % substituting the paper's own converged c_d=0.201,
            % sigma_an,o=4.11 (Table I) into the corrected formula:
            % gamma_opt lands at 53 deg, close to the paper's 48.15 deg.
            % The remaining gap comes from c_d/sigma_an,o themselves,
            % which computeElectricalParameters_ derives via its own
            % already-documented heuristic approximation of eqs. 95-110
            % (not fixed here -- see that method's comments).

            k_w       = obj.WindingFactor;
            tau_r_m   = obj.RotorPolePitch_mm * 1e-3;
            B_g1o     = obj.Bg1o_T;
            c_d       = obj.Cd;
            sigma_an  = obj.SigmaAnis;
            lambda_is = obj.LambdaIs_uHm * 1e-6;

            Mq = (sqrt(2)/pi) .* k_w .* tau_r_m .* Delta .* cosd(gamma_deg);

            % Handle Mq == 0 edge case
            safe_Mq = max(Mq, 1e-9);
            eta_phi  = obj.etaPhiM_(safe_Mq);
            sigma_s  = obj.sigmaSM_(safe_Mq);
            eta_phi(Mq < 1e-9) = 1;
            sigma_s(Mq < 1e-9) = 1;

            % Alignment term
            fT_al = eta_phi .* cosd(gamma_deg);

            % Anisotropy term
            fT_an = (sqrt(2)*pi/6) .* (c_d .* lambda_is) ./ (k_w .* B_g1o) ...
                    .* Delta ...
                    .* (sigma_an .* sigma_s - 1) .* sind(2.*gamma_deg);

            fT = fT_al + fT_an;
        end

        function gamma_opt = findGammaOpt_(obj, Delta)
            % findGammaOpt_  Optimal phase advance — Eq. (58).
            %   Coarse grid search then fminbnd refinement.
            gamma_vec = 1:0.5:89;
            fT_vec    = arrayfun(@(g) obj.fT_(Delta, g), gamma_vec);
            [~, idx]  = max(fT_vec);
            g_lo      = max(1,  gamma_vec(idx) - 5);
            g_hi      = min(89, gamma_vec(idx) + 5);
            gamma_opt = fminbnd(@(g) -obj.fT_(Delta, g), g_lo, g_hi);
        end

        function requireSolved_(obj, caller)
            % requireSolved_  Guard: error if solve() has not been called.
            if isnan(obj.StackLength_mm)
                error('IPMRotorSizer:notSolved', ...
                    '%s() called before solve(). Run sizer.solve() first.', ...
                    caller);
            end
        end

    end % private helpers

    % =====================================================================
    % Private static helpers
    % =====================================================================
    methods (Static, Access = private)

        function x = invertMonotone_(fn, y_target, x_lo, x_hi)
            % invertMonotone_  Invert a monotone function fn(x) = y_target.
            %   Robust wrapper: samples the interval, finds a sign change,
            %   then calls fzero.  Returns NaN if inversion fails.
            N  = 200;
            xv = linspace(x_lo, x_hi, N);
            yv = arrayfun(fn, xv) - y_target;

            % Find first sign change
            for i = 1:(N-1)
                if isfinite(yv(i)) && isfinite(yv(i+1)) && yv(i)*yv(i+1) < 0
                    x = fzero(@(xi) fn(xi) - y_target, [xv(i), xv(i+1)]);
                    return
                end
            end
            % Fallback: closest finite point
            [~, idx] = min(abs(yv));
            try
                x = fzero(@(xi) fn(xi) - y_target, xv(idx));
            catch
                x = NaN;
            end
        end

    end

end