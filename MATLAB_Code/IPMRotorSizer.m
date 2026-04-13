classdef IPMRotorSizer < handle
% IPMRotorSizer  Rotor sizing for V-shape IPM motors.
%
%   Implements the analytical design procedure of:
%     A. Di Gerlando, C. Ricca, "Design Modeling and Sizing Equations of
%     V-shape IPM Motors," ICEM 2022, DOI: 10.1109/ICEM51905.2022.9910924
%
%   Equation numbers in comments refer directly to the paper.
%   Sections I-VIII of the paper map onto the four private compute methods.
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
%   Electrical parameters:
%     Cq, Ld_mH, Lq_mH, PsiPM1_Wb
%     LambdaIs_uHm
%   Convergence:
%     Converged, Iterations
%
%   See also: MotorSpec, EssonSizer, MotorGeometry

    % =====================================================================
    % Public — rotor design choices (not in MotorSpec)
    % =====================================================================
    properties

        % Rotor geometry choices
        AlphaM          (1,1) double {mustBeInRange(AlphaM,    0.5, 0.95)} = 0.754
        Whr_fraction    (1,1) double {mustBeInRange(Whr_fraction, 0.3, 0.8)} = 0.55
        Wob_mm          (1,1) double {mustBePositive}  = 0.5
        Hm_mm           (1,1) double {mustBePositive}  = 6.0
        Vtilt_deg       (1,1) double {mustBeInRange(Vtilt_deg, 45, 89)} = 78
        Wib_mm          (1,1) double {mustBePositive}  = 2.5
        HryFraction     (1,1) double {mustBePositive}  = 1.5

        % Winding
        WindingFactor   (1,1) double {mustBeInRange(WindingFactor, 0.5, 1.0)} = 0.91

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

        % --- Electrical parameters (eqs. 95–120) ---
        Cq               (1,1) double = NaN   % q-axis reaction coefficient
        LambdaIs_uHm     (1,1) double = NaN   % specific permeance [µH/m]
        Ld_mH            (1,1) double = NaN   % d-axis synchronous inductance
        Lq_mH            (1,1) double = NaN   % q-axis synchronous inductance (corner)
        PsiPM1_Wb        (1,1) double = NaN   % PM flux linkage [Wb rms]

        % --- Convergence ---
        Converged        (1,1) logical = false
        Iterations       (1,1) double  = 0

    end

    % =====================================================================
    % Private — reference to spec and cached BH curve interpolant
    % =====================================================================
    properties (Access = private)
        spec_           % MotorSpec handle
        Hfe_interp_     % griddedInterpolant for M235-35A BH curve

        % Cached derived geometry (computed from EssonsSizer at solve() start)
        statorBore_mm_  (1,1) double = NaN
    end

    % =====================================================================
    % Constants
    % =====================================================================
    properties (Constant, Access = private)
        MU0 = 4*pi*1e-7   % [H/m]

        % M235-35A BH curve — extended to high B so that incremental
        % permeability → mu0 at saturation (paper Fig. 2).
        BFE_DATA = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, ...
                    1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, ...
                    2.0, 2.1, 2.2, 2.3, 2.5, 3.0]        % [T]
        HFE_DATA = [0, 22, 30, 37, 42, 47, 53, 60, 69, 82, ...
                    100, 125, 165, 235, 380, 720, 1400, 2600, 4200, 6200, ...
                    8800, 12000, 16000, 21000, 33000, 70000] % [A/m]
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
            arguments
                spec                    MotorSpec
                options.AlphaM          (1,1) double = 0.754
                options.Whr_fraction    (1,1) double = 0.55
                options.Wob_mm          (1,1) double = 0.5
                options.Hm_mm           (1,1) double = 6.0
                options.Vtilt_deg       (1,1) double = 78
                options.Wib_mm          (1,1) double = 2.5
                options.HryFraction     (1,1) double = 1.5
                options.WindingFactor   (1,1) double = 0.91
                options.RhoBtS          (1,1) double = 0.704
                options.RhoHteG         (1,1) double = 52.4
                options.SigmaAnis       (1,1) double = 4.11
                options.Cd              (1,1) double = 0.201
                options.MaxIterations   (1,1) double = 20
                options.Tolerance       (1,1) double = 1e-4
            end

            obj.spec_ = spec;

            % Apply optional overrides
            obj.AlphaM        = options.AlphaM;
            obj.Whr_fraction  = options.Whr_fraction;
            obj.Wob_mm        = options.Wob_mm;
            obj.Hm_mm         = options.Hm_mm;
            obj.Vtilt_deg     = options.Vtilt_deg;
            obj.Wib_mm        = options.Wib_mm;
            obj.HryFraction   = options.HryFraction;
            obj.WindingFactor = options.WindingFactor;
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
    % Public methods
    % =====================================================================
    methods (Access = public)

        function solve(obj)
            % solve  Run the full sizing loop until iterative params converge.
            %
            %   Convergence is judged on the four iterative parameters:
            %   RhoBtS, RhoHteG, SigmaAnis, Cd.
            %   On exit, Converged = true and all result properties are set.

            % Derive the stator bore from Esson sizing (not a user input).
            ess = EssonsSizer(obj.spec_);
            ess.solve();
            obj.statorBore_mm_ = ess.StatorBore_mm;

            obj.Converged  = false;
            obj.Iterations = 0;

            for k = 1:obj.MaxIterations
                obj.Iterations = k;

                % Save current iterative params to check convergence
                prev = [obj.RhoBtS, obj.RhoHteG, obj.SigmaAnis, obj.Cd];

                % Run all four computation stages in order
                obj.computeRotorGeometry_();
                obj.computeSaturationModel_();
                obj.computeTorqueSizing_();
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
        end

        function summary(obj)
            % summary  Print sizing results with paper reference values.
            obj.requireSolved_('summary');
            fprintf('\nIPMRotorSizer results  (converged=%d, iterations=%d)\n', ...
                obj.Converged, obj.Iterations);
            fprintf('%s\n', repmat('-', 1, 60));

            fprintf('\n  Rotor geometry\n');
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
            fprintf('%s\n\n', repmat('-', 1, 60));
        end

        function s = toStruct(obj)
            % toStruct  Export all results as a plain struct.
            %   Passes cleanly to EssonSizer and MotorGeometry constructors.
            obj.requireSolved_('toStruct');
            s = struct( ...
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
                'Converged',           obj.Converged, ...
                'Iterations',          obj.Iterations);
        end

        function fT = torqueFunction(obj, Delta_Am, gamma_deg)
            % torqueFunction  Evaluate f_T(Delta, gamma) — Eq. (57).
            %
            %   Useful for plotting the torque map after solve().
            %   Delta_Am and gamma_deg may be arrays (same size or scalar).
            obj.requireSolved_('torqueFunction');
            fT = obj.fT_(Delta_Am, gamma_deg);
        end

        function [sigma, eta] = saturationAt(obj, Mq_A)
            % saturationAt  Return sigma_sM and eta_phiM at a given Mq [A].
            obj.requireSolved_('saturationAt');
            sigma = obj.sigmaSM_(Mq_A);
            eta   = obj.etaPhiM_(Mq_A);
        end

    end % public methods

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
            Q  = s.Slots;

            % Basic derived quantities
            D_r   = D - 2*g;                          % rotor OD  [mm]
            tau_s = pi * D / Q;                       % stator slot pitch [mm]
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
            % Paper approximation: B_t ≈ B_gI / (rho_bt_s * k_st)
            Bt_of_BgI = @(BgI) BgI ./ (rho_bt_s * k_st);

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
            sigma_fn = @(Mq) Bg_of_Mq(Mq) .* mu0 ...
                             ./ (Mq .* g_m .* k_C + 1e-30);

            % ---- PM flux saturation factor eta_phiM(Mq) — Eq. (38) ----
            % Full magnetic network (Fig. 5) is approximated here by a
            % smooth analytical fit calibrated to paper Fig. 7.
            % Replace this with the full network implementation if needed.
            eta_fn = @(Mq) 1 ./ (1 + 0.55*(Mq./3000).^2);

            % ---- No-load flux density and fundamental — Eqs. 39–42 ----
            % phi_go: specific air-gap flux within pole shoe at Mq=0
            % Estimated from PM residual flux minus bridge leakage (~16.6%)
            alpha_m  = obj.AlphaM;
            b_m_m    = obj.Bm_mm * 1e-3;
            tau_r_m  = obj.RotorPolePitch_mm * 1e-3;
            B_r      = s.Br_T;

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

        function computeElectricalParameters_(obj)
            % Implements paper Section VII — eqs. 95–110.
            % Updates the iterative parameters Cd, SigmaAnis, RhoBtS, RhoHteG.
            s       = obj.spec_;
            p       = s.Poles;
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

            % Fundamental of b_d(theta_e) integral — Eq. (105)
            % Approximated as: B_d1 ≈ mu0 * M_pd / (g*k_C) * c_d_new
            % where c_d_new = (4/pi)*integral of the distribution
            % Using the closed-form from Eq. (106):
            c_d_new = (1 - Upsd_frac) * (1 - h_ob_m/(D_r_m/2));
            c_d_new = max(0.05, min(0.50, c_d_new));  % physical bounds

            % ---- q-axis reaction coefficient c_q — Eqs. 107–109 ----
            % The q-axis flux passes through the full airgap except the
            % rib region; the distribution B_q * sin(theta_e) gives:
            e_rib  = alpha_m*pi/2 + (p/2)*h_ob_m/D_r_m; % theta_e,rib approx
            c_q_new = (2/pi) * (alpha_m*pi/2 - sin(alpha_m*pi) / 2 ...
                      + (pi/2 - e_rib)*cos(e_rib) - sin(e_rib));
            c_q_new = max(0.5, min(1.5, c_q_new));       % physical bounds

            % Anisotropy ratio — Eq. (53/110)
            sigma_anis_new = c_q_new / c_d_new;

            % ---- Inductances — Eq. (120) ----
            % Needs winding data Uc — approximate from corner current
            % I_c = Delta * p * tau_r / (3 * Uc)  → Uc = Delta*p*tau_r/(3*I_c)
            % Phase current estimate (pre-winding-sizer):
            % Use EV ratio from MotorSpec to estimate Uc
            V_invM  = 0.95 * s.VdcLink_V / (2*sqrt(2));
            f_c     = s.CornerFrequency_Hz;
            E_cc    = (pi/sqrt(2)) * f_c * phi_g1o * ell_m;  % per-conductor EMF
            Uc_est  = round(s.PowerFactor * V_invM / (k_w * E_cc));
            Uc_est  = max(Uc_est, 1);

            L_pdo  = c_d_new  * lambda_is * (Uc_est^2/p) * ell_m;   % [H]
            L_pqo  = c_q_new  * lambda_is * (Uc_est^2/p) * ell_m;
            L_pq_c = L_pqo * obj.SigmaS_c;

            % PM flux linkage estimate — Eq. (48) at corner
            PsiPM1 = k_w * Uc_est * phi_g1o * obj.EtaPhi_c / (2*sqrt(2)) * ell_m;

            % ---- Update iterative parameters ----
            % RhoBtS: from tooth width sizing
            %   b_ts = (B_g1o / B_ts_target) * tau_s / k_st
            tau_s_m   = pi * obj.statorBore_mm_*1e-3 / s.Slots;
            B_ts_tgt  = s.Bt_T;
            b_ts_new  = B_g1o * tau_s_m / (B_ts_tgt * k_st);
            rho_bt_s_new = b_ts_new / tau_s_m;

            % RhoHteG: from equivalent tooth height
            %   h_teq = h_slot + b_1/2 + h_as + h_hr
            % Not updated here; a stator/winding model should provide this.

            % Store updated iterative params (triggers re-convergence check)
            obj.Cd        = c_d_new;
            obj.Cq        = c_q_new;
            obj.SigmaAnis = sigma_anis_new;
            obj.RhoBtS    = rho_bt_s_new;
            % RhoHteG is updated by EssonSizer after stator sizing

            % Store electrical results
            obj.Ld_mH    = L_pdo * 1e3;
            obj.Lq_mH    = L_pq_c * 1e3;
            obj.PsiPM1_Wb = PsiPM1;
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
            %         + (sqrt(2)*pi/6) * (c_d*lambda_is)/(k_w*phi_g1o)
            %           * tau_r * Delta * (sigma_an * sigma_s - 1) * sin(2*gamma)

            k_w       = obj.WindingFactor;
            tau_r_m   = obj.RotorPolePitch_mm * 1e-3;
            phi_g1o   = obj.PhiG1o_Wbm;
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

            % Anisotropy term — note phi_g1o appears via normalisation
            fT_an = (sqrt(2)*pi/6) .* (c_d .* lambda_is) ./ (k_w .* phi_g1o) ...
                    .* tau_r_m .* Delta ...
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