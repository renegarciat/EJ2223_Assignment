classdef MotorSpec < handle
% MotorSpec  Validated input specification for an IPM motor design.
%
%   MotorSpec is the root input object for the motor design pipeline.
%   It holds only true user inputs; quantities that cannot be derived
%   without an explicit engineering decision. All derived quantities
%   (slot count, pole pitch, rotor diameter, etc.) live in the sizers
%   or in MotorGeometry.
%
%   Construction
%   ------------
%     spec = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, vdcLink_V, ...
%                      poles, slots, airgap_mm, ...
%                      br20_T)
%
%     spec = MotorSpec( ___ , Name=Value)   % override any optional property
%
%   Required inputs (positional)
%   ----------------------------
%     torque_Nm           Corner-point torque [Nm]
%     cornerSpeed_rpm     Speed at corner point [rpm]
%     maxSpeed_rpm        Maximum (flux-weakening) speed [rpm]
%     vdcLink_V           DC-link voltage [V]
%     poles               Number of poles (pole pairs! even integer >= 2)
%     slots               Total number of slots Q
%     airgap_mm           Air-gap length g [mm]
%     br20_T              PM remanent flux density at 20 degC [T]
%
%   Optional name-value inputs (with defaults)
%   ------------------------------------------
%     Phases              Number of phases              (default 3)
%     SlotOpening_mm      Slot-opening width b_as       (default 2 mm)
%     MuRec               Recoil permeability           (default 1.05)
%     kBr_pctPerC         Br temperature coefficient    (default -0.10 %/degC)
%     PMTemp_C            PM operating temperature      (default 140 degC)
%     WindingTemp_C       Winding reference temperature (default 180 degC)
%     Bg1_T               Airgap fund. flux density     (default 0.95 T)
%     Bt_T                Tooth flux density target     (default 1.80 T)
%     Bc_T                Back-iron flux density target (default 1.30 T)
%     LinearCurrentDensity_Am   Peak delta [A/m]        (default 90e3)
%     CurrentDensity_Amm2       J_rms [A/mm^2]          (default 8.0)
%     CopperFillFactor    k_cu                          (default 0.40)
%     StackingFactor      k_st = le/li                  (default 0.97)
%     IronFillFactor      k_is                          (default 0.97)
%     AspectRatio         le/tau_p (Esson shape factor) (default 1.0)
%     EfficiencyEstimate  eta, used in Esson volume eq. (default 0.95)
%     PowerFactor         cos(phi) estimate             (default 0.90)
%
%   Read-only dependent properties (computed, never set by user)
%   ------------------------------------------------------------
%     SlotsPerPolePerPhase  q = Slots / (Phases * Poles)
%     CornerFrequency_Hz  f_c = CornerSpeed_rpm * Poles / 120
%     MaxFrequency_Hz     f_M = MaxSpeed_rpm   * Poles / 120
%     FluxWeakeningRatio  N_M / N_c
%     Br_T                Br at PMTemp_C (temperature-corrected)
%
%   Example
%   -------
%     spec = MotorSpec(200, 2900, 13500, 650, 8, 60, 1, 1.37, ...
%                      PMTemp_C=140, Bg1_T=0.95);
%     spec.summary()
%
%   See also: IPMRotorSizer, EssonSizer, MotorGeometry

    % =====================================================================
    % Required properties — no default, SetAccess locked after construction
    % =====================================================================
    properties (SetAccess = private)

        % Operating point
        Torque_Nm           (1,1) double
        CornerSpeed_rpm     (1,1) double
        MaxSpeed_rpm        (1,1) double
        VdcLink_V           (1,1) double

        % Topology
        Poles               (1,1) double
        Slots               (1,1) double

        % Key geometry
        Airgap_mm           (1,1) double

        % PM material (required)
        Br20_T              (1,1) double

    end

    % =====================================================================
    % Optional properties — have defaults, user may override via Name=Value
    % =====================================================================
    properties

        % Topology
        Phases              (1,1) double {mustBeMember(Phases, [1 3 5])} = 3

        % Geometry
        SlotOpening_mm      (1,1) double {mustBePositive} = 2.0   % b_as [mm]

        % PM material
        MuRec               (1,1) double {mustBePositive}         = 1.05
        kBr_pctPerC         (1,1) double                          = -0.10
        PMTemp_C            (1,1) double                          = 140
        WindingTemp_C       (1,1) double                          = 180

        % Flux density targets
        Bg1_T               (1,1) double {mustBeInRange(Bg1_T,    0.5, 1.5)} = 0.95
        Bt_T                (1,1) double {mustBeInRange(Bt_T,     1.0, 2.5)} = 1.80
        Bc_T                (1,1) double {mustBeInRange(Bc_T,     0.5, 2.0)} = 1.30

        % Electrical loading
        LinearCurrentDensity_Am  (1,1) double {mustBePositive}   = 90e3
        CurrentDensity_Amm2      (1,1) double {mustBePositive}   = 8.0
        CopperFillFactor         (1,1) double {mustBeInRange(...
                                   CopperFillFactor, 0.2, 0.8)}  = 0.40

        % Lamination & process
        StackingFactor      (1,1) double {mustBeInRange(...
                                   StackingFactor, 0.85, 1.0)}   = 0.97
        IronFillFactor      (1,1) double {mustBeInRange(...
                                   IronFillFactor, 0.85, 1.0)}   = 0.97
        AspectRatio         (1,1) double {mustBePositive}         = 1.0
        EfficiencyEstimate  (1,1) double {mustBeInRange(...
                                   EfficiencyEstimate, 0.5, 1.0)} = 0.95
        PowerFactor         (1,1) double {mustBeInRange(...
                                   PowerFactor, 0.5, 1.0)}        = 0.90

    end

    % =====================================================================
    % Dependent properties — computed from other properties, never stored
    % =====================================================================
    properties (Dependent, SetAccess = private)

        SlotsPerPolePerPhase    % q = Slots / (Phases * Poles)
        CornerFrequency_Hz      % f_c = CornerSpeed_rpm * Poles / 120
        MaxFrequency_Hz         % f_M = MaxSpeed_rpm   * Poles / 120
        FluxWeakeningRatio      % N_M / N_c
        Br_T                    % Br corrected to PMTemp_C

    end

    % =====================================================================
    % Constructor
    % =====================================================================
    methods

        function obj = MotorSpec(torque_Nm, cornerSpeed_rpm, maxSpeed_rpm, ...
                                 vdcLink_V, poles, slots,   ...
                                 airgap_mm, br20_T, options)
            % MotorSpec  Construct a validated motor specification.
            arguments
                torque_Nm            (1,1) double {mustBePositive}
                cornerSpeed_rpm      (1,1) double {mustBePositive}
                maxSpeed_rpm         (1,1) double {mustBePositive}
                vdcLink_V            (1,1) double {mustBePositive}
                poles                (1,1) double {mustBePositive,...
                                                   mustBeInteger,...
                                                   MotorSpec.mustBeEven}
                slots                (1,1) double {mustBePositive,...
                                                   mustBeInteger}
                airgap_mm            (1,1) double {mustBePositive}
                br20_T               (1,1) double {mustBePositive}

                % --- optional name-value ---
                options.Phases               (1,1) double = 3
                options.SlotOpening_mm       (1,1) double = 2.0
                options.MuRec                (1,1) double = 1.05
                options.kBr_pctPerC          (1,1) double = -0.10
                options.PMTemp_C             (1,1) double = 140
                options.WindingTemp_C        (1,1) double = 180
                options.Bg1_T                (1,1) double = 0.95
                options.Bt_T                 (1,1) double = 1.80
                options.Bc_T                 (1,1) double = 1.30
                options.LinearCurrentDensity_Am (1,1) double = 90e3
                options.CurrentDensity_Amm2  (1,1) double = 8.0
                options.CopperFillFactor     (1,1) double = 0.40
                options.StackingFactor       (1,1) double = 0.97
                options.IronFillFactor       (1,1) double = 0.97
                options.AspectRatio          (1,1) double = 1.0
                options.EfficiencyEstimate   (1,1) double = 0.95
                options.PowerFactor          (1,1) double = 0.90
            end

            % --- assign required ---
            obj.Torque_Nm            = torque_Nm;
            obj.CornerSpeed_rpm      = cornerSpeed_rpm;
            obj.MaxSpeed_rpm         = maxSpeed_rpm;
            obj.VdcLink_V            = vdcLink_V;
            obj.Poles                = poles;
            obj.Slots                = slots;
            obj.Airgap_mm            = airgap_mm;
            obj.Br20_T               = br20_T;

            % --- assign optional (from options struct) ---
            obj.Phases                   = options.Phases;
            obj.SlotOpening_mm           = options.SlotOpening_mm;
            obj.MuRec                    = options.MuRec;
            obj.kBr_pctPerC              = options.kBr_pctPerC;
            obj.PMTemp_C                 = options.PMTemp_C;
            obj.WindingTemp_C            = options.WindingTemp_C;
            obj.Bg1_T                    = options.Bg1_T;
            obj.Bt_T                     = options.Bt_T;
            obj.Bc_T                     = options.Bc_T;
            obj.LinearCurrentDensity_Am  = options.LinearCurrentDensity_Am;
            obj.CurrentDensity_Amm2      = options.CurrentDensity_Amm2;
            obj.CopperFillFactor         = options.CopperFillFactor;
            obj.StackingFactor           = options.StackingFactor;
            obj.IronFillFactor           = options.IronFillFactor;
            obj.AspectRatio              = options.AspectRatio;
            obj.EfficiencyEstimate       = options.EfficiencyEstimate;
            obj.PowerFactor              = options.PowerFactor;

            % --- cross-property validation (can't be done per-property) ---
            if maxSpeed_rpm <= cornerSpeed_rpm
                error('MotorSpec:invalidSpeed', ...
                    'MaxSpeed_rpm (%g) must be greater than CornerSpeed_rpm (%g).', ...
                    maxSpeed_rpm, cornerSpeed_rpm);
            end

            % Validate slots-pole-phase feasibility
            % p = pole pairs (poles / 2)
            % t = machine periodicity = GCD(slots, pole pairs)
            p = obj.Poles / 2;
            t = gcd(obj.Slots, p);
            feasibilityRatio = obj.Slots / (obj.Phases * t);

            % 1. Hard Error: Is the winding physically possible to balance?
            if mod(feasibilityRatio, 1) ~= 0
                error('MotorSpec:invalidWinding', ...
                    ['The combination of %d slots, %d poles, and %d phases is not ' ...
                     'physically feasible for a balanced winding. \n' ...
                     'Mathematical rule: Q / (m * t) must be an integer. Result was: %g'], ...
                    obj.Slots, obj.Poles, obj.Phases, feasibilityRatio);
            end

            % 2. Warning: Check for diametric symmetry (Unbalanced Magnetic Pull)
            % If GCD(slots, total poles) == 1, the motor is completely asymmetrical.
            if gcd(obj.Slots, obj.Poles) == 1
                warning('MotorSpec:highUMPRisk', ...
                    ['The GCD of slots (%d) and total poles (%d) is 1. \n' ...
                     'This topology lacks diametric symmetry and will generate severe ' ...
                     'Unbalanced Magnetic Pull (UMP) at high speeds, risking bearing failure.'], ...
                    obj.Slots, obj.Poles);
            end
        end

    end

    % =====================================================================
    % Dependent property getters
    % =====================================================================
    methods

        function q = get.SlotsPerPolePerPhase(obj)
            q = obj.Slots / (obj.Phases * obj.Poles);
        end

        function f = get.CornerFrequency_Hz(obj)
            f = obj.CornerSpeed_rpm * obj.Poles / 120;
        end

        function f = get.MaxFrequency_Hz(obj)
            f = obj.MaxSpeed_rpm * obj.Poles / 120;
        end

        function r = get.FluxWeakeningRatio(obj)
            r = obj.MaxSpeed_rpm / obj.CornerSpeed_rpm;
        end

        function Br = get.Br_T(obj)
            % Temperature-corrected remanent flux density (linear model).
            Br = obj.Br20_T * (1 + obj.kBr_pctPerC/100 * (obj.PMTemp_C - 20));
        end

    end

    % =====================================================================
    % Public methods
    % =====================================================================
    methods (Access = public)

        function summary(obj)
            % summary  Print a formatted overview of the specification.
            fprintf('\n');
            fprintf('MotorSpec summary\n');
            fprintf('%s\n', repmat('-', 1, 52));

            fprintf('  %-30s %g Nm\n',  'Corner torque:',       obj.Torque_Nm);
            fprintf('  %-30s %g / %g rpm\n', 'Speed (corner / max):', ...
                    obj.CornerSpeed_rpm, obj.MaxSpeed_rpm);
            fprintf('  %-30s %.1f  (N_M/N_c)\n', 'Flux-weakening ratio:', ...
                    obj.FluxWeakeningRatio);
            fprintf('  %-30s %g V\n',   'DC-link voltage:',     obj.VdcLink_V);
            fprintf('\n');
            fprintf('  %-30s %d\n',     'Poles:',               obj.Poles);
            fprintf('  %-30s %g\n',     'Slots/pole/phase (q):', obj.SlotsPerPolePerPhase);
            fprintf('  %-30s %d\n',     'Total slots (Q):',     obj.Slots);
            fprintf('  %-30s %d\n',     'Phases:',              obj.Phases);
            fprintf('\n');
            fprintf('  %-30s %g mm\n',  'Air gap g:',           obj.Airgap_mm);
            fprintf('  %-30s %g mm\n',  'Slot opening b_as:',   obj.SlotOpening_mm);
            fprintf('\n');
            fprintf('  %-30s %.3f T  (at %g degC)\n', 'Br (corrected):', ...
                    obj.Br_T, obj.PMTemp_C);
            fprintf('  %-30s %.3f T  (at 20 degC)\n', 'Br20:', obj.Br20_T);
            fprintf('  %-30s %.2f\n',   'Recoil permeability:',  obj.MuRec);
            fprintf('\n');
            fprintf('  %-30s %.2f T\n', 'Target Bg1:',           obj.Bg1_T);
            fprintf('  %-30s %.2f T\n', 'Target Bt (tooth):',    obj.Bt_T);
            fprintf('  %-30s %.2f T\n', 'Target Bc (yoke):',     obj.Bc_T);
            fprintf('\n');
            fprintf('  %-30s %g A/m\n', 'Linear current density:', ...
                    obj.LinearCurrentDensity_Am);
            fprintf('  %-30s %g A/mm^2\n', 'Current density J_rms:', ...
                    obj.CurrentDensity_Amm2);
            fprintf('  %-30s %.2f\n',   'Copper fill factor:',   obj.CopperFillFactor);
            fprintf('%s\n\n', repmat('-', 1, 52));
        end

        function s = toStruct(obj)
            % toStruct  Export all properties to a plain struct.
            %   Useful for serialisation, logging, or passing to
            %   legacy function-based code.
            s = struct();

            % required
            s.Torque_Nm            = obj.Torque_Nm;
            s.CornerSpeed_rpm      = obj.CornerSpeed_rpm;
            s.MaxSpeed_rpm         = obj.MaxSpeed_rpm;
            s.VdcLink_V            = obj.VdcLink_V;
            s.Poles                = obj.Poles;
            s.Slots                = obj.Slots;
            s.Airgap_mm            = obj.Airgap_mm;
            s.Br20_T               = obj.Br20_T;

            % optional
            s.Phases                   = obj.Phases;
            s.SlotOpening_mm           = obj.SlotOpening_mm;
            s.MuRec                    = obj.MuRec;
            s.kBr_pctPerC              = obj.kBr_pctPerC;
            s.PMTemp_C                 = obj.PMTemp_C;
            s.WindingTemp_C            = obj.WindingTemp_C;
            s.Bg1_T                    = obj.Bg1_T;
            s.Bt_T                     = obj.Bt_T;
            s.Bc_T                     = obj.Bc_T;
            s.LinearCurrentDensity_Am  = obj.LinearCurrentDensity_Am;
            s.CurrentDensity_Amm2      = obj.CurrentDensity_Amm2;
            s.CopperFillFactor         = obj.CopperFillFactor;
            s.StackingFactor           = obj.StackingFactor;
            s.IronFillFactor           = obj.IronFillFactor;
            s.AspectRatio              = obj.AspectRatio;
            s.EfficiencyEstimate       = obj.EfficiencyEstimate;
            s.PowerFactor              = obj.PowerFactor;

            % dependent (computed)
            s.SlotsPerPolePerPhase = obj.SlotsPerPolePerPhase;
            s.CornerFrequency_Hz   = obj.CornerFrequency_Hz;
            s.MaxFrequency_Hz      = obj.MaxFrequency_Hz;
            s.FluxWeakeningRatio   = obj.FluxWeakeningRatio;
            s.Br_T                 = obj.Br_T;
        end

    end

    % =====================================================================
    % Private static validators
    %   MATLAB property validators must be static methods so they can be
    %   called before the object is fully constructed.
    % =====================================================================
    methods (Static, Access = private)

        function mustBeEven(val)
            % mustBeEven  Validator: value must be divisible by 2.
            if mod(val, 2) ~= 0
                error('MotorSpec:notEven', ...
                    'Value must be an even integer; got %g.', val);
            end
        end

    end

end