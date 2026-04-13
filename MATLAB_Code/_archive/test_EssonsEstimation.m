EssonsTest();

function EssonsTest()
    % ==========================================
    % 1. Requirements & Constraints Definition
    % ==========================================
    windingHeads_m = 0.012 + 0.0128; % [m] Length of winding heads.
    coolingJacket_m = 0.0; % [m] Assume 0 until we find the actual value.

    % Max chassis constraints
    L_MAX = 110e-3 - windingHeads_m; % [m] Active length available
    D_ROTOR_MAX = 80e-3; % [m] Physical rotor limit
    D_STATOR_MAX = 120e-3 - (2 * coolingJacket_m); % [m] Max stator OD

    % Targets
    TORQUE_MAX = 7; % [Nm] Changed to 18 Nm based on your clipping requirement!
    AIR_GAP = 0.7e-3; % [m] Assumption!
    C_0 = 50000; % [W*s/m^3] Output Coefficient
    K = L_MAX / D_ROTOR_MAX; % Aspect Ratio

    % ==========================================
    % 2. Motor Core Sizing (Essen's Rule)
    % ==========================================
    % D is the Stator Bore (Airgap) Diameter
    D = nthroot((TORQUE_MAX/(C_0*K)), 3); % [m]
    L = K * D; % [m] Active Stack Length

    % Actual physical rotor outer diameter
    D_ROTOR_ACTUAL = D - (2 * AIR_GAP); % [m]

    % ==========================================
    % 3. Stator Dimensioning (C. Ricca Paper)
    % ==========================================
    % In Phase 2, these will be replaced with equations based on Current and Flux
    h_as = 0.001; % [m] Tooth tip height (Estimated at 1 mm)
    h_ts = 0.015; % [m] Slot depth (Estimated at 15 mm to hold 107A of copper)
    h_sy = 0.009; % [m] Stator yoke thickness (Estimated at 9 mm to carry flux)

    % External Stator Diameter calculation
    D_ES = D + 2 * (h_as + h_ts + h_sy); % [m]

    % Display the results
    disp('--- Initial Motor Sizing ---');
    fprintf('Calculated Rotor OD: %.2f mm\n', D_ROTOR_ACTUAL * 1000);
    fprintf('Calculated Stator OD (D_ES): %.2f mm\n', D_ES * 1000);
    fprintf('Calculated Stack Length: %.2f mm\n\n', L * 1000);

    % ==========================================
    % 4. Validation Stage (Assertions)
    % ==========================================
    % Check 1: Does the Rotor fit?
    assert(D_ROTOR_ACTUAL <= D_ROTOR_MAX, ...
        'DESIGN FAILED: Calculated Rotor diameter (%.2f mm) exceeds the limit (%.2f mm).', ...
        D_ROTOR_ACTUAL * 1000, D_ROTOR_MAX * 1000);

    % Check 2: Does the Length fit?
    assert(L <= L_MAX, ...
        'DESIGN FAILED: Calculated stack length (%.2f mm) exceeds active limit (%.2f mm).', ...
        L * 1000, L_MAX * 1000);

    % Check 3: Does the Stator fit inside the cooling jacket?
    assert(D_ES <= D_STATOR_MAX, ...
        'DESIGN FAILED: Calculated Stator OD (%.2f mm) exceeds chassis limit (%.2f mm). Decrease slot depth, yoke thickness, or increase C_0.', ...
        D_ES * 1000, D_STATOR_MAX * 1000);

    disp('TEST PASSED: All Rotor and Stator dimensions meet the packaging constraints.');

    % Call the EssonsEstimation function (use positional arguments)
    [Dis,le,tau_p,tau_s,t_s,h_slot,h_cs,Dro,Dso,ratio,q_spp] = EssonsEstimation( ...
        TORQUE_MAX, 10, 12, 3, 0.9, 1.8, 1.8, 90000, 5, K, 0.95, 0.95, AIR_GAP, 1.05, 0.95, 0.5e-3, 0.45);

    % Print only the variables returned by the function (avoid internal-only names)
    fprintf('Dis = %.2f mm\n', Dis*1000);
    fprintf('Dso = %.2f mm\n', Dso*1000);
    fprintf('le = %.2f mm\n', le*1000);
    fprintf('tau_p = %.2f mm\n', tau_p*1000);
    fprintf('tau_s = %.2f mm\n', tau_s*1000);
    fprintf('t_s = %.2f mm\n', t_s*1000);
    fprintf('h_slot = %.2f mm\n', h_slot*1000);
    fprintf('h_cs = %.2f mm\n', h_cs*1000);
    fprintf('Dro = %.2f mm\n', Dro*1000);
    fprintf('ratio = %.2f\n', ratio);
    fprintf('q_spp = %.2f\n', q_spp);
end