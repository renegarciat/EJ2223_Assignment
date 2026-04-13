% Requirements:
T = 17; % Torque target (Nm)
D = 0.08; % Rotor outer diameter (m)
C0 = 45000; % Target Output Coefficient (Ws/m^3)
L = T / (C0 * D^2); % Calculate the required stack length (m)
L = L + 0.0238; % Add winding head length (m)
% NOTE: We know that C0 = B * K_s, where B is the magnetic flux density and K_s is the armature current loading.
fprintf('Torque (T): %.2f Nm\n', T);
fprintf('Rotor Outer Diameter (D): %.2f m\n', D);
fprintf('Output Coefficient (C0): %.2f Ws/m^3\n', C0);
fprintf('Calculated Stack Length (L): %.4f m\n', L);

assert_requirements(T, D, C0, L);
w_m = 0.01; % Magnetic Width (m). Length of the magnet "slab"
t_m = 0.005; % Magnet Thickness (m). Depth of the magnet
alpha_v = 0.5; % V angle. angle between the two magnet slabs.
b_t = 0.002; % Bridge Thickness (m). Thin piece of iron between the magnet and the outer edge of the rotor.



function assert_requirements(T, D, C0, L)
    D_MAX = 0.08; % Maximum rotor outer diameter (m)
    L_MAX = 0.061 + 0.0124 + 0.0118; % Maximum stack length (m)
    assert(L <= L_MAX, 'The required stack length (%.4f m) exceeds the maximum allowed length: %.4f m.', L, L_MAX);
    assert(D <= D_MAX, 'The required outer diameter (%.4f m) exceeds the maximum allowed diameter: %.4f m.', D, D_MAX);
end