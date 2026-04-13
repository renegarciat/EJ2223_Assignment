import numpy as np

# --- 1. DESIGN CONSTRAINTS & TARGETS ---
V_dc_nom = 515.0        # Nominal Bus Voltage [V]
I_max_inv = 107.0       # Inverter Peak Phase Current [A]
T_target = 18.0         # Target Peak Torque [Nm] [cite: 20, 24]
P_target = 28000.0      # Target Peak Power [W] [cite: 38]
N_max = 12000.0         # Max Operating Speed [RPM] [cite: 35, 44]
# TODO: Reduce the number of poles to reduce iron losses at high speed, but this may require a larger diameter to maintain torque density.
pole_pairs = 5          # 10 poles total 

# --- 2. ASSUMPTIONS FOR FIRST-ORDER SIZING ---
# Typical "Specific Shear Stress" (sigma) for liquid-cooled FS motors: 35,000 - 55,000 Pa
sigma = 45000.0         
# Aspect Ratio (L/D): Typical values 0.5 to 1.5. 
# Smaller diameter = higher L/D.
aspect_ratio = 1.2      
efficiency_est = 0.92   
power_factor = 0.90     

def size_motor():
    # A. Calculate Required Rotor Volume (V_r)
    # Equation: T = 2 * sigma * V_rotor
    v_rotor = T_target / (2 * sigma) # [m^3]
    
    # B. Derive Dimensions from Aspect Ratio (L = lambda * D)
    # V = pi * (D/2)^2 * L  => V = pi * (D^2 / 4) * (lambda * D)
    # D = cuberoot( (4 * V) / (pi * lambda) )
    d_rotor = ((4 * v_rotor) / (np.pi * aspect_ratio))**(1/3)
    l_stack = d_rotor * aspect_ratio
    
    # C. Electrical Check (Back-EMF & Base Speed)
    # V_phase_max = V_dc / sqrt(3)
    v_phase_max = V_dc_nom / np.sqrt(3)
    
    # Estimate base speed (where we hit voltage limit)
    # We want this earlier (lower RPM) to force field weakening
    omega_max = (N_max * 2 * np.pi) / 60
    
    # D. Required Flux Linkage (Psi_m)
    # For IPM, T approx = 1.5 * p * Psi_m * Iq
    # Solving for Psi_m assuming Iq is most of the current at base speed
    psi_m = T_target / (1.5 * pole_pairs * (I_max_inv * 0.8))

    return {
        "Rotor Diameter (mm)": d_rotor * 1000,
        "Stack Length (mm)": l_stack * 1000,
        "Rotor Volume (cm3)": v_rotor * 1e6,
        "Max Phase Voltage (V_peak)": v_phase_max,
        "Estimated Flux Linkage (Wb)": psi_m
    }

results = size_motor()

print("--- NEW MOTOR PRELIMINARY SIZING ---")
for key, value in results.items():
    print(f"{key}: {value:.2f}")