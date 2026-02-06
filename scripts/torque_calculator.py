import math

def calculate_torque(femur_cm, tibia_cm, robot_mass_kg, stride_length_cm=8.0, servo_stall_torque_kgcm=10.0, per_leg_mass_g=150.0, forced_reach=None):
    """
    Calculates torque requirements and reach for a given hexapod configuration.
    
    Args:
        femur_cm: Length of femur in cm
        tibia_cm: Length of tibia in cm
        robot_mass_kg: Total mass of robot body + electronics + batteries + motors
        stride_length_cm: Dynamic stride range (defaults to 8cm Max Stride)
        forced_reach: Optional override for standing reach (to match Hexapod.h)
        servo_stall_torque_kgcm: Stall torque of servo (default MG996R ~10-11kgcm)
        per_leg_mass_g: Mass of one leg assembly (approximated)
    """
    
    # 1. Reach Calculations
    # Approx vertical reach (fully extended down vs tucked up)
    # Assumes hip has some clearance.
    max_vertical_reach = femur_cm + tibia_cm
    
    # 2. Torque Context
    # Worst case leverage: Leg extended horizontally.
    # But usually we stand with tibia angled.
    # Let's assess the "Standing Reach" - horizontal distance from hip pivot to foot tip.
    # If Femur is horizontal and Tibia is 45 deg down:
    # Horizontal Reach = Femur + Tibia * cos(45)
    # But worst case torque is when fully extended horizontally (if physically possible)
    
    max_horizontal_reach = femur_cm + tibia_cm
    
    # 3. Load Calculations
    # Tripod Gait: 3 legs support the robot.
    load_per_leg_tripod = robot_mass_kg / 3.0
    
    # Dynamic/Worst Case: 2 legs support (during fast transition) or 1 leg taking impact.
    load_per_leg_heavy = robot_mass_kg / 2.0
    
    # Torque = Force * Distance
    # Force = Mass * Gravity? (We work in kg-cm for servos usually, simplifying g)
    
    # Torque at Femur joint (lifts the robot)
    # Worst case: Fully extended horizontally
    torque_tripod_max_reach = load_per_leg_tripod * max_horizontal_reach
    torque_heavy_max_reach = load_per_leg_heavy * max_horizontal_reach
    
    # Realistic Standing Pose (e.g., Tibia angled in slightly, or legs not fully spread)
    # Let's assume a "Standard" horizontal reach of (Femur + 0.5 * Tibia)
    # This represents a typical walking pose where feet aren't at max extension
    if forced_reach:
        typical_reach = forced_reach
    else:
        typical_reach = femur_cm + (tibia_cm * 0.5) 
    torque_tripod_typical = load_per_leg_tripod * typical_reach
    
    # 4a. Dynamic Stride Analysis
    # During a step, the foot moves forward/back by stride_length/2.
    # We must assume the worst case: The leg is fully extended at the end of the stride.
    # However, stride is tangential (Y-axis), while Reach is radial (X-axis).
    # Peak Distance = sqrt(Stand_Reach^2 + (Stride/2)^2)
    # Let's use the provided 'average' stance reach for this base.
    # But usually we define Reach as the radial distance.
    # Let's assume 'typical_reach' is our Stand Reach.
    
    stride_reach_peak = math.sqrt(typical_reach**2 + (stride_length_cm / 2)**2)
    
    # Check if this peak reach exceeds the physical leg length
    is_stride_possible = stride_reach_peak < max_horizontal_reach
    
    # Torque at Peak Stride
    torque_tripod_peak_stride = load_per_leg_tripod * stride_reach_peak
    
    
    # 4. Athleticism Score
    # We want operating torque to be < 30% of stall for "Athletic" movement.
    # < 50% for "Walking"
    # > 50% for "Struggling/Overheating"
    
    safety_margin_typical = servo_stall_torque_kgcm / torque_tripod_typical
    safety_margin_peak = servo_stall_torque_kgcm / torque_tripod_peak_stride
    
    return {
        "reach_v": max_vertical_reach,
        "reach_h_max": max_horizontal_reach,
        "reach_h_typical": typical_reach,
        "reach_peak_stride": stride_reach_peak,
        "torque_tripod_typical": torque_tripod_typical,
        "torque_tripod_peak": torque_tripod_peak_stride,
        "safety_margin": safety_margin_typical,
        "safety_margin_peak": safety_margin_peak,
        "can_climb_stairs": max_vertical_reach > 18.0, # Standard stair is ~18cm
        "stride_possible": is_stride_possible
    }

def print_scenario(name, f, t, mass, stride=8.0, reach=None):
    res = calculate_torque(f, t, mass, stride, forced_reach=reach)
    print(f"--- {name} (Stride {stride}cm) ---")
    print(f"Legs: F={f}cm, T={t}cm | Mass: {mass}kg")
    print(f"Vertical Reach: {res['reach_v']:.1f} cm")
    print(f"Standing Reach: {res['reach_h_typical']:.1f} cm")
    print(f"Peak Stride Reach: {res['reach_peak_stride']:.1f} cm (Possible: {res['stride_possible']})")
    print(f"Torque (Stand): {res['torque_tripod_typical']:.2f} kg-cm")
    print(f"Torque (Peak):  {res['torque_tripod_peak']:.2f} kg-cm")
    print(f"Safety Factor (Stand): {res['safety_margin']:.2f}x")
    print(f"Safety Factor (Peak):  {res['safety_margin_peak']:.2f}x")
    
    if not res['stride_possible']:
         print("Verdict: IMPOSSIBLE (Legs too short for this stride)")
    elif res['safety_margin_peak'] > 2.0:
        print("Verdict: ATHLETIC (Excellent)")
    elif res['safety_margin_peak'] > 1.5:
        print("Verdict: GOOD (Safe)")
    elif res['safety_margin_peak'] > 1.0:
        print("Verdict: MARGINAL (Low Safety Factor)")
    else:
        print("Verdict: FAIL (Stall Likely)")
    print("")

print("--- Hexapod Design Analysis (MG996R @ 6V = 10kg-cm) ---\n")

# Scenario 1: Actual Implementation
print_scenario("Actual Implementation (Reach 11cm)", 5.0, 8.0, 2.0, stride=8.0, reach=11.0)
print_scenario("Actual Implementation (Reach 11cm, Stride 4cm)", 5.0, 8.0, 2.0, stride=4.0, reach=11.0)

# Scenario 2: Compact Athletic (Shortest)
print_scenario("Compact Athletic", 5.0, 8.0, 2.0)

# Scenario 3: Balanced Walker
print_scenario("Balanced Walker", 6.0, 9.0, 2.0)

# Scenario 4: Stock PhantomX Style
print_scenario("Stock Kit Style", 5.2, 8.2, 2.0)
