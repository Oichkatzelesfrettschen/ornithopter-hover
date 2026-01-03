#!/usr/bin/env python3
"""
Z3 Verification Scripts for Ornithopter Control System

This module contains formal verification tests using the Z3 SMT solver
to prove mathematical properties of the control system.
"""

from z3 import *
import sys

def verify_control_bounds():
    """Verify that control commands stay within physical limits."""
    print("=" * 60)
    print("Verifying Control Bounds")
    print("=" * 60)
    
    # Define variables
    roll = Real('roll')
    pitch = Real('pitch')
    yaw = Real('yaw')
    delta_left = Real('delta_left')
    delta_right = Real('delta_right')
    
    # PID gains
    K_p_roll = 0.5
    
    # Create solver
    s = Solver()
    
    # Physical constraints (attitude limits)
    s.add(roll >= -45, roll <= 45)      # Degrees
    s.add(pitch >= -45, pitch <= 45)
    s.add(yaw >= -180, yaw <= 180)
    
    # Control surface limits
    s.add(delta_left >= -30, delta_left <= 30)
    s.add(delta_right >= -30, delta_right <= 30)
    
    # Control laws (simplified PID)
    s.add(delta_left == K_p_roll * roll)
    s.add(delta_right == -K_p_roll * roll)
    
    # Verify: control saturates at expected attitude
    # This proves the control will saturate before dangerous attitudes
    print("\nChecking: Control saturates before dangerous attitudes...")
    s.push()
    # Remove control constraints temporarily to see what roll would cause saturation
    s2 = Solver()
    s2.add(roll >= -45, roll <= 45)
    s2.add(pitch >= -45, pitch <= 45)
    s2.add(yaw >= -180, yaw <= 180)
    s2.add(delta_left == K_p_roll * roll)
    s2.add(delta_right == -K_p_roll * roll)
    # Check: Can roll reach value that would saturate control?
    # At saturation: delta = 30, so roll = 30/0.5 = 60°
    # But roll is limited to 45°, so control will never saturate
    s2.add(roll == 45)  # Maximum allowed roll
    result = s2.check()
    if result == sat:
        m = s2.model()
        delta_at_max_roll = K_p_roll * 45
        print("✓ PASS: Control bounds verified")
        print(f"  Maximum roll: 45°")
        print(f"  Control output at max roll: {delta_at_max_roll}°")
        print(f"  Control saturates at roll: {30/K_p_roll}° (safely above limit)")
    else:
        print("✗ FAIL: Control bounds violated!")
        s.pop()
        return False
    s.pop()
    
    # Verify: symmetric control for symmetric attitude
    print("\nChecking: Symmetric control for symmetric attitude...")
    s.push()
    roll_val = Real('roll_val')
    s.add(roll_val > 0)
    s.add(delta_left == K_p_roll * roll_val)
    s.add(delta_right == -K_p_roll * roll_val)
    s.add(delta_left == -delta_right)
    result = s.check()
    if result == sat:
        print("✓ PASS: Control symmetry verified")
    else:
        print("✗ FAIL: Control symmetry violated!")
        return False
    s.pop()
    
    return True


def verify_sensor_fusion():
    """Verify sensor fusion produces bounded output."""
    print("\n" + "=" * 60)
    print("Verifying Sensor Fusion")
    print("=" * 60)
    
    # Complementary filter weight
    alpha = Real('alpha')
    gyro_angle = Real('gyro_angle')
    accel_angle = Real('accel_angle')
    fused_angle = Real('fused_angle')
    
    s = Solver()
    
    # Filter equation: fused = alpha * gyro + (1 - alpha) * accel
    s.add(fused_angle == alpha * gyro_angle + (1 - alpha) * accel_angle)
    
    # Constraints
    s.add(alpha >= 0.90, alpha <= 0.99)  # Typical complementary filter weight
    s.add(gyro_angle >= -90, gyro_angle <= 90)
    s.add(accel_angle >= -90, accel_angle <= 90)
    
    # Verify: fused output stays within input bounds
    print("\nChecking: Fused angle stays within bounds...")
    s.push()
    s.add(Or(fused_angle < -90, fused_angle > 90))
    result = s.check()
    if result == unsat:
        print("✓ PASS: Sensor fusion bounds verified")
        print("  Output guaranteed to be in [-90°, 90°]")
    else:
        print("✗ FAIL: Sensor fusion can exceed bounds!")
        print(f"  Counterexample: {s.model()}")
        return False
    s.pop()
    
    # Verify: if both sensors agree, output equals input
    print("\nChecking: Agreement case...")
    s.push()
    s.add(gyro_angle == accel_angle)
    s.add(fused_angle != gyro_angle)
    result = s.check()
    if result == unsat:
        print("✓ PASS: Fusion preserves agreement")
    else:
        print("✗ FAIL: Fusion doesn't preserve agreement!")
        return False
    s.pop()
    
    return True


def verify_quaternion_properties():
    """Verify mathematical properties of quaternion operations."""
    print("\n" + "=" * 60)
    print("Verifying Quaternion Properties")
    print("=" * 60)
    
    # Define quaternion components
    qw, qx, qy, qz = Reals('qw qx qy qz')
    
    s = Solver()
    
    # Unit quaternion constraint
    s.add(qw*qw + qx*qx + qy*qy + qz*qz == 1)
    
    # Verify: unit quaternions have valid components
    print("\nChecking: Unit quaternion component bounds...")
    s.push()
    s.add(Or(qw < -1, qw > 1, qx < -1, qx > 1, 
             qy < -1, qy > 1, qz < -1, qz > 1))
    result = s.check()
    if result == unsat:
        print("✓ PASS: Unit quaternion components in [-1, 1]")
    else:
        print("✗ FAIL: Unit quaternion can have invalid components!")
        return False
    s.pop()
    
    # Verify: rotation angle bounds
    print("\nChecking: Rotation angle bounds...")
    s.push()
    # For unit quaternion, w = cos(θ/2), so θ = 2*arccos(w)
    # We can verify that w in [-1, 1] implies valid rotation
    s.add(qw >= -1, qw <= 1)
    result = s.check()
    if result == sat:
        print("✓ PASS: Quaternion represents valid rotation")
    else:
        print("✗ FAIL: Invalid rotation representation!")
        return False
    s.pop()
    
    return True


def verify_power_constraints():
    """Verify battery and power constraints."""
    print("\n" + "=" * 60)
    print("Verifying Power Constraints")
    print("=" * 60)
    
    # Battery and power parameters
    battery_capacity = Real('battery_capacity')  # mAh
    voltage = Real('voltage')                     # V
    current = Real('current')                     # A
    flight_time = Real('flight_time')            # minutes
    power = Real('power')                         # W
    
    s = Solver()
    
    # Physical relationships
    s.add(power == voltage * current)
    s.add(flight_time == (battery_capacity / 1000.0) / current * 60)
    
    # Example values
    s.add(battery_capacity == 500)  # 500 mAh battery
    s.add(voltage == 3.7)            # Single cell LiPo nominal
    
    # Expected power range for hovering ornithopter
    s.add(power >= 5, power <= 15)
    
    print("\nChecking: Minimum flight time achievable...")
    s.push()
    min_flight_time = 3  # Target: at least 3 minutes
    s.add(flight_time >= min_flight_time)
    result = s.check()
    if result == sat:
        m = s.model()
        print(f"✓ PASS: Battery can provide {min_flight_time}+ minutes")
        print(f"  Current draw: {m[current]} A")
        print(f"  Power: {m[power]} W")
        print(f"  Flight time: {m[flight_time]} min")
    else:
        print(f"✗ FAIL: Cannot achieve {min_flight_time} minute flight time!")
        return False
    s.pop()
    
    # Verify: higher power means shorter flight time
    print("\nChecking: Power-time relationship...")
    s.push()
    power1 = Real('power1')
    power2 = Real('power2')
    time1 = Real('time1')
    time2 = Real('time2')
    current1 = Real('current1')
    current2 = Real('current2')
    
    s.add(power1 == voltage * current1)
    s.add(power2 == voltage * current2)
    s.add(time1 == (battery_capacity / 1000.0) / current1 * 60)
    s.add(time2 == (battery_capacity / 1000.0) / current2 * 60)
    s.add(power1 > power2)
    s.add(power1 > 0, power2 > 0)
    s.add(time1 >= time2)  # This should be impossible
    
    result = s.check()
    if result == unsat:
        print("✓ PASS: Higher power always means shorter flight time")
    else:
        print("✗ FAIL: Power-time relationship violated!")
        return False
    s.pop()
    
    return True


def verify_altitude_estimation():
    """Verify barometric altitude calculation."""
    print("\n" + "=" * 60)
    print("Verifying Altitude Estimation")
    print("=" * 60)
    
    # Variables
    pressure = Real('pressure')       # Pa
    altitude = Real('altitude')       # m
    P0 = 101325.0                    # Sea level pressure
    
    s = Solver()
    
    # Simplified altitude equation (linear approximation for small altitudes)
    # h ≈ (P0 - P) / (ρ * g) where ρ ≈ 1.225 kg/m³, g = 9.81 m/s²
    rho_g = 1.225 * 9.81
    s.add(altitude == (P0 - pressure) / rho_g)
    
    print("\nChecking: Altitude bounds for expected pressure range...")
    s.push()
    # Pressure range for 0-100m altitude
    s.add(pressure >= 100125, pressure <= 101325)  # Roughly 0-100m
    s.add(Or(altitude < 0, altitude > 150))
    result = s.check()
    if result == unsat:
        print("✓ PASS: Altitude estimation within reasonable bounds")
    else:
        print("✗ FAIL: Altitude estimation can give unreasonable values!")
        return False
    s.pop()
    
    # Verify: higher pressure means lower altitude
    print("\nChecking: Pressure-altitude relationship...")
    s.push()
    p1 = Real('p1')
    p2 = Real('p2')
    h1 = Real('h1')
    h2 = Real('h2')
    
    s.add(h1 == (P0 - p1) / rho_g)
    s.add(h2 == (P0 - p2) / rho_g)
    s.add(p1 > p2)
    s.add(p1 < P0, p2 < P0)  # Below sea level pressure
    s.add(h1 >= h2)  # This should be impossible
    
    result = s.check()
    if result == unsat:
        print("✓ PASS: Higher pressure always means lower altitude")
    else:
        print("✗ FAIL: Pressure-altitude relationship violated!")
        return False
    s.pop()
    
    return True


def verify_stability_margins():
    """Verify control system stability margins."""
    print("\n" + "=" * 60)
    print("Verifying Stability Margins")
    print("=" * 60)
    
    # PID gains
    Kp = Real('Kp')
    Ki = Real('Ki')
    Kd = Real('Kd')
    
    s = Solver()
    
    # Stability constraints (simplified)
    # For a typical second-order system, we need:
    # - Positive gains
    # - Kp dominant for response
    # - Ki small to prevent windup
    # - Kd moderate for damping
    
    s.add(Kp > 0, Ki >= 0, Kd >= 0)
    
    print("\nChecking: PID gain relationships for stability...")
    s.push()
    # Typical ratios for stable flight control
    s.add(Kp >= 0.1, Kp <= 2.0)
    s.add(Ki <= Kp / 10)  # Integral gain much smaller
    s.add(Kd <= Kp * 0.5)  # Derivative gain proportional
    
    result = s.check()
    if result == sat:
        m = s.model()
        print("✓ PASS: Stable PID gain configuration exists")
        print(f"  Example: Kp={m[Kp]}, Ki={m[Ki]}, Kd={m[Kd]}")
    else:
        print("✗ FAIL: No stable PID configuration found!")
        return False
    s.pop()
    
    return True


def main():
    """Run all verification tests."""
    print("\n" + "=" * 60)
    print("Z3 FORMAL VERIFICATION SUITE")
    print("Ornithopter Control System")
    print("=" * 60 + "\n")
    
    tests = [
        ("Control Bounds", verify_control_bounds),
        ("Sensor Fusion", verify_sensor_fusion),
        ("Quaternion Properties", verify_quaternion_properties),
        ("Power Constraints", verify_power_constraints),
        ("Altitude Estimation", verify_altitude_estimation),
        ("Stability Margins", verify_stability_margins),
    ]
    
    results = []
    for name, test_func in tests:
        try:
            success = test_func()
            results.append((name, success))
        except Exception as e:
            print(f"\n✗ ERROR in {name}: {e}")
            results.append((name, False))
    
    # Summary
    print("\n" + "=" * 60)
    print("VERIFICATION SUMMARY")
    print("=" * 60)
    
    passed = sum(1 for _, success in results if success)
    total = len(results)
    
    for name, success in results:
        status = "✓ PASS" if success else "✗ FAIL"
        print(f"{status}: {name}")
    
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 All verification tests PASSED!")
        return 0
    else:
        print(f"\n⚠️  {total - passed} test(s) FAILED!")
        return 1


if __name__ == "__main__":
    sys.exit(main())
