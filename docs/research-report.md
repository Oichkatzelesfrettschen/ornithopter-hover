# Comprehensive Research and Development Report: Ornithopter Hover System

## Executive Summary

This document provides an exhaustive analysis of the mathematical, physical, and computational foundations required for a hovering ornithopter system. It integrates materials science, fluid mechanics, advanced spatial mathematics, machine learning algorithms, sensor fusion, and formal verification methods to create a robust development framework.

## 1. Mathematical Foundations and Technical Debt Analysis

### 1.1 Lacunae (Knowledge Gaps) Identification

#### Current System Gaps:
1. **Rotational Dynamics**: No implementation of advanced rotation representations (quaternions, octonions)
2. **State Estimation**: Missing Kalman filtering or complementary filters for sensor fusion
3. **Control Theory**: Absence of PID tuning methodology and adaptive control
4. **Aerodynamic Modeling**: No computational fluid dynamics integration
5. **Materials Optimization**: Lack of structural analysis and fatigue modeling

### 1.2 Technical Debt (Debitum Technicum)

#### Architectural Debt:
- **Controller Architecture**: Single-file monolithic design limits scalability
- **Sensor Abstraction**: No hardware abstraction layer for sensor swapping
- **Communication Protocol**: Missing telemetry and debugging infrastructure
- **Testing Infrastructure**: No unit tests, integration tests, or hardware-in-the-loop testing

#### Mathematical Debt:
- **Numerical Stability**: Potential for gimbal lock with Euler angles
- **Precision**: Need for fixed-point arithmetic analysis on embedded systems
- **Algorithm Complexity**: Missing Big-O analysis for real-time constraints

## 2. Rotation Mathematics: Quaternions and Octonions

### 2.1 Quaternion Representation

Quaternions provide a singularity-free representation of 3D rotations.

**Definition**: A quaternion q is defined as:
```
q = w + xi + yj + zk
where i² = j² = k² = ijk = -1
```

**Unit Quaternion for Rotation**:
```
q = [cos(θ/2), sin(θ/2)·n̂]
where θ is rotation angle, n̂ is rotation axis
```

**Key Operations**:

1. **Quaternion Multiplication** (for rotation composition):
```
q₁ ⊗ q₂ = [w₁w₂ - v₁·v₂, w₁v₂ + w₂v₁ + v₁ × v₂]
```

2. **Conjugate** (for inverse rotation):
```
q* = [w, -x, -y, -z]
```

3. **Vector Rotation**:
```
v' = q ⊗ [0, v] ⊗ q*
```

**Advantages for Ornithopter Control**:
- No gimbal lock at 90° pitch
- Compact representation (4 values vs 9 for rotation matrix)
- Efficient interpolation (SLERP)
- Numerically stable for integration

### 2.2 Octonion Extensions

Octonions (8-dimensional hypercomplex numbers) extend quaternions but are non-associative.

**Definition**: 
```
o = a + be₁ + ce₂ + de₃ + ee₄ + fe₅ + ge₆ + he₇
```

**Application in Ornithopter Context**:
- **Extended State Space**: Model coupled translation-rotation dynamics
- **Wing Kinematics**: Represent complex flapping patterns with phase relationships
- **Multi-Body Dynamics**: Handle 4-wing configurations with independent control

**Limitations**:
- Non-associativity requires careful ordering of operations
- Higher computational cost (64 multiplications vs 16 for quaternions)
- Limited embedded implementation libraries

**Recommendation**: Use quaternions for attitude representation; reserve octonions for offline trajectory optimization and advanced multi-wing coordination research.

### 2.3 Spatial Mathematics for Flight Control

#### Coordinate Frame Transformations

**Body Frame (B)**: Fixed to aircraft
```
x̂_B: forward (nose direction)
ŷ_B: right wing
ẑ_B: down (perpendicular to wings)
```

**Inertial Frame (I)**: Earth-fixed
```
x̂_I: North
ŷ_I: East
ẑ_I: Down (NED convention)
```

**Rotation Matrix from Quaternion**:
```
R(q) = ⎡1-2(y²+z²)   2(xy-wz)     2(xz+wy)  ⎤
       ⎢2(xy+wz)     1-2(x²+z²)   2(yz-wx)  ⎥
       ⎣2(xz-wy)     2(yz+wx)     1-2(x²+y²)⎦
```

**Transformation**:
```
v_I = R(q) · v_B
```

#### Angular Velocity Integration

**Quaternion Derivative**:
```
q̇ = (1/2) · q ⊗ [0, ω_B]
where ω_B = [ωₓ, ωᵧ, ωᵨ]ᵀ (body-frame angular velocity)
```

**Discrete Integration (4th order Runge-Kutta)**:
```
q(t+Δt) = q(t) + (Δt/6)(k₁ + 2k₂ + 2k₃ + k₄)
where k₁ = q̇(t, q(t))
      k₂ = q̇(t+Δt/2, q(t)+Δt·k₁/2)
      k₃ = q̇(t+Δt/2, q(t)+Δt·k₂/2)
      k₄ = q̇(t+Δt, q(t)+Δt·k₃)
```

**Normalization** (critical for numerical stability):
```
q ← q / ||q||
```

## 3. Materials Science for Ornithopter Design

### 3.1 Wing Material Requirements

**Critical Properties**:

1. **Specific Strength** (σ/ρ):
   - Carbon fiber: 2,457 kN·m/kg
   - Mylar film: 200 kN·m/kg
   - Polyimide (Kapton): 231 kN·m/kg

2. **Elastic Modulus**:
   - Wing spar (carbon fiber): 230 GPa
   - Wing membrane: 3-5 GPa
   - Required flexibility for ~30 Hz flapping

3. **Fatigue Life**:
   ```
   N = C(Δσ)⁻ᵐ
   where N = cycles to failure
         Δσ = stress amplitude
         C, m = material constants
   ```
   
   For 30 Hz operation: N > 10⁸ cycles (≈1 hour flight)

### 3.2 Structural Analysis

**Wing Bending Under Aerodynamic Load**:

Euler-Bernoulli beam equation:
```
EI · d⁴w/dx⁴ = q(x)
where E = Young's modulus
      I = second moment of area
      w = deflection
      q(x) = distributed load
```

**Critical Buckling Load**:
```
P_cr = π²EI / (KL)²
where K = effective length factor (1.0 for pinned-pinned)
      L = unsupported length
```

**Safety Factor**: Minimum 2.0 for dynamic loading

### 3.3 Material Selection Matrix

| Component | Material | Density (g/cm³) | Reason |
|-----------|----------|-----------------|--------|
| Wing spar | Carbon fiber | 1.6 | High strength-to-weight |
| Wing membrane | Mylar (6μm) | 1.4 | Flexibility, low weight |
| Frame | Carbon fiber tube | 1.6 | Structural rigidity |
| Gears | Delrin (POM) | 1.41 | Low friction, machinable |
| Linkages | Titanium alloy | 4.5 | Fatigue resistance |

## 4. Fluid Mechanics and Aerodynamics

### 4.1 Flapping Wing Aerodynamics

**Lift Generation Mechanisms**:

1. **Quasi-Steady Model**:
   ```
   L(t) = (1/2)ρV²(t)S·C_L(α(t))
   where ρ = air density (1.225 kg/m³)
         V(t) = instantaneous wing velocity
         S = wing area
         C_L = lift coefficient (function of angle of attack α)
   ```

2. **Unsteady Effects**:
   - **Wagner Effect**: Delayed circulation development
   - **Leading Edge Vortex (LEV)**: Enhances lift at high α
   - **Added Mass**: Acceleration reaction
   
   Total lift:
   ```
   L_total = L_quasi-steady + L_unsteady
   L_unsteady = ρπc²/4 · (dα/dt + dw/dt)
   where c = chord length
         w = induced velocity
   ```

3. **Clap-and-Fling Mechanism**:
   ```
   Γ_enhanced = 2πcV[sin(α) + π/4 · c/b]
   where b = wing separation distance
   ```
   Enhancement factor: 1.5-2.5× for hovering

### 4.2 Hover Condition

**Force Balance**:
```
L_avg = mg (weight)
T = 0 (no net thrust required)
M_pitch = 0 (balanced torques)
```

**Power Required**:
```
P_hover = mg · v_induced + P_profile + P_inertial
where v_induced = √(mg / (2ρA)) (induced velocity)
      P_profile = (1/2)ρV³S·C_D
      P_inertial = I_wing·ω²·f (accelerating wings)
```

**Figure of Merit**:
```
FM = P_ideal / P_actual = (mg)^(3/2) / (√(2ρA) · P_actual)
```
Target: FM > 0.5 for efficient hover

### 4.3 Stability Derivatives

**Pitch Stability**:
```
M = M₀ + M_α·α + M_q·(qc/2V) + M_δ·δ
where M_α = ∂M/∂α (pitch stiffness)
      M_q = ∂M/∂q (pitch damping)
      M_δ = ∂M/∂δ (control effectiveness)
```

**Stability Criterion**:
```
M_α < 0 (restoring moment)
M_q < 0 (damping)
```

### 4.4 Computational Fluid Dynamics (CFD)

**Recommended Approach**:

1. **Reynolds Number**:
   ```
   Re = ρVc/μ ≈ 10,000-50,000 (transitional regime)
   ```

2. **Numerical Methods**:
   - Immersed Boundary Method for moving wings
   - Large Eddy Simulation (LES) for vortex resolution
   - Adaptive mesh refinement near wing surfaces

3. **Validation**:
   - Particle Image Velocimetry (PIV) experiments
   - Force/torque measurements on test rig

## 5. Machine Learning and Adaptive Control

### 5.1 Multi-Layer Perceptron (MLP) for Control

**Architecture**:
```
Input Layer: [ω_x, ω_y, ω_z, α, β, γ, V_x, V_y, V_z, sensor_raw...] (15-20 inputs)
Hidden Layer 1: 32 neurons (ReLU activation)
Hidden Layer 2: 16 neurons (ReLU activation)
Output Layer: [δ_left, δ_right, δ_freq] (3-4 outputs, tanh activation)
```

**Training Data**:
- Simulate diverse flight conditions
- Collect pilot demonstrations
- Online learning during flight

**Loss Function**:
```
L = α·||x_desired - x_actual||² + β·||u||² + γ·J_smoothness
where α, β, γ are weighting factors
      J_smoothness penalizes high-frequency control
```

### 5.2 Self-Awareness Algorithms

**State Estimation Confidence**:
```
P(t+1) = F·P(t)·Fᵀ + Q (prediction)
K = P·Hᵀ(H·P·Hᵀ + R)⁻¹ (Kalman gain)
x(t+1) = x_pred + K(z - H·x_pred) (update)
P(t+1) = (I - K·H)P (covariance update)
```

**Anomaly Detection**:
```
χ² = (z - H·x)ᵀ·S⁻¹·(z - H·x)
where S = H·P·Hᵀ + R (innovation covariance)
```
If χ² > threshold: flag sensor fault or model mismatch

**Remaining Flight Time Estimation**:
```
t_remaining = E_battery / P_avg(hover_state)
```

### 5.3 Situational Awareness

**Wind Estimation**:
```
v_wind = v_GPS - R(q)·v_body
Filtered: v_wind_filtered = α·v_wind_filtered + (1-α)·v_wind
```

**Obstacle Detection**:
- Optical flow divergence
- Ultrasonic ranging
- Machine learning-based object recognition

**Decision Making**:
```
Action = argmax_a Q(s, a)
where Q is learned action-value function
      s = current state
      a = action (continue, land, avoid)
```

### 5.4 Adaptive Control

**Model Reference Adaptive Control (MRAC)**:
```
u = θᵀφ(x) (adaptive controller)
θ̇ = -Γ·φ(x)·eᵀ·P·B (adaptation law)
where θ = adaptive parameters
      Γ = adaptation rate
      e = tracking error
```

**Neural Network Augmentation**:
```
u_total = u_baseline + u_NN
where u_baseline = PID or LQR controller
      u_NN = learned correction
```

## 6. Sensor Integration and Hardware Mathematics

### 6.1 MPU-6050 IMU

**Specifications**:
- Gyroscope range: ±250, ±500, ±1000, ±2000 °/s
- Accelerometer range: ±2, ±4, ±8, ±16 g
- Sample rate: 4-8000 Hz
- Temperature sensor: -40°C to +85°C

**Data Fusion**:

**Complementary Filter** (lightweight):
```
α_fused = α·(α_prev + ω_gyro·Δt) + (1-α)·α_accel
where α = 0.98 (trust gyro for short term)
      α_accel = atan2(a_y, a_z)
```

**Extended Kalman Filter** (optimal):
```
State: x = [q_w, q_x, q_y, q_z, ω_x, ω_y, ω_z, b_x, b_y, b_z]
       (quaternion, angular velocity, gyro biases)

Prediction:
  q̇ = (1/2)q ⊗ [0, ω - b]
  ω̇ = 0
  ḃ = 0

Measurement: z = [a_x, a_y, a_z] (accelerometer)
Expected: h(x) = R(q)ᵀ·[0, 0, g]

Update with standard EKF equations
```

### 6.2 Pressure Sensors (BMP280/MS5611)

**Altitude Estimation**:
```
h = h₀ · (1 - (P/P₀)^(1/5.255))
where h₀ = 44,330 m
      P₀ = 101,325 Pa (sea level)
```

**Vertical Velocity**:
```
v_z = -dh/dt ≈ -(h(t) - h(t-Δt)) / Δt
Filtered with 1st order low-pass filter
```

**Sensor Fusion Weight**:
```
h_fused = w₁·h_baro + w₂·h_IMU
where w₁ = σ²_IMU / (σ²_baro + σ²_IMU)
      w₂ = 1 - w₁
```

### 6.3 Humidity Sensors (DHT22/SHT31)

**Air Density Correction**:
```
ρ_humid = ρ_dry · (1 - 0.378·e/P)
where e = RH · e_sat(T) (partial pressure of water vapor)
      e_sat = 611.2 · exp(17.67·T / (T+243.5)) Pa
      RH = relative humidity
```

**Impact on Aerodynamics**:
- Lift reduction: ~0.5% per 10% RH increase at 20°C
- Adjust control gains accordingly

### 6.4 Wind Sensors (Hot-Wire Anemometer/Pitot Tube)

**Airspeed from Pitot-Static**:
```
V = √(2·ΔP / ρ)
where ΔP = P_total - P_static
```

**3D Wind Vector Estimation**:
```
v_wind = [v_x, v_y, v_z]ᵀ
Estimated from GPS velocity and IMU:
v_wind = v_GPS - R(q)·v_body
```

**Gust Response**:
```
M_gust = (1/2)ρV²Sc̄·C_M_α·(w_gust/V)
```

### 6.5 Gyroscope Calibration

**Bias Estimation** (at startup):
```
b̂ = (1/N) Σᵢ ωᵢ (average over N samples)
```

**Scale Factor**:
```
ω_true = (ω_raw - b) / s
where s = scale factor (typically 1 ± 0.02)
```

**Temperature Compensation**:
```
b(T) = b₀ + b₁·T + b₂·T²
Calibrated across temperature range
```

## 7. Stability Tracking and Control

### 7.1 PID Control

**Standard Form**:
```
u(t) = K_p·e(t) + K_i·∫e(τ)dτ + K_d·de/dt
```

**Discrete Implementation**:
```
u[k] = K_p·e[k] + K_i·Σe[j]·Δt + K_d·(e[k]-e[k-1])/Δt
```

**Tuning Guidelines**:
- Ziegler-Nichols method
- Relay auto-tuning
- Genetic algorithms for multi-objective optimization

**Anti-Windup**:
```
if u > u_max:
    integral_error = integral_error - (u - u_max)/K_i
```

### 7.2 Linear Quadratic Regulator (LQR)

**Optimal Control**:
```
u = -K·x
where K = R⁻¹·Bᵀ·P
      P solves Riccati equation: AᵀP + PA - PBR⁻¹BᵀP + Q = 0
```

**Cost Function**:
```
J = ∫₀^∞ (xᵀQx + uᵀRu) dt
```

**Tuning Matrices**:
```
Q = diag([q_roll, q_pitch, q_yaw, q_ω_x, q_ω_y, q_ω_z])
R = diag([r_δ_left, r_δ_right])
```

### 7.3 Gain Scheduling

**Flight Regime Detection**:
```
Regime = {hover, forward_flight, transition}
```

**Smooth Interpolation**:
```
K(V) = K_hover + (K_forward - K_hover)·σ(V)
where σ(V) = 1 / (1 + exp(-a(V - V_transition)))
```

### 7.4 Stability Metrics

**Phase Margin**:
```
PM = 180° + ∠G(jω_gc)
where ω_gc = gain crossover frequency
Target: PM > 45°
```

**Gain Margin**:
```
GM = 1 / |G(jω_pc)|
where ω_pc = phase crossover frequency
Target: GM > 6 dB
```

**Time Response**:
- Rise time: < 0.2 s
- Settling time: < 0.5 s
- Overshoot: < 20%

## 8. Formal Verification with TLA+ and Z3

### 8.1 TLA+ Overview

TLA+ (Temporal Logic of Actions) is a formal specification language for modeling and verifying concurrent and distributed systems. For the ornithopter controller, TLA+ enables:

- **State Space Exploration**: Verify all possible system states
- **Safety Properties**: Ensure bad states are never reached
- **Liveness Properties**: Ensure good things eventually happen
- **Temporal Logic**: Reason about system behavior over time

### 8.2 Z3 Solver Overview

Z3 is a high-performance SMT (Satisfiability Modulo Theories) solver developed by Microsoft Research. Applications for ornithopter control:

- **Constraint Verification**: Verify mathematical relationships hold
- **Bound Checking**: Ensure values stay within safe ranges
- **Invariant Proving**: Prove system properties mathematically
- **Optimization**: Find optimal parameter values

### 8.3 Integration Workflow

**Development Process**:
```
1. Design → TLA+ Specification → Model Check
2. Implementation → Unit Tests
3. Integration → Z3 Verification → Property Tests
4. Hardware-in-Loop → Flight Tests
```

**Benefits**:
- Early bug detection
- Formal proof of safety properties
- Documentation of system behavior
- Regression prevention

## 9. Build System Modernization

### 9.1 PlatformIO Overview

PlatformIO is a modern cross-platform build system for embedded development that provides:

- **Unified Interface**: Single tool for multiple platforms
- **Dependency Management**: Automatic library installation
- **Unit Testing**: Built-in testing framework
- **CI/CD Integration**: GitHub Actions, Travis, etc.
- **Multiple Frameworks**: Arduino, mbed, ESP-IDF, etc.

### 9.2 Key Features

**Multi-Board Support**:
- Arduino Nano
- Teensy 4.0
- ESP32
- STM32

**Build Optimization**:
- Compiler flags customization
- Link-time optimization
- Size/speed trade-offs

**Testing Framework**:
- Unit tests on host machine
- Integration tests on target hardware
- Test-driven development support

## 10. Hardware Interaction Principles

### 10.1 Sensor Fusion Strategy

**Hierarchical Approach**:
1. **Raw Sensors** → Calibration → Calibrated Data
2. **Calibrated Data** → Filtering → Clean Signals
3. **Clean Signals** → Fusion → State Estimate
4. **State Estimate** → Controller → Control Commands

**Redundancy**:
- Multiple sensors for critical measurements
- Cross-validation between sensor types
- Graceful degradation on sensor failure

### 10.2 Control Architecture

**Layered Control**:
```
Mission Planning (high level, slow)
    ↓
Trajectory Generation (medium level, medium speed)
    ↓
Stabilization Control (low level, fast)
    ↓
Motor/Servo Commands (hardware level, very fast)
```

**Timing Requirements**:
- Stabilization loop: 100-200 Hz
- Trajectory updates: 10-50 Hz
- Mission decisions: 1-10 Hz

### 10.3 Safety Systems

**Multi-Level Safety**:
1. **Hardware**: Voltage monitors, watchdog timers
2. **Software**: Bounds checking, fault detection
3. **Control**: Envelope protection, safe mode
4. **Mission**: Geofencing, auto-land

**Fault Response Hierarchy**:
```
1. Recover (switch to backup sensor)
2. Degrade (reduce performance, maintain safety)
3. Land (controlled descent)
4. Emergency (immediate motor cutoff if needed)
```

## 11. Interaction Analysis

### 11.1 System Coupling Matrix

| Component | IMU | Pressure | Humidity | Wind | Controller | Actuators | Aerodynamics |
|-----------|-----|----------|----------|------|------------|-----------|--------------|
| IMU | - | Weak | None | None | Strong | None | Weak |
| Pressure | Weak | - | Medium | Medium | Medium | None | Medium |
| Humidity | None | Medium | - | Weak | Weak | None | Weak |
| Wind | None | Medium | Weak | - | Strong | None | Strong |
| Controller | Strong | Medium | Weak | Strong | - | Strong | Strong |
| Actuators | None | None | None | None | Strong | - | Strong |
| Aerodynamics | Weak | Medium | Weak | Strong | Strong | Strong | - |

**Coupling Strength**:
- Strong: Direct, fast, critical interaction
- Medium: Indirect or slower interaction
- Weak: Minor influence
- None: No significant interaction

### 11.2 Critical Feedback Loops

**Primary Loop** (fastest, ~5-10 ms):
```
Gyroscope → Attitude Estimate → Roll/Pitch Controller → Servos → Wing Motion → Body Rotation → Gyroscope
```

**Secondary Loop** (medium, ~50-100 ms):
```
Accelerometer → Position Estimate → Altitude Controller → Motor Throttle → Thrust → Altitude → Accelerometer
```

**Tertiary Loop** (slowest, ~1-5 s):
```
Wind Estimation → Adaptive Gains → Control Performance → Tracking Error → Wind Estimation
```

### 11.3 Emergent Behaviors

**Desired Emergent Behaviors**:
- Automatic disturbance rejection
- Smooth transition between flight modes
- Energy-efficient hover

**Undesired Emergent Behaviors to Prevent**:
- Pilot-induced oscillations (PIO)
- Sensor-induced instability
- Actuator saturation spiral

## 12. Implementation Roadmap

### 12.1 Phase 1: Foundation (Weeks 1-4)

**Tasks**:
- [ ] Set up PlatformIO project structure
- [ ] Implement quaternion math library
- [ ] Create sensor abstraction layer
- [ ] Develop basic IMU integration
- [ ] Write unit tests for core math

**Deliverables**:
- Functional quaternion library
- MPU-6050 driver with calibration
- Test coverage > 80%

### 12.2 Phase 2: Control System (Weeks 5-8)

**Tasks**:
- [ ] Implement complementary filter
- [ ] Develop PID controllers for roll/pitch/yaw
- [ ] Create altitude controller
- [ ] Implement anti-windup and rate limiting
- [ ] Tune controllers in simulation

**Deliverables**:
- Complete control system
- Simulation validation
- Tuning documentation

### 12.3 Phase 3: Advanced Features (Weeks 9-12)

**Tasks**:
- [ ] Implement Extended Kalman Filter
- [ ] Add wind estimation
- [ ] Develop MLP controller (optional)
- [ ] Create telemetry system
- [ ] Implement data logging

**Deliverables**:
- EKF-based state estimator
- Telemetry protocol
- Flight data recorder

### 12.4 Phase 4: Verification (Weeks 13-16)

**Tasks**:
- [ ] Write TLA+ specifications
- [ ] Create Z3 verification scripts
- [ ] Perform model checking
- [ ] Conduct hardware-in-loop tests
- [ ] Flight testing and validation

**Deliverables**:
- Formal verification suite
- HIL test results
- Flight test data and analysis

## 13. Conclusion

This comprehensive research and development report provides the theoretical and practical foundation for creating an advanced hovering ornithopter system. The integration of:

- **Mathematical rigor** through quaternions and spatial calculations
- **Physical modeling** via materials science and fluid mechanics
- **Intelligent control** using MLP and adaptive algorithms
- **Sensor fusion** with multi-modal hardware integration
- **Formal methods** via TLA+ and Z3 verification
- **Modern tooling** through PlatformIO and CI/CD

creates a robust framework for development, testing, and deployment of autonomous flapping-wing aircraft.

The identified lacunae and technical debt provide clear directions for improvement, while the detailed mathematical formulations enable precise implementation. The formal verification workflow ensures safety and correctness, and the modern build system facilitates rapid iteration and testing.

This integrated approach represents the state-of-the-art in ornithopter control systems and provides a solid foundation for future research and development.

## References

1. Ellington, C. P. (1984). "The aerodynamics of hovering insect flight." *Philosophical Transactions of the Royal Society B*, 305(1122), 1-181.

2. Karásek, M., et al. (2018). "A tailless aerial robotic flapper reveals that flies use torque coupling in rapid banked turns." *Science*, 361(6407), 1089-1094.

3. Tedrake, R., et al. (2009). "LQR-trees: Feedback motion planning via solvable reachability games." *ICRA*.

4. Lamport, L. (2002). *Specifying Systems: The TLA+ Language and Tools for Hardware and Software Engineers*. Addison-Wesley.

5. de Moura, L., & Bjørner, N. (2008). "Z3: An efficient SMT solver." *TACAS*.

6. Siciliano, B., et al. (2016). *Robotics: Modelling, Planning and Control*. Springer.

7. Shyy, W., et al. (2013). *Flapping Wing Flight: Aerodynamics, Stability and Control*. Cambridge University Press.

8. Stengel, R. F. (2015). *Flight Dynamics*. Princeton University Press.

---

*Document Version: 1.0*  
*Date: 2026-01-02*  
*Classification: Research and Development*
