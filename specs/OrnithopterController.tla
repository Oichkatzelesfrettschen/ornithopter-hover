----------------------------- MODULE OrnithopterController -----------------------------
(*
    TLA+ Specification for Ornithopter Flight Controller
    
    This specification models the state machine and safety properties
    of a hovering ornithopter control system.
*)

EXTENDS Integers, Reals, Sequences, TLC

CONSTANTS 
    MaxThrottle,           \* Maximum motor throttle (0-100)
    MinSafeAltitude,       \* Minimum safe altitude in meters
    MaxTiltAngle,          \* Maximum allowable tilt in degrees
    SensorTimeout,         \* Maximum sensor update interval in ms
    CriticalBatteryVoltage,\* Minimum battery voltage for safe operation
    FullBatteryVoltage,    \* Fully charged battery voltage
    HoverThrottle         \* Nominal throttle for hover

VARIABLES
    mode,                  \* Flight mode: STARTUP, ARMED, HOVER, LANDING, EMERGENCY
    altitude,              \* Current altitude in meters (Real)
    attitudeRoll,          \* Roll angle in degrees
    attitudePitch,         \* Pitch angle in degrees
    attitudeYaw,           \* Yaw angle in degrees
    throttle,              \* Motor throttle percentage (0-100)
    sensorHealthIMU,       \* IMU sensor health (TRUE/FALSE)
    sensorHealthPressure,  \* Pressure sensor health (TRUE/FALSE)
    lastSensorUpdate,      \* Timestamp of last sensor update
    batteryVoltage,        \* Current battery voltage
    timeInMode,            \* Time spent in current mode
    emergencyReason        \* Reason for emergency mode (if any)

vars == <<mode, altitude, attitudeRoll, attitudePitch, attitudeYaw, throttle, 
          sensorHealthIMU, sensorHealthPressure, lastSensorUpdate, 
          batteryVoltage, timeInMode, emergencyReason>>

-----------------------------------------------------------------------------

(* Type Invariants *)

TypeOK == 
    /\ mode \in {"STARTUP", "ARMED", "HOVER", "LANDING", "EMERGENCY"}
    /\ altitude \in Int  \* Simplified to Int for model checking
    /\ attitudeRoll \in Int
    /\ attitudePitch \in Int
    /\ attitudeYaw \in Int
    /\ throttle \in 0..MaxThrottle
    /\ sensorHealthIMU \in BOOLEAN
    /\ sensorHealthPressure \in BOOLEAN
    /\ lastSensorUpdate \in Nat
    /\ batteryVoltage \in Int  \* Simplified to Int for model checking
    /\ timeInMode \in Nat
    /\ emergencyReason \in {"NONE", "LOW_BATTERY", "SENSOR_FAULT", 
                            "ATTITUDE_LIMIT", "TIMEOUT"}

-----------------------------------------------------------------------------

(* Helper Functions *)

GetTiltAngle ==
    \* Simplified tilt magnitude calculation
    IF attitudeRoll < 0 THEN -attitudeRoll ELSE attitudeRoll

SensorsHealthy ==
    sensorHealthIMU /\ sensorHealthPressure

SensorTimedOut ==
    lastSensorUpdate > SensorTimeout

BatteryCritical ==
    batteryVoltage < CriticalBatteryVoltage

AttitudeExceedsLimit ==
    GetTiltAngle > MaxTiltAngle

-----------------------------------------------------------------------------

(* Safety Invariants *)

SafetyInvariant ==
    /\ mode = "HOVER" => altitude >= MinSafeAltitude
    /\ mode = "EMERGENCY" => throttle = 0
    /\ GetTiltAngle <= MaxTiltAngle \/ mode \in {"EMERGENCY", "LANDING"}
    /\ BatteryCritical => mode \in {"LANDING", "EMERGENCY"}
    /\ ~SensorsHealthy => mode \in {"LANDING", "EMERGENCY"}

-----------------------------------------------------------------------------

(* Initial State *)

Init ==
    /\ mode = "STARTUP"
    /\ altitude = 0
    /\ attitudeRoll = 0
    /\ attitudePitch = 0
    /\ attitudeYaw = 0
    /\ throttle = 0
    /\ sensorHealthIMU = TRUE
    /\ sensorHealthPressure = TRUE
    /\ lastSensorUpdate = 0
    /\ batteryVoltage = FullBatteryVoltage
    /\ timeInMode = 0
    /\ emergencyReason = "NONE"

-----------------------------------------------------------------------------

(* State Transitions *)

Startup ==
    /\ mode = "STARTUP"
    /\ SensorsHealthy
    /\ timeInMode > 100  \* Require 100ms startup time
    /\ mode' = "ARMED"
    /\ timeInMode' = 0
    /\ UNCHANGED <<altitude, attitudeRoll, attitudePitch, attitudeYaw, 
                   throttle, sensorHealthIMU, sensorHealthPressure, 
                   lastSensorUpdate, batteryVoltage, emergencyReason>>

Arm ==
    /\ mode = "ARMED"
    /\ SensorsHealthy
    /\ ~BatteryCritical
    /\ throttle' = HoverThrottle
    /\ mode' = "HOVER"
    /\ timeInMode' = 0
    /\ UNCHANGED <<altitude, attitudeRoll, attitudePitch, attitudeYaw,
                   sensorHealthIMU, sensorHealthPressure, lastSensorUpdate, 
                   batteryVoltage, emergencyReason>>

Hover ==
    /\ mode = "HOVER"
    /\ SensorsHealthy
    /\ ~BatteryCritical
    /\ ~AttitudeExceedsLimit
    \* Simulate altitude changes
    /\ altitude' \in {altitude - 1, altitude, altitude + 1}
    /\ altitude' >= MinSafeAltitude
    \* Simulate attitude changes
    /\ attitudeRoll' \in {attitudeRoll - 1, attitudeRoll, attitudeRoll + 1}
    /\ attitudePitch' \in {attitudePitch - 1, attitudePitch, attitudePitch + 1}
    /\ GetTiltAngle <= MaxTiltAngle
    /\ lastSensorUpdate' = lastSensorUpdate + 1
    /\ batteryVoltage' = IF batteryVoltage > CriticalBatteryVoltage 
                         THEN batteryVoltage - 1 
                         ELSE batteryVoltage
    /\ timeInMode' = timeInMode + 1
    /\ UNCHANGED <<mode, attitudeYaw, throttle, sensorHealthIMU, 
                   sensorHealthPressure, emergencyReason>>

InitiateLanding ==
    /\ mode = "HOVER"
    /\ \/ BatteryCritical
       \/ timeInMode > 1000  \* Maximum hover time
    /\ mode' = "LANDING"
    /\ timeInMode' = 0
    /\ emergencyReason' = IF BatteryCritical 
                          THEN "LOW_BATTERY" 
                          ELSE emergencyReason
    /\ UNCHANGED <<altitude, attitudeRoll, attitudePitch, attitudeYaw, 
                   throttle, sensorHealthIMU, sensorHealthPressure, 
                   lastSensorUpdate, batteryVoltage>>

Land ==
    /\ mode = "LANDING"
    /\ altitude > 0
    /\ altitude' = altitude - 1  \* Descend
    /\ throttle' = IF altitude' = 0 THEN 0 ELSE throttle
    /\ mode' = IF altitude' = 0 THEN "ARMED" ELSE mode
    /\ timeInMode' = IF altitude' = 0 THEN 0 ELSE timeInMode + 1
    /\ UNCHANGED <<attitudeRoll, attitudePitch, attitudeYaw, sensorHealthIMU, 
                   sensorHealthPressure, lastSensorUpdate, batteryVoltage, 
                   emergencyReason>>

TriggerEmergency ==
    /\ mode \in {"HOVER", "LANDING"}
    /\ \/ ~SensorsHealthy
       \/ SensorTimedOut
       \/ AttitudeExceedsLimit
    /\ mode' = "EMERGENCY"
    /\ throttle' = 0
    /\ emergencyReason' = IF ~SensorsHealthy THEN "SENSOR_FAULT"
                          ELSE IF SensorTimedOut THEN "TIMEOUT"
                          ELSE IF AttitudeExceedsLimit THEN "ATTITUDE_LIMIT"
                          ELSE "NONE"
    /\ timeInMode' = 0
    /\ UNCHANGED <<altitude, attitudeRoll, attitudePitch, attitudeYaw, 
                   sensorHealthIMU, sensorHealthPressure, lastSensorUpdate, 
                   batteryVoltage>>

\* Simulate sensor failures (for model checking)
SensorFailure ==
    /\ mode \in {"HOVER", "LANDING"}
    /\ \/ sensorHealthIMU' = FALSE
       \/ sensorHealthPressure' = FALSE
    /\ UNCHANGED <<mode, altitude, attitudeRoll, attitudePitch, attitudeYaw, 
                   throttle, lastSensorUpdate, batteryVoltage, timeInMode, 
                   emergencyReason>>

-----------------------------------------------------------------------------

(* Next State Relation *)

Next ==
    \/ Startup
    \/ Arm
    \/ Hover
    \/ InitiateLanding
    \/ Land
    \/ TriggerEmergency
    \/ SensorFailure

-----------------------------------------------------------------------------

(* Temporal Properties *)

\* Eventually reach hover mode from startup
EventuallyHover ==
    <>[](mode = "HOVER")

\* Emergency mode is triggered on critical conditions
EmergencyResponse ==
    [](~SensorsHealthy => <>(mode = "EMERGENCY"))

\* System eventually lands safely
EventuallyLand ==
    [](mode = "LANDING" => <>(altitude = 0))

\* Battery monotonically decreases during flight
BatteryMonotonic ==
    [](mode = "HOVER" => batteryVoltage' <= batteryVoltage)

\* Altitude maintained above minimum during hover
AltitudeSafety ==
    [](mode = "HOVER" => altitude >= MinSafeAltitude)

-----------------------------------------------------------------------------

(* Specification *)

Spec == Init /\ [][Next]_vars /\ WF_vars(Next)

\* Properties to check
THEOREM Spec => []TypeOK
THEOREM Spec => []SafetyInvariant
THEOREM Spec => EmergencyResponse
THEOREM Spec => AltitudeSafety

=============================================================================
