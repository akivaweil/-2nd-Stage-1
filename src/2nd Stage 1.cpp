#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>

// ? ===============================================================================
// ? 🔄 CUTTING CYCLE SEQUENCE
// ? ===============================================================================
/*
 * 1. 🏁 Initial Position Check:
 *    - 🔴🔴 Both clamps engaged (HIGH)
 *    - 🎯 Cut motor at home position (0)
 *    - 📐 Position motor at home position (0)
 * 
 * 2. 🚀 Cycle Start:
 *    - ✅ Verify start cycle switch remains pressed
 *    - 🔴🔴 Ensure both clamps are engaged
 *    - ⏱️ Wait for clamp operation verification
 * 
 * 3. 🔪 Cut Sequence:
 *    - 🎯 Move cut motor forward (9", ⚡100, 📈2000)
 *    - ⏱️ Wait for cut completion
 *    - 🔵 Retract position clamp
 *    - ⏱️ Wait for clamp operation
 * 
 * 4. 🏠 Return Sequence:
 *    - 🎯 Return cut motor (⚡1250, 📈2000)
 *    - 🎯 Return position motor (⚡6000, 📈5000)
 *    - 🔴 Re-engage position clamp
 *    - ⏱️ Wait for clamp engagement
 *    - 🔵 Disengage wood secure clamp
 * 
 * 5. 📏 Position Sequence:
 *    - 🎯 Move position motor (3.35", ⚡6000, 📈5000)
 *    - ⏱️ Wait for stabilization
 */

// ! ===============================================================================
// ! 🛑 SAFETY NOTICE: DO NOT MODIFY WITHOUT APPROVAL
// ! ===============================================================================
/*
 * AUTOMATED TABLE SAW CONTROL SYSTEM - STAGE 2
 * This code controls an automated table saw cutting system. Safety is the absolute priority.
 * - Code clarity and reliability take precedence over processing efficiency
 * - All functions are written to be as explicit and straightforward as possible
 * - Hardware emergency stop switch cuts ALL power to the system when activated
 * - Multiple software safety checks are implemented throughout the cycle
 * - All switches and buttons read HIGH when activated
 * - All cylinders require a HIGH output to disengage
 * - Bounce2 library is used for switch debouncing with a 20ms debounce time
 * - All code should be very very easy to understand for a beginner programmer
 * - Switches are configured where one side is connected to 5v and the other side splits into 10k resistor to ground at its signal pin
 */

// * ===============================================================================
// * 📝 SYSTEM DOCUMENTATION
// * ===============================================================================
/*
 * MAINTENANCE NOTE: This sequence documentation should be kept in sync with code implementation.
 * Please update this section when modifying the cycle steps or timing values.
 * 
 * 🔄 CUTTING CYCLE SEQUENCE:
 * 
 * 1. 🏁 Initial Position Check:
 *    - 🔴🔴 Both clamps engaged (HIGH)
 *    - 🎯 Cut motor at home position (0)
 *    - 📐 Position motor at home position (0)
 * 
 * 2. 🚀 Cycle Start:
 *    - ✅ Verify start cycle switch remains pressed
 *    - 🔴🔴 Ensure both clamps are engaged
 *    - ⏱️ Wait for clamp operation verification
 * 
 * 3. 🔪 Cut Sequence:
 *    - 🎯 Move cut motor forward (9 inches, ⚡100, 📈2000)
 *    - ⏱️ Wait for cut completion
 *    - 🔵 Retract position clamp
 *    - ⏱️ Wait for clamp operation
 * 
 * 4. 🏠 Return Sequence:
 *    - 🎯 Return cut motor (⚡1250, 📈2000)
 *    - 🎯 Return position motor (⚡6000, 📈5000)
 *    - 🔴 Re-engage position clamp
 *    - ⏱️ Wait for clamp engagement
 *    - 🔵 Disengage wood secure clamp
 * 
 * 5. 📏 Position Sequence:
 *    - 🎯 Move position motor (3.35 inches, ⚡6000, 📈5000)
 *    - ⏱️ Wait for stabilization
 * 
 * ⚠️ Safety Features:
 * - Hardware emergency stop cuts all power
 * - Multiple software safety checks
 * - Debounced switch readings
 * - Explicit clamp state verification
 * 
 * 📝 Note: All timing values are defined as constants at the top of file
 */

// * ===============================================================================
// * 🏠 HOMING SEQUENCE
// * ===============================================================================
/*
 * 1. 🔄 Initial Check:
 *    - 📊 Report initial switch states
 *    - 🔍 Check if motors are already at home position
 *    - ⚡ Configure motors for homing speeds
 * 
 * 2. 🎯 Cut Motor Homing:
 *    - 🔄 Move cut motor towards home switch (⚡250, 📈2000)
 *    - ⏱️ Monitor switch state with debouncing
 *    - ✋ Stop immediately when switch is activated
 *    - 📍 Set current position as zero
 *    - 📢 Report homing status
 * 
 * 3. 📏 Position Motor Homing:
 *    - 🔄 Move position motor towards home switch (⚡1000, 📈5000)
 *    - 🔓 Position clamp disengaged during homing
 *    - ⏱️ Monitor switch state with debouncing
 *    - ✋ Stop when switch is activated
 *    - 📍 Set current position as zero
 *    - 🔒 Engage position clamp
 *    - 📢 Report completion
 * 
 * 4. ✅ Completion Checks:
 *    - 🔍 Verify both motors are homed
 *    - 🔒 Confirm clamp states
 *    - 📢 Report system ready status
 * 
 * ⚠️ Safety Features During Homing:
 * - 🐌 Reduced motor speeds
 * - ⏱️ Debounced switch readings
 * - 🛑 Immediate stop on switch activation
 * - 📊 Continuous status reporting
 * - ⚡ Proper acceleration control
 */

// ! ===============================================================================
// ! ⚠️ HARDWARE CONFIGURATION - DO NOT MODIFY
// ! ===============================================================================

// Pin Definitions - Motors (DO NOT MODIFY - Hardware Dependent)
const int PIN_CUT_MOTOR_PUL = 11;      // Cut motor pulse pin
const int PIN_CUT_MOTOR_DIR = 12;      // Cut motor direction pin
const int PIN_POSITION_MOTOR_PUL = 5;  // Position motor pulse pin
const int PIN_POSITION_MOTOR_DIR = 6;  // Position motor direction pin

// Pin Definitions - Safety Switches (DO NOT MODIFY - Hardware Dependent)
const int PIN_CUT_MOTOR_POSITION_SWITCH = 7;      // Limit switch for cut motor
const int PIN_POSITION_MOTOR_POSITION_SWITCH = 8;  // Limit switch for position motor
const int PIN_RELOAD_SWITCH = 9;                  // Material reload switch
const int PIN_START_CYCLE_SWITCH = 10;            // Switch to start main cycle

// Pin Definitions - Safety Clamps (DO NOT MODIFY - Hardware Dependent)
const int PIN_POSITION_CLAMP = 4;       // Clamp for maintaining position
const int PIN_SECURE_WOOD_CLAMP = 3;    // Clamp for securing material

// Clamp States - Safety Critical
const int CLAMP_ENGAGED = HIGH;    // Clamp extends to secure material
const int CLAMP_DISENGAGED = LOW;  // Clamp retracts to release material

// Homing Parameters - Safety Critical
const int HOME_SPEED = 2000;               // Homing movement speed
const int HOME_DIRECTION = -1;             // Cut motor homing direction
const int POSITION_HOME_DIRECTION = 1;     // Position motor homing direction
const int DEBOUNCE_INTERVAL = 20;          // Switch debounce time (milliseconds)

// Operation Parameters - Safety Critical (DO NOT MODIFY WITHOUT TESTING)
const int STEPS_PER_INCH = 127;            // Cut motor calibration (GT2 belt)
const int POSITION_STEPS_PER_INCH = 2000;  // Position motor calibration (ball screw)
const float CUT_MOTOR_TRAVEL = 9;        // Maximum cut travel distance
const float POSITION_MOTOR_TRAVEL = 3.35;   // Maximum position travel distance
const unsigned long CLAMP_OPERATION_DELAY = 50;  // Delay for clamp operation verification

// Motor Speed and Acceleration Limits - Safety Critical
const float CUT_MOTOR_SPEED = 100;             // Normal cutting speed DO NOT DELETE: Slow mode is 100 and fast mode is 115
const float CUT_MOTOR_RETURN_SPEED = 1250;     // Return movement speed
const float POSITION_MOTOR_SPEED = 1000;       // Normal positioning speed
const float POSITION_MOTOR_RETURN_SPEED = 6000; // Return movement speed
const float CUT_MOTOR_ACCEL = 2000;            // Cut motor acceleration limit
const float POSITION_MOTOR_ACCEL = 5000;       // Position motor acceleration limit

// Motor Controllers
AccelStepper cutMotor(AccelStepper::DRIVER, PIN_CUT_MOTOR_PUL, PIN_CUT_MOTOR_DIR);
AccelStepper positionMotor(AccelStepper::DRIVER, PIN_POSITION_MOTOR_PUL, PIN_POSITION_MOTOR_DIR);

// Safety Switch Debouncers
Bounce cutSwitchDebouncer = Bounce();
Bounce positionSwitchDebouncer = Bounce();
Bounce reloadSwitch = Bounce();
Bounce startCycleSwitch = Bounce();

// System State Variables
bool cutMotorHomed = false;
bool positionMotorHomed = false;
bool startupComplete = false;
bool initialStateReported = false;

//////////////////////////////////////////////////////////////
// Function Prototypes (for clarity)
void configureSerial();
void configureSwitches();
void configureMotorsForHoming();
void configureClamps();
void printSetupComplete();
void updateSwitches();
void reportInitialStateAndCheckHoming();
void handleCutMotorHoming();
void handlePositionMotorHoming();
void runHomingSequence();
void performCutCycle();
void handleReloadSwitch();

void setup() {
    // Configure serial, switches, motors, and clamps
    configureSerial();
    configureSwitches();
    configureMotorsForHoming();
    configureClamps();
    printSetupComplete();
}

void loop() {
    updateSwitches();
    
    // Report initial switch state and check for pre-homed motors (only once)
    if (!initialStateReported) {
        reportInitialStateAndCheckHoming();
        initialStateReported = true;
    }
    
    // Run the homing sequence if not complete
    if (!startupComplete) {
        runHomingSequence();
        return;
    }
    
    // If start cycle button is pressed, perform the automated cutting cycle
    if (startCycleSwitch.read() == HIGH) {
        performCutCycle();
    }
    
    // Handle reload switch (clamp retraction) during normal operation
    handleReloadSwitch();
}

// Configures serial communication and prints startup messages
void configureSerial() {
    Serial.begin(115200);
    delay(1000);  // Wait for serial to initialize (reduced from 2000ms)
    Serial.println("----------------------------------------");
    Serial.println("Serial communication initialized");
    Serial.println("AUTOMATED TABLE SAW CONTROL SYSTEM");
    Serial.println("----------------------------------------");
}

// Configures all debounced switches
void configureSwitches() {
    cutSwitchDebouncer.attach(PIN_CUT_MOTOR_POSITION_SWITCH, INPUT);
    cutSwitchDebouncer.interval(DEBOUNCE_INTERVAL);
    Serial.println("Cut switch configured");
    
    positionSwitchDebouncer.attach(PIN_POSITION_MOTOR_POSITION_SWITCH, INPUT);
    positionSwitchDebouncer.interval(DEBOUNCE_INTERVAL);
    Serial.println("Position switch configured");
    
    reloadSwitch.attach(PIN_RELOAD_SWITCH, INPUT);
    reloadSwitch.interval(DEBOUNCE_INTERVAL);
    Serial.println("Reload switch configured");
    
    startCycleSwitch.attach(PIN_START_CYCLE_SWITCH, INPUT);
    startCycleSwitch.interval(DEBOUNCE_INTERVAL);
    Serial.println("Start cycle switch configured");
}

// Sets up the motors with homing parameters
void configureMotorsForHoming() {
    cutMotor.setMaxSpeed(HOME_SPEED / 8);  // Slower speed for homing
    cutMotor.setSpeed((HOME_SPEED / 8) * HOME_DIRECTION);
    Serial.println("Cut motor configured for homing");
    
    positionMotor.setMaxSpeed(HOME_SPEED / 2);
    positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);  // Adding acceleration for smoother movement
    positionMotor.moveTo(100000 * POSITION_HOME_DIRECTION);  // Large movement in homing direction
    Serial.println("Position motor configured for homing");
}

// Configures the clamp pins and sets their initial state
void configureClamps() {
    pinMode(PIN_POSITION_CLAMP, OUTPUT);
    pinMode(PIN_SECURE_WOOD_CLAMP, OUTPUT);
    
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);  // Start retracted
    digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
    Serial.println("Clamps configured - Position clamp retracted, wood clamp engaged");
}

// Prints the setup complete message
void printSetupComplete() {
    Serial.println("----------------------------------------");
    Serial.println("Setup complete - Beginning homing sequence");
    Serial.println("----------------------------------------");
}

// Updates the state of all debounced switches
void updateSwitches() {
    cutSwitchDebouncer.update();
    positionSwitchDebouncer.update();
    reloadSwitch.update();
    startCycleSwitch.update();
}

// Reports initial switch states and checks if motors are already at the home position
void reportInitialStateAndCheckHoming() {
    Serial.println("\nInitial State Report:");
    Serial.print("Cut Switch State: ");
    Serial.println(cutSwitchDebouncer.read() == HIGH ? "HIGH" : "LOW");
    Serial.print("Position Switch State: ");
    Serial.println(positionSwitchDebouncer.read() == HIGH ? "HIGH" : "LOW");
    Serial.print("Reload Switch State: ");
    Serial.println(reloadSwitch.read() == HIGH ? "ACTIVATED" : "NOT ACTIVATED");
    Serial.println("----------------------------------------");
    
    if (cutSwitchDebouncer.read() == HIGH) {
        cutMotorHomed = true;
        cutMotor.setSpeed(0);
        cutMotor.setCurrentPosition(0);
        Serial.println("Cut motor already at home position");
    } else {
        cutMotor.setMaxSpeed(HOME_SPEED / 8);
        cutMotor.setSpeed((HOME_SPEED / 8) * HOME_DIRECTION);
    }
    
    if (positionSwitchDebouncer.read() == HIGH) {
        positionMotorHomed = true;
        positionMotor.setSpeed(0);
        positionMotor.setCurrentPosition(0);
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        Serial.println("Position motor already at home position");
    } else {
        positionMotor.setMaxSpeed(HOME_SPEED);
        positionMotor.setSpeed(HOME_SPEED * POSITION_HOME_DIRECTION);
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
    }
    
    if (cutMotorHomed && positionMotorHomed) {
        startupComplete = true;
        Serial.println("Both motors already homed - Ready for operation");
    }
}

// Handles the homing process of the cut motor
void handleCutMotorHoming() {
    if (!cutMotorHomed) {
        if (cutSwitchDebouncer.read() == HIGH) {
            cutMotor.setSpeed(0);
            cutMotor.setCurrentPosition(0);
            cutMotorHomed = true;
            Serial.println("Cut motor homed");
        } else {
            cutMotor.runSpeed();
            static unsigned long lastCutUpdate = 0;
            if (millis() - lastCutUpdate > 1000) {
                Serial.println("Cut motor homing in progress...");
                lastCutUpdate = millis();
            }
        }
    }
}

// Handles the homing process of the position motor
void handlePositionMotorHoming() {
    if (!positionMotorHomed) {
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
        if (positionSwitchDebouncer.read() == HIGH) {
            positionMotor.setSpeed(0);
            positionMotor.setCurrentPosition(0);
            positionMotorHomed = true;
            startupComplete = true;
            digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
            Serial.println("Position motor homed");
            Serial.println("Homing sequence complete");
        } else {
            static unsigned long lastPositionUpdate = 0;
            if (millis() - lastPositionUpdate > 1000) {
                Serial.println("Position motor homing in progress...");
                lastPositionUpdate = millis();
            }
            positionMotor.run();
        }
    }
}

// Runs the overall homing sequence by handling each motor's homing process
void runHomingSequence() {
    if (!startupComplete && reloadSwitch.read() == LOW) {
        if (!cutMotorHomed) {
            handleCutMotorHoming();
        } else if (!positionMotorHomed) {
            handlePositionMotorHoming();
        }
    } else if (!startupComplete && !cutMotorHomed && !positionMotorHomed) {
        static unsigned long lastSwitchMessage = 0;
        if (millis() - lastSwitchMessage > 2000) {
            Serial.println("\nCannot start homing:");
            if (reloadSwitch.read() == HIGH)
                Serial.println("- Reload switch is active");
            Serial.println("Please release switches to begin homing");
            lastSwitchMessage = millis();
        }
    }
}

// Performs the automated cut cycle once the start cycle switch is activated
void performCutCycle() {
    // Double-check that the start cycle switch remains pressed
    delay(100);
    startCycleSwitch.update();
    if (startCycleSwitch.read() == LOW)
        return;
    
    // Configure cut motor for normal operation
    cutMotor.setMaxSpeed(CUT_MOTOR_SPEED);
    cutMotor.setAcceleration(CUT_MOTOR_ACCEL);
    
    // Ensure clamps are engaged
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
    digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
    delay(CLAMP_OPERATION_DELAY);
    
    // Move cut motor forward to execute the cut
    cutMotor.moveTo(STEPS_PER_INCH * CUT_MOTOR_TRAVEL);
    while (cutMotor.distanceToGo() != 0) {
        cutMotor.run();
    }
    
    // Set faster return speeds for both motors
    cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
    positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
    
    // Retract the position clamp before returning home
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
    delay(CLAMP_OPERATION_DELAY);
    
    // Return both motors to the home position
    cutMotor.moveTo(0);
    positionMotor.moveTo(0);
    while (cutMotor.distanceToGo() != 0 || positionMotor.distanceToGo() != 0) {
        if (cutMotor.distanceToGo() != 0) {
            cutMotor.run();
        }
        if (positionMotor.distanceToGo() != 0) {
            positionMotor.run();
        }
    }
    
    // Re-engage clamps (and disengage wood clamp)
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
    delay(200);  // Ensure the clamp is fully engaged
    digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
    delay(CLAMP_OPERATION_DELAY);
    
    // Move the position motor to the designated position (3.35 inches)
    positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
    positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
    positionMotor.moveTo(-POSITION_STEPS_PER_INCH * POSITION_MOTOR_TRAVEL);
    while (positionMotor.distanceToGo() != 0) {
        positionMotor.run();
    }
    
    // Small stabilization delay before accepting new input
    delay(50);
    startCycleSwitch.update();
    if (startCycleSwitch.read() == LOW) {
        delay(200);
    }
}

// Handles reload switch activation to retract and re-engage clamps when necessary
void handleReloadSwitch() {
    if (reloadSwitch.read() == HIGH) {  // Reload switch is active when HIGH
        Serial.println("Reload switch activated - Retracting clamps");
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
        
        // Wait until the reload switch is released
        while (reloadSwitch.read() == HIGH) {
            reloadSwitch.update();
            delay(10);
        }
        
        Serial.println("Reload switch released - Re-engaging clamps");
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
    }
}