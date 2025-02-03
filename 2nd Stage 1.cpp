#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>
#include <TaskScheduler.h>

/*
 * AUTOMATED TABLE SAW CONTROL SYSTEM
 * 
 * SAFETY NOTICE: PLEASE DO NOT DELETE OR MODIFY ANYTHING HERE
 * This code controls an automated table saw cutting system. Safety is the absolute priority.
 * - Code clarity and reliability take precedence over processing efficiency
 * - All functions are written to be as explicit and straightforward as possible
 * - Hardware emergency stop switch cuts ALL power to the system when activated
 * - Multiple software safety checks are implemented throughout the cycle
 * - All switches and buttons read HIGH when activated
 * - All cylinders require a HIGH output to disengage
 * - Bounce2 library is used for switch debouncing with a 20ms debounce time
 * - All code should be very very easy to understand for a beginner programmer
 * - My switches are configured with one side connected to 5V and the other side connected to a 10k resistor pulling to ground
 */

// Pin Definitions - Motors (DO NOT MODIFY - Hardware Dependent)
const int PIN_CUT_MOTOR_PUL = 11;      // Cut motor pulse pin
const int PIN_CUT_MOTOR_DIR = 12;      // Cut motor direction pin
const int PIN_POSITION_MOTOR_PUL = 5;  // Position motor pulse pin
const int PIN_POSITION_MOTOR_DIR = 6;  // Position motor direction pin

// Pin Definitions - Safety Switches (DO NOT MODIFY - Hardware Dependent)
const int PIN_CUT_MOTOR_POSITION_SWITCH = 7;      // Limit switch for cut motor
const int PIN_POSITION_MOTOR_POSITION_SWITCH = 8;  // Limit switch for position motor
const int PIN_RELOAD_SWITCH = 9;                   // Material reload switch
const int PIN_START_CYCLE_SWITCH = 10;             // Switch to start main cycle

// Pin Definitions - Safety Clamps (DO NOT MODIFY - Hardware Dependent)
const int PIN_POSITION_CLAMP = 4;       // Clamp for maintaining position
const int PIN_SECURE_WOOD_CLAMP = 3;      // Clamp for securing material

// Clamp States - Safety Critical
const int CLAMP_ENGAGED = HIGH;         // Clamp extends to secure material
const int CLAMP_DISENGAGED = LOW;       // Clamp retracts to release material

// Homing Parameters - Safety Critical
const int HOME_SPEED = 2000;            // Homing movement speed
const int HOME_DIRECTION = -1;          // Cut motor homing direction
const int POSITION_HOME_DIRECTION = 1;  // Position motor homing direction
const int DEBOUNCE_INTERVAL = 20;       // Switch debounce time (milliseconds)

// Operation Parameters - Safety Critical (DO NOT MODIFY WITHOUT TESTING)
const int STEPS_PER_INCH = 127;         // Cut motor calibration (GT2 belt)
const int POSITION_STEPS_PER_INCH = 2000; // Position motor calibration (ball screw)
const float CUT_MOTOR_TRAVEL = 9;         // Maximum cut travel distance
const float POSITION_MOTOR_TRAVEL = 3.35; // Maximum position travel distance
const unsigned long CLAMP_OPERATION_DELAY = 50;  // Delay for clamp operation verification

// Motor Speed and Acceleration Limits - Safety Critical
const float CUT_MOTOR_SPEED = 150;              // Normal cutting speed
const float CUT_MOTOR_RETURN_SPEED = 1250;        // Return movement speed
const float POSITION_MOTOR_SPEED = 1000;          // Normal positioning speed
const float POSITION_MOTOR_RETURN_SPEED = 6000;   // Return movement speed
const float CUT_MOTOR_ACCEL = 2000;               // Cut motor acceleration limit
const float POSITION_MOTOR_ACCEL = 5000;          // Position motor acceleration limit

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

// --------------------------------------------------------------------
// Function Prototypes for setup and homing routines
// --------------------------------------------------------------------
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

// --------------------------------------------------------------------
// TaskScheduler Integration
// --------------------------------------------------------------------

// Global Scheduler instance
Scheduler runner;

// Enum for the cut cycle sub–states
enum CycleStage { 
    CYCLE_NOT_STARTED, 
    CYCLE_PREP, 
    CYCLE_CUT_MOVEMENT, 
    CYCLE_RETURN, 
    CYCLE_POSITIONING, 
    CYCLE_COMPLETE 
};
CycleStage cycleStage = CYCLE_NOT_STARTED;

// Forward declarations for TaskScheduler callback functions
int updateHomingTask();
int updateCutCycleTask();
int updateReloadTask();

// Create Task objects
// – Homing task runs every 50ms; active until homing is complete.
Task taskHoming(50, TASK_FOREVER, updateHomingTask, &runner, true);
// – Cut cycle task is initially disabled; will be enabled when the start switch is pressed.
Task taskCutCycle(50, TASK_FOREVER, updateCutCycleTask, &runner, false);
// – Reload task always runs to check the reload switch.
Task taskReload(50, TASK_FOREVER, updateReloadTask, &runner, true);

// Task callback for homing sequence (called every 50ms)
int updateHomingTask() {
    if (!startupComplete) {
        runHomingSequence();  // Calls handleCutMotorHoming() & handlePositionMotorHoming()
    }
    return TASK_RESCHEDULE;
}

// Non‑blocking cut cycle state machine (called every 50ms by taskCutCycle)
int updateCutCycleTask() {
    switch(cycleStage) {
        case CYCLE_PREP:
            // Short delay to stabilize and check the start cycle switch
            delay(10);
            startCycleSwitch.update();
            if (startCycleSwitch.read() == LOW) {
                cycleStage = CYCLE_NOT_STARTED;
                taskCutCycle.disable();
                break;
            }
            // Configure motor parameters for normal cutting operation
            cutMotor.setMaxSpeed(CUT_MOTOR_SPEED);
            cutMotor.setAcceleration(CUT_MOTOR_ACCEL);
            // Ensure clamps are engaged
            digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
            digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
            delay(CLAMP_OPERATION_DELAY);
            // Command the cut motor to move forward
            cutMotor.moveTo(STEPS_PER_INCH * CUT_MOTOR_TRAVEL);
            cycleStage = CYCLE_CUT_MOVEMENT;
            break;
            
        case CYCLE_CUT_MOVEMENT:
            // Non‑blocking execution of the cutting movement
            if (cutMotor.distanceToGo() != 0) {
                cutMotor.run();
            } else {
                // When cutting is complete, prepare to return home
                cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
                positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
                digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
                delay(CLAMP_OPERATION_DELAY);
                cutMotor.moveTo(0);
                positionMotor.moveTo(0);
                cycleStage = CYCLE_RETURN;
            }
            break;
            
        case CYCLE_RETURN:
            // Run motors until both return to home
            if (cutMotor.distanceToGo() != 0) {
                cutMotor.run();
            }
            if (positionMotor.distanceToGo() != 0) {
                positionMotor.run();
            }
            if (cutMotor.distanceToGo() == 0 && positionMotor.distanceToGo() == 0) {
                // Re‑engage clamps and command the position motor to the final position
                digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
                delay(200);
                digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
                delay(CLAMP_OPERATION_DELAY);
                positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
                positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
                positionMotor.moveTo(-POSITION_STEPS_PER_INCH * POSITION_MOTOR_TRAVEL);
                cycleStage = CYCLE_POSITIONING;
            }
            break;
            
        case CYCLE_POSITIONING:
            if (positionMotor.distanceToGo() != 0) {
                positionMotor.run();
            } else {
                cycleStage = CYCLE_COMPLETE;
            }
            break;
            
        case CYCLE_COMPLETE:
            // Final stabilization before ending the cycle
            delay(50);
            startCycleSwitch.update();
            if (startCycleSwitch.read() == LOW) {
                delay(200);
            }
            cycleStage = CYCLE_NOT_STARTED;
            taskCutCycle.disable();
            break;
            
        default:
            break;
    }
    return TASK_RESCHEDULE;
}

// Task callback for the reload switch (called every 50ms)
int updateReloadTask() {
    if (reloadSwitch.read() == HIGH) {  // Reload switch active
        Serial.println("Reload switch activated - Retracting clamps");
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
    } else {
        // Re-engage clamps after reload
        digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
        digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
    }
    return TASK_RESCHEDULE;
}

// --------------------------------------------------------------------
// Main Setup and Loop
// --------------------------------------------------------------------
void setup() {
    // Configure serial communication, switches, motors, and clamps
    configureSerial();
    configureSwitches();
    configureMotorsForHoming();
    configureClamps();
    printSetupComplete();
}

void loop() {
    updateSwitches();
    runner.execute();
    
    // If homing is complete and the start switch is pressed, trigger the cut cycle.
    if (startupComplete && startCycleSwitch.read() == HIGH && cycleStage == CYCLE_NOT_STARTED) {
        cycleStage = CYCLE_PREP;
        taskCutCycle.restart();  // Enable/restart the cut cycle task
    }
}

// --------------------------------------------------------------------
// Configuration and Helper Functions
// --------------------------------------------------------------------

// Initializes serial communication and prints startup messages.
void configureSerial() {
    Serial.begin(115200);
    delay(1000);  // Allow time for serial initialization
    Serial.println("----------------------------------------");
    Serial.println("Serial communication initialized");
    Serial.println("AUTOMATED TABLE SAW CONTROL SYSTEM");
    Serial.println("----------------------------------------");
}

// Configures debounced switches.
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

// Sets up the motors for homing behavior.
void configureMotorsForHoming() {
    cutMotor.setMaxSpeed(HOME_SPEED / 8);  // Slower speed for homing
    cutMotor.setSpeed((HOME_SPEED / 8) * HOME_DIRECTION);
    Serial.println("Cut motor configured for homing");
    
    positionMotor.setMaxSpeed(HOME_SPEED / 2);
    positionMotor.setAcceleration(POSITION_MOTOR_ACCEL);
    positionMotor.moveTo(100000 * POSITION_HOME_DIRECTION);
    Serial.println("Position motor configured for homing");
}

// Configures clamp pins as outputs and sets their initial states.
void configureClamps() {
    pinMode(PIN_POSITION_CLAMP, OUTPUT);
    pinMode(PIN_SECURE_WOOD_CLAMP, OUTPUT);
    
    digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);  // Start with position clamp retracted
    digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);    // Wood clamp engaged
    Serial.println("Clamps configured - Position clamp retracted, wood clamp engaged");
}

// Prints a message indicating completion of the setup.
void printSetupComplete() {
    Serial.println("----------------------------------------");
    Serial.println("Setup complete - Beginning homing sequence");
    Serial.println("----------------------------------------");
}

// Updates the state of all debounced switches.
void updateSwitches() {
    cutSwitchDebouncer.update();
    positionSwitchDebouncer.update();
    reloadSwitch.update();
    startCycleSwitch.update();
}

// Reports the initial state of switches and checks for pre-homed motors.
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

// Handles the homing process of the cut motor.
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

// Handles the homing process of the position motor.
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

// Runs the overall homing sequence by invoking motor-specific homing routines.
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