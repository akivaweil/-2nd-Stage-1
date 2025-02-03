#include <TaskScheduler.h>  // Added for non-blocking task scheduling

// ... [existing includes and declarations]

// Add TaskScheduler global instance and task declarations
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

// Forward declarations for task callback functions
int updateHomingTask();
int updateCutCycleTask();
int updateReloadTask();

// Create Task objects
// – Homing task runs every 50ms; it stays active until homing is complete.
Task taskHoming(50, TASK_FOREVER, updateHomingTask, &runner, true);

// – Cut cycle task is initially disabled; it will be enabled when the start switch is pressed.
Task taskCutCycle(50, TASK_FOREVER, updateCutCycleTask, &runner, false);

// – Reload task always runs to check the reload switch.
Task taskReload(50, TASK_FOREVER, updateReloadTask, &runner, true); 

// Task callback for homing sequence, called every 50ms
int updateHomingTask() {
    if (!startupComplete) {
        runHomingSequence();  // Already non-blocking: it calls handleCutMotorHoming() & handlePositionMotorHoming()
    }
    return TASK_RESCHEDULE;
}

// Non-blocking cut cycle state machine; called every 50ms by taskCutCycle.
int updateCutCycleTask() {
    switch(cycleStage) {
        case CYCLE_PREP:
            // Double-check the start-cycle switch (a short delay helps to stabilize)
            delay(10);
            startCycleSwitch.update();
            if (startCycleSwitch.read() == LOW) {
                cycleStage = CYCLE_NOT_STARTED;
                taskCutCycle.disable();
                break;
            }
            // Configure cut motor for normal operation
            cutMotor.setMaxSpeed(CUT_MOTOR_SPEED);
            cutMotor.setAcceleration(CUT_MOTOR_ACCEL);
            
            // Ensure clamps are engaged
            digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
            digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_ENGAGED);
            delay(CLAMP_OPERATION_DELAY);
            
            // Command the cut motor to move forward for the cut
            cutMotor.moveTo(STEPS_PER_INCH * CUT_MOTOR_TRAVEL);
            cycleStage = CYCLE_CUT_MOVEMENT;
            break;
        
        case CYCLE_CUT_MOVEMENT:
            // Non-blocking execution of the cutting movement
            if (cutMotor.distanceToGo() != 0) {
                cutMotor.run();
            } else {
                // When cutting is complete, prepare to return home:
                // Set faster return speeds for both motors …
                cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
                positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
                // Retract the position clamp before return movement
                digitalWrite(PIN_POSITION_CLAMP, CLAMP_DISENGAGED);
                delay(CLAMP_OPERATION_DELAY);
                // Command both motors to return to the home position
                cutMotor.moveTo(0);
                positionMotor.moveTo(0);
                cycleStage = CYCLE_RETURN;
            }
            break;
        
        case CYCLE_RETURN:
            // Run motors in non-blocking steps until both have returned home
            if (cutMotor.distanceToGo() != 0) {
                cutMotor.run();
            }
            if (positionMotor.distanceToGo() != 0) {
                positionMotor.run();
            }
            if (cutMotor.distanceToGo() == 0 && positionMotor.distanceToGo() == 0) {
                // Both motors are home – re-engage the clamps appropriately:
                digitalWrite(PIN_POSITION_CLAMP, CLAMP_ENGAGED);
                delay(200);  // Give time to ensure clamp engagement
                digitalWrite(PIN_SECURE_WOOD_CLAMP, CLAMP_DISENGAGED);
                delay(CLAMP_OPERATION_DELAY);
                // Command the position motor to move to the final position (e.g. 3.35 inches)
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
            // Final stabilization before ending cycle:
            delay(50);
            startCycleSwitch.update();
            if (startCycleSwitch.read() == LOW) {
                delay(200);
            }
            // Reset state and disable the cut cycle task until next trigger
            cycleStage = CYCLE_NOT_STARTED;
            taskCutCycle.disable();
            break;
            
        default:
            break;
    }
    
    return TASK_RESCHEDULE;
}

// Task callback for reload switch; called every 50ms.
int updateReloadTask() {
    if (reloadSwitch.read() == HIGH) {  // Reload switch is active when HIGH
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

void loop() {
    updateSwitches();  // Update all debounced switches
    runner.execute();  // Execute scheduled tasks
    
    // Check if the system is homed and the start cycle button is pressed.
    // Only trigger the cut cycle if not already running (cycleStage == CYCLE_NOT_STARTED)
    if (startupComplete && startCycleSwitch.read() == HIGH && cycleStage == CYCLE_NOT_STARTED) {
        cycleStage = CYCLE_PREP;
        taskCutCycle.restart();  // Enable (or restart) the cut cycle task
    }
} 