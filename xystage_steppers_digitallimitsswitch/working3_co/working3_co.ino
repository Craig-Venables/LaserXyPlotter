#include <AccelStepper.h>
#include <MultiStepper.h>

float stepsPerRevolution = 1600.0;
float distance_per_rotation = 8.0;
int carriagespeed = 5000;
int acceleration = 2000;
float xpos = 0.0;
float ypos = 0.0;

const int enablepin = 2;
const int xdirpin = 3;
const int xsteppin = 4;
const int xhomepin = 5;
const int xendpin = 6;
const int ydirpin = 7;
const int ysteppin = 8;
const int yhomepin = 9;
const int yendpin = 10;
const int laserpin = 11;
const int bufferSize = 5;
String commandBuffer[bufferSize];
int bufferHead = 0;
int bufferTail = 0;

int commandsProcessed = 0;
const int maxBatchSize = 5;

long positions[2]; // Global array

AccelStepper stepperx(1, xsteppin, xdirpin);
AccelStepper steppery(1, ysteppin, ydirpin);
MultiStepper steppers;

void setup() {
    Serial.begin(115200);
    stepperx.setMaxSpeed(carriagespeed);
    steppery.setMaxSpeed(carriagespeed);
    steppers.addStepper(stepperx);
    steppers.addStepper(steppery);

    pinMode(xhomepin, INPUT);
    pinMode(xendpin, INPUT);
    pinMode(yhomepin, INPUT);
    pinMode(yendpin, INPUT);
    pinMode(enablepin, OUTPUT);
    pinMode(laserpin, OUTPUT);
    digitalWrite(enablepin, LOW);
    digitalWrite(laserpin, LOW);
}

void loop() {
    // Receive serial input and store in buffer
    if (Serial.available() > 0) {
        String incomingString = Serial.readStringUntil('\n');
        commandBuffer[bufferHead] = incomingString;
        bufferHead = (bufferHead + 1) % bufferSize;

        // Prevent buffer overrun
        if (bufferHead == bufferTail) {
            bufferTail = (bufferTail + 1) % bufferSize;
        }
    }

    // Process up to maxBatchSize commands
    commandsProcessed = 0;
    while (bufferTail != bufferHead && commandsProcessed < maxBatchSize) {
        String command = commandBuffer[bufferTail];
        bufferTail = (bufferTail + 1) % bufferSize;

        processCommand(command);
        commandsProcessed++;
    }

    // Let Python know we're ready for the next batch
    if (commandsProcessed > 0) {
        Serial.println("READY");
    }
}

bool xPositiveAllowed = true;
bool xNegativeAllowed = true;
bool yPositiveAllowed = true;
bool yNegativeAllowed = true;

bool handleLimitSwitches() {
    bool limitReached = false;
    
    if (digitalRead(xendpin) == HIGH) {
        stepperx.stop();
        xNegativeAllowed = false;
        Serial.println("X-End limit reached: Negative movement stopped");
        limitReached = true;
    }
    if (digitalRead(xhomepin) == HIGH) {
        stepperx.stop();
        xPositiveAllowed = false;
        Serial.println("X-Home limit reached: Positive movement stopped");
        limitReached = true;
    }

    if (digitalRead(yendpin) == HIGH) {
        steppery.stop();
        yNegativeAllowed = false;
        Serial.println("Y-End limit reached: Negative movement stopped");
        limitReached = true;
    }
    if (digitalRead(yhomepin) == HIGH) {
        steppery.stop();
        yPositiveAllowed = false;
        Serial.println("Y-Home limit reached: Positive movement stopped");
        limitReached = true;
    }
    
    return limitReached;
}


void processCommand(String commandString) {
    String comstring = commandString.substring(0, 5);   

    if (comstring == "HOMEX") {
        //4572 LONG    
    stepperx.setAcceleration(10000); // Set high acceleration for immediate stop
    
    // Move to the lower limit
    positions[0] = 15000;
    positions[1] = 0;
    steppers.moveTo(positions);
    digitalWrite(enablepin, HIGH);
    while (digitalRead(xhomepin) == LOW && stepperx.isRunning()) {
        stepperx.run();
        if (digitalRead(xhomepin) == HIGH) {
            stepperx.stop(); // Immediately stop if limit switch is triggered
            stepperx.setCurrentPosition(0);
        }
    }

    int lowerLimit = stepperx.currentPosition();

    // Move to the upper limit
    positions[0] = -15000;
    steppers.moveTo(positions);
    while (digitalRead(xendpin) == LOW && stepperx.isRunning()) {
        stepperx.run();
        if (digitalRead(xendpin) == HIGH) {
            stepperx.stop(); // Immediately stop if limit switch is triggered
        }
    }

    int upperLimit = stepperx.currentPosition();

    // Calculate the midpoint
    int midpoint = (lowerLimit + upperLimit) / 2;
    Serial.println(lowerLimit);
    Serial.println(upperLimit);
    stepperx.moveTo(midpoint);
    while (stepperx.isRunning()) {
        stepperx.run();
    }
    xpos = midpoint * distance_per_rotation / stepsPerRevolution;
    ypos = 0;
    Serial.print("Midpoint X: ");
    Serial.print(xpos);
    Serial.print(", Y: ");
    Serial.println(ypos);
    stepperx.setCurrentPosition(midpoint);
    steppery.setCurrentPosition(0);
    stepperx.setAcceleration(acceleration); // Restore original acceleration

    }else if (comstring == "HOMEY") {
        //4300 Long
    steppery.setAcceleration(10000); // Set to a very high acceleration value to effectively disable deceleration

    // Move to the lower limit
    positions[0] = 0;
    positions[1] = 15000;
    steppers.moveTo(positions);
    digitalWrite(enablepin, HIGH);
    while (digitalRead(yhomepin) == LOW && steppery.isRunning()) {
        steppery.run();
        if (digitalRead(yhomepin) == HIGH) {
            steppery.stop(); // Immediately stop if limit switch is triggered
            steppery.setCurrentPosition(0);
        }
    }
    int lowerLimit = steppery.currentPosition();
    // Move to the upper limit
    positions[1] = -15000;
    steppers.moveTo(positions);
    digitalWrite(enablepin, HIGH);
    while (digitalRead(yendpin) == LOW && steppery.isRunning()) {
        steppery.run();
        if (digitalRead(yendpin) == HIGH) {
            steppery.stop(); // Immediately stop if limit switch is triggered
        }
    }
    
    int upperLimit = steppery.currentPosition();
    int midpoint = (lowerLimit + upperLimit) / 2;// Calculate the midpoint
    Serial.println(lowerLimit);
    Serial.println(upperLimit);
    steppery.moveTo(midpoint);
    while (steppery.isRunning()) {
        steppery.run();
    }

    
    
    ypos = midpoint * distance_per_rotation / stepsPerRevolution;
    xpos = 0;
    Serial.print("Midpoint Y: ");
    Serial.print(ypos);
    Serial.print(", X: ");
    Serial.println(xpos);

    steppery.setCurrentPosition(midpoint);
    stepperx.setCurrentPosition(0);

    // Restore original acceleration
    steppery.setAcceleration(acceleration);

    } else if (comstring == "SPEED") {
        int speedstringindex = commandString.indexOf(' ');
        String speedstring = commandString.substring(speedstringindex + 1);
        float speed = speedstring.toFloat() * stepsPerRevolution / distance_per_rotation;
        carriagespeed = (int)speed;

    } else if (comstring == "ACCEL") {
        int accelstringindex = commandString.indexOf(' ');
        String accelstring = commandString.substring(accelstringindex + 1);
        float accel = accelstring.toFloat();
        acceleration = (int)accel;

    } else if (comstring == "LASON") {
        int powerstringindex = commandString.indexOf(' ');
        String powerstring = commandString.substring(powerstringindex + 1);
        int laserpower = powerstring.toInt();
        if (laserpower > 255) {
            laserpower = 255;
        }
        analogWrite(laserpin, laserpower);
        //Serial.println("Laser on");

    } else if (comstring == "LASOF") {
        digitalWrite(laserpin, LOW);
        //Serial.println("Laser off");
    } else if (comstring == "SETHO") {
        xpos = 0.0;
        ypos = 0.0;
        Serial.println("Home position set");
        Serial.println(xpos);
        Serial.print(',');
        Serial.print(ypos);
       
    } else if (comstring == "ENDSX") {
        stepperx.setMaxSpeed(carriagespeed);
        stepperx.setAcceleration(acceleration);
        positions[0] = -15000;
        positions[1] = 0;
        steppers.moveTo(positions);
        digitalWrite(enablepin, HIGH);
        while (digitalRead(xendpin) == LOW) {
            stepperx.run();
        }
        stepperx.setCurrentPosition(0);
        steppery.setCurrentPosition(0);
        digitalWrite(enablepin, LOW);
        Serial.println("X motor moved to end stop");
    } else if (comstring == "ENDSY") {
        steppery.setMaxSpeed(carriagespeed);
        steppery.setAcceleration(acceleration);
        positions[0] = 0;
        positions[1] = -15000;
        steppers.moveTo(positions);
        digitalWrite(enablepin, HIGH);
        while (digitalRead(yendpin) == LOW) {
            steppery.run();
        }
        steppery.setCurrentPosition(0);
        stepperx.setCurrentPosition(0);
        Serial.println("Y motor moved to end stop");

    } else if (comstring == "MOVEX") {
    stepperx.setMaxSpeed(carriagespeed);  // Use setMaxSpeed, not setSpeed
    stepperx.setAcceleration(acceleration);
    
    int movestringindex = commandString.indexOf(' ');
    String movestring = commandString.substring(movestringindex + 1);
    float newxpos = movestring.toFloat();
    int xsteps = (newxpos - xpos) * stepsPerRevolution / distance_per_rotation;

    // Check direction flags
    if (xsteps > 0 && !xPositiveAllowed) xsteps = 0;
    if (xsteps < 0 && !xNegativeAllowed) xsteps = 0;

    positions[0] = -xsteps;
    positions[1] = 0;
    steppers.moveTo(positions);
    
    digitalWrite(enablepin, HIGH);

    // Use runSpeedToPosition for smoother movement
    while (steppers.run()) {
        // Only check limit switches periodically, not every iteration
        static unsigned long lastCheck = 0;
        if (millis() - lastCheck > 10) {  // Check every 10ms
            lastCheck = millis();
            if (handleLimitSwitches()) {
                break;
            }
        }
    }

    // Update position
    int stepsCompleted = xsteps - stepperx.distanceToGo();
    xpos += stepsCompleted * distance_per_rotation / stepsPerRevolution;
    
    Serial.print("POS:");
    Serial.print(xpos);
    Serial.print(',');
    Serial.println(ypos);
    
    stepperx.setCurrentPosition(0);
    steppery.setCurrentPosition(0);
} else if (comstring == "MOVEY") {
    steppery.setMaxSpeed(carriagespeed);  // Use setMaxSpeed, not setSpeed
    steppery.setAcceleration(acceleration);
    
    int movestringindex = commandString.indexOf(' ');
    String movestring = commandString.substring(movestringindex + 1);
    float newypos = movestring.toFloat();
    float ysteps = (newypos - ypos) * stepsPerRevolution / distance_per_rotation;

    // Check direction flags
    if (ysteps > 0 && !yPositiveAllowed) ysteps = 0;
    if (ysteps < 0 && !yNegativeAllowed) ysteps = 0;

    positions[0] = 0;
    positions[1] = -ysteps;
    steppers.moveTo(positions);
    
    digitalWrite(enablepin, HIGH);

    // Execute movement
    while (steppers.run()) {
        // Only check limit switches periodically, not every iteration
        static unsigned long lastCheck = 0;
        if (millis() - lastCheck > 10) {  // Check every 10ms
            lastCheck = millis();
            if (handleLimitSwitches()) {
                break;
            }
        }
    } 
    // Update position based on actual steps completed
    int stepsCompleted = ysteps - steppery.distanceToGo();
    ypos += stepsCompleted * distance_per_rotation / stepsPerRevolution;
    
    Serial.print("POS:");
    Serial.print(xpos);
    Serial.print(',');
    Serial.println(ypos);

    steppery.setCurrentPosition(0);
    stepperx.setCurrentPosition(0);
    }
    else if (comstring == "MOVXY") {
    // Set movement parameters
    stepperx.setMaxSpeed(carriagespeed);
    steppery.setMaxSpeed(carriagespeed);
    stepperx.setAcceleration(acceleration);
    steppery.setAcceleration(acceleration);

    // Extract the command string after "MOVXY"
    int movestringindex = commandString.indexOf(' ');
    String movestring = commandString.substring(movestringindex + 1);

    // Parse x, y, and laser power
    int movestringindex2 = movestring.indexOf(',');
    String xmovestring = movestring.substring(0, movestringindex2);
    movestring = movestring.substring(movestringindex2 + 1);

    int movestringindex3 = movestring.indexOf(',');
    String ymovestring = movestring.substring(0, movestringindex3);
    String powerstring = movestring.substring(movestringindex3 + 1);
    int laserpower = powerstring.toInt();

    // Convert to floats
    float newxpos = xmovestring.toFloat();
    float newypos = ymovestring.toFloat();

    // Convert to steps
    int xsteps = (newxpos - xpos) * stepsPerRevolution / distance_per_rotation;
    int ysteps = (newypos - ypos) * stepsPerRevolution / distance_per_rotation;

    // Safety: prevent movement if direction is blocked
    if (xsteps > 0 && !xPositiveAllowed) xsteps = 0;
    if (xsteps < 0 && !xNegativeAllowed) xsteps = 0;
    if (ysteps > 0 && !yPositiveAllowed) ysteps = 0;
    if (ysteps < 0 && !yNegativeAllowed) ysteps = 0;

    // Set new target positions
    positions[0] = -xsteps;
    positions[1] = -ysteps;
    steppers.moveTo(positions);

    // Enable motors and set laser
    digitalWrite(enablepin, HIGH);
    if (laserpower > 255) {
        laserpower = 255;
    }
    analogWrite(laserpin, laserpower);

    // Execute movement until target reached or limit hit
    while (steppers.run()) {
        // Only check limit switches periodically, not every iteration
        static unsigned long lastCheck = 0;
        if (millis() - lastCheck > 10) {  // Check every 10ms
            lastCheck = millis();
            if (handleLimitSwitches()) {
                break;
            }
        }
    }

    // Update logical position based on actual steps completed
    int xstepsCompleted = xsteps - stepperx.distanceToGo();
    int ystepsCompleted = ysteps - steppery.distanceToGo();
    
    xpos += xstepsCompleted * distance_per_rotation / stepsPerRevolution;
    ypos += ystepsCompleted * distance_per_rotation / stepsPerRevolution;

    // Print final position for logging/debugging
    Serial.print("POS:");
    Serial.print(xpos);
    Serial.print(',');
    Serial.println(ypos);

    // Reset internal step counter positions
    stepperx.setCurrentPosition(0);
    steppery.setCurrentPosition(0);

    // Turn off laser after movement
    digitalWrite(laserpin, LOW);

    // Notify host that this command is finished
    Serial.println("READY");
}}