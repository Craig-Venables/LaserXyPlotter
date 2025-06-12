#include <AccelStepper.h>
#include <MultiStepper.h>

float stepsPerRevolution = 200.0;
float distance_per_rotation = 8.0;
int carriagespeed = 800;
int acceleration = 500;
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
const int bufferSize = 50;
String commandBuffer[bufferSize];
int bufferHead = 0;
int bufferTail = 0;

long positions[2]; // Global array

AccelStepper stepperx(1, xsteppin, xdirpin);
AccelStepper steppery(1, ysteppin, ydirpin);
MultiStepper steppers;

void setup() {
    Serial.begin(9600);
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
    digitalWrite(enablepin, HIGH);
    digitalWrite(laserpin, LOW);
}

void loop() {
    if (Serial.available() > 0) {
        String incomingString = Serial.readStringUntil('\n');
        commandBuffer[bufferHead] = incomingString;
        bufferHead = (bufferHead + 1) % bufferSize;

        // Avoid overwriting the buffer
        if (bufferHead == bufferTail) {
            bufferTail = (bufferTail + 1) % bufferSize; // Drop the oldest command
        }
    }

    while (bufferTail != bufferHead) {
        String command = commandBuffer[bufferTail];
        bufferTail = (bufferTail + 1) % bufferSize;

        // Process the command
        processCommand(command);
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
        

        if (xsteps < 0) {
            stepperx.setSpeed(carriagespeed);
        } else {
            stepperx.setSpeed(-carriagespeed);
        }
        digitalWrite(enablepin, HIGH);

        while (steppers.run()) {
        if (handleLimitSwitches()) {
            break; // Exit the loop if a limit switch is triggered
        }
}
        //steppers.runSpeedToPosition();

        if (xsteps < 0){
        xpos=xpos-(abs(xsteps)-abs(stepperx.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
      }
      else{
        xpos=xpos+(abs(xsteps)-abs(stepperx.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
      }
          

        //xpos += (abs(xsteps) - abs(stepperx.distanceToGo())) * distance_per_rotation / stepsPerRevolution;
        
        Serial.print(xpos);
        Serial.print(',');
        Serial.print(ypos);
        stepperx.setCurrentPosition(0);
        steppery.setCurrentPosition(0);
    } else if (comstring == "MOVEY") {
        steppery.setSpeed(carriagespeed);
        steppery.setAcceleration(acceleration);
        int movestringindex = commandString.indexOf(' ');
        String movestring = commandString.substring(movestringindex + 1);
        float newypos = movestring.toFloat();
        float ysteps = (newypos - ypos) * stepsPerRevolution / distance_per_rotation;

        if (ysteps > 0 && !yPositiveAllowed) ysteps = 0;
        if (ysteps < 0 && !yNegativeAllowed) ysteps = 0;

        positions[0] = 0;
        positions[1] = -ysteps;
        steppers.moveTo(positions);
        if (ysteps < 0) {
            steppery.setSpeed(carriagespeed);
        } else {
            steppery.setSpeed(-carriagespeed);
        }
        digitalWrite(enablepin, HIGH);

        while (steppers.run()) {
            if (handleLimitSwitches()) {
                break;
            }
        }
        //steppers.runSpeedToPosition();

        if (ysteps < 0){
        ypos=ypos-(abs(ysteps)-abs(steppery.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
        }
        else{
        ypos=ypos+(abs(ysteps)-abs(steppery.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
        }

        //ypos += (abs(ysteps) - abs(steppery.distanceToGo())) * distance_per_rotation / stepsPerRevolution;
        
        Serial.print("POS:");
    Serial.print(xpos);
    Serial.print(',');
    Serial.println(ypos);
        //Serial.println('');

        steppery.setCurrentPosition(0);
        stepperx.setCurrentPosition(0);
    } 
    else if (comstring == "MOVXY") {
    stepperx.setMaxSpeed(carriagespeed);
    steppery.setMaxSpeed(carriagespeed);

    int movestringindex = commandString.indexOf(' ');
    String movestring = commandString.substring(movestringindex + 1);
    int movestringindex2 = movestring.indexOf(',');
    String xmovestring = movestring.substring(0, movestringindex2);
    movestring = movestring.substring(movestringindex2 + 1);
    int movestringindex3 = movestring.indexOf(',');
    String ymovestring = movestring.substring(0, movestringindex3);
    String powerstring = movestring.substring(movestringindex3 + 1);
    int laserpower = powerstring.toInt();

    float newxpos = xmovestring.toFloat();
    float newypos = ymovestring.toFloat();
    int xsteps = (newxpos - xpos) * stepsPerRevolution / distance_per_rotation;
    int ysteps = (newypos - ypos) * stepsPerRevolution / distance_per_rotation;


    if (xsteps > 0 && !xPositiveAllowed) xsteps = 0;
    if (xsteps < 0 && !xNegativeAllowed) xsteps = 0;
    if (ysteps > 0 && !yPositiveAllowed) ysteps = 0;
    if (ysteps < 0 && !yNegativeAllowed) ysteps = 0;

    positions[0] = -xsteps;
    positions[1] = -ysteps;
    steppers.moveTo(positions);

    if (xsteps < 0)
        stepperx.setSpeed(carriagespeed);
    else
        stepperx.setSpeed(-carriagespeed);
    
    if (ysteps < 0)
        steppery.setSpeed(carriagespeed);
    else
        steppery.setSpeed(-carriagespeed);

    digitalWrite(enablepin, HIGH);
    analogWrite(laserpin, laserpower);
    while (steppers.run()) {
            if (handleLimitSwitches()) {
                break;
            }
        }
    
    if (xsteps < 0){
    xpos=xpos-(abs(xsteps)-abs(stepperx.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
    }
    else{
    xpos=xpos+(abs(xsteps)-abs(stepperx.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
    }

    if (ysteps < 0){
    ypos=ypos-(abs(ysteps)-abs(steppery.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
    }
    else{
    ypos=ypos+(abs(ysteps)-abs(steppery.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
    }

    // xpos += (abs(xsteps) - abs(stepperx.distanceToGo())) * distance_per_rotation / stepsPerRevolution;
    // ypos += (abs(ysteps) - abs(steppery.distanceToGo())) * distance_per_rotation / stepsPerRevolution;
    
    Serial.print(xpos);
    Serial.print(',');
    Serial.print(ypos);
    // Serial.print(',');
    // Serial.println(laserpower);
    stepperx.setCurrentPosition(0);
    steppery.setCurrentPosition(0);
}}