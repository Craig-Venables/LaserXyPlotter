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
const int bufferSize = 10;
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

    

void processCommand(String commandString) {
      String comstring = commandString.substring(0, 5);   
      
      if (comstring == "HOMEX") {                                                 //if stage 1 home requested
        stepperx.setMaxSpeed(carriagespeed);  // set motor speed 
        stepperx.setAcceleration(acceleration);  // set motor speed 
        positions[0] = 15000;
        positions[1] = 0;
        steppers.moveTo(positions); // set move value to be large
        digitalWrite(enablepin,HIGH);  //enable outputs
        // delay(2000);
        while (digitalRead(xhomepin)==LOW){  // check to see if limit switch
          // if not move motors
          stepperx.run();
        }

        stepperx.setCurrentPosition(0); // stops the stepper turning in the wrong direction when a new move is called
        steppery.setCurrentPosition(0);
        //stepperx.move(-5);   // reverse direction of stepper motor - give a smoother pull as it prevents motor from moving in other direction initially (backlash)
        //while (abs(stepperx.distanceToGo()) > 0){
        //  stepperx.run();
        //} 
        
        xpos=0.0;        
        //Serial.println(xpos);
    }
    
    else  if (comstring == "HOMEY") {
      steppery.setMaxSpeed(carriagespeed);  // set motor speed 
      steppery.setAcceleration(acceleration);  // set motor speed 
      positions[0] = 0;
      positions[1] = 15000;
      steppers.moveTo(positions);
      
         // set move value to be large
      digitalWrite(enablepin,HIGH);  //enable outputs
      // delay(2000);
         
             
      while (digitalRead(yhomepin) == LOW){  // check to see if limit switch
        // if not move motors
        steppery.run(); // run motors
      }


      steppery.setCurrentPosition(0); // stops the stepper turning in the wrong direction when a new move is called
      stepperx.setCurrentPosition(0); 

       ypos=0.0;         
       //Serial.println(ypos);
    }
    else if (comstring=="SPEED"){    // turn laser on at the specified power
      int speedstringindex=commandString.indexOf(' ');
      String speedstring=commandString.substring(speedstringindex+1);
      Serial.println(speedstring);
      float speed=speedstring.toFloat()*stepsPerRevolution/distance_per_rotation;
      carriagespeed=(int)speed;    // speed in steps per second
      //Serial.println(carriagespeed);
    }
    else if (comstring=="ACCEL"){    // turn laser on at the specified power
      int accelstringindex=commandString.indexOf(' ');
      String accelstring=commandString.substring(accelstringindex+1);
      //Serial.print(powerstring);
      float accel=accelstring.toFloat();
      acceleration=(int)accel;    // Accel in steps per second

        
    }
    else if (comstring=="LASON"){    // turn laser on at the specified power
      int powerstringindex=commandString.indexOf(' ');
      String powerstring=commandString.substring(powerstringindex+1);
      //Serial.print(powerstring);
      int laserpower=powerstring.toInt();

      if (laserpower > 255){laserpower=255;} 
      
      analogWrite(laserpin, laserpower);
      Serial.println("Laser on");
        
    }
    else if (comstring=="LASOF"){
      digitalWrite(laserpin, LOW);
      Serial.println("Laser off");
    }
    else if (comstring=="SETHO"){
      xpos=0.0;
      ypos=0.0;
      Serial.println("Home position set");
    }
    else if (comstring=="ENDSX"){
      stepperx.setMaxSpeed(carriagespeed);  // set motor speed 
      stepperx.setAcceleration(acceleration);  // set motor speed 
      positions[0] = -15000;
      positions[1] = 0;
      steppers.moveTo(positions);
      

         // set move value to be large
      digitalWrite(enablepin,HIGH);  //enable outputs
      // delay(2000);
         
      while (digitalRead(xendpin) == LOW){  // check to see if limit switch
        // if not move motors
        stepperx.run();  // run motors
      }


      stepperx.setCurrentPosition(0); // stops the stepper turning in the wrong direction when a new move is called 
      steppery.setCurrentPosition(0);
       digitalWrite(enablepin,LOW);  //disable outputs        
       Serial.println("X motor moved to end stop");
    }
    else if (comstring=="ENDSY"){
      steppery.setMaxSpeed(carriagespeed);  // set motor speed 
      steppery.setAcceleration(acceleration);  // set motor speed   
      positions[0] = 0;
      positions[1] = -15000;
      steppers.moveTo(positions);
      

         // set move value to be large
      digitalWrite(enablepin,HIGH);  //enable outputs
      // delay(2000);
         
      while (digitalRead(yendpin) == LOW){  // check to see if limit switch
        // if not move motors
        steppery.run();  // run motors
      }


      steppery.setCurrentPosition(0); // stops the stepper turning in the wrong direction when a new move is called
      stepperx.setCurrentPosition(0);  
       Serial.println("Y motor moved to end stop");
    }
    
    else if (comstring=="MOVEX"){
      
      stepperx.setAcceleration(acceleration);  // set motor speed 
      int movestringindex=commandString.indexOf(' ');
      String movestring=commandString.substring(movestringindex+1);
      
      //Serial.print(movestring);
      float newxpos=movestring.toFloat();
      int xsteps=(newxpos-xpos)*stepsPerRevolution/distance_per_rotation;  // required number of steps to move
      //Serial.println(xsteps);

      positions[0] = -xsteps;
      positions[1] = 0;
      steppers.moveTo(positions);
      //stepperx.setSpeed(carriagespeed);
      if (xsteps < 0){
          stepperx.setSpeed(carriagespeed);  // set motor speed 
      }
      else{
          stepperx.setSpeed(-carriagespeed);  // set motor speed 
      }

         // set move value to be large
      digitalWrite(enablepin,HIGH);  //enable outputs
      // delay(2000);

      
      //while (abs(stepperx.distanceToGo()) > 0 && digitalRead(xhomepin) == LOW && digitalRead(xendpin) == LOW){
 
      steppers.runSpeedToPosition();
      //} 

      if (xsteps < 0){
        xpos=xpos-(abs(xsteps)-abs(stepperx.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
      }
      else{
        xpos=xpos+(abs(xsteps)-abs(stepperx.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
      }
          
      Serial.println(xpos);
       //Serial.println((xsteps-abs(stepperx.distanceToGo())));
       stepperx.setCurrentPosition(0); // stops the stepper turning in the wrong direction when a new move is called  
       steppery.setCurrentPosition(0);   
    }            
     else if (comstring=="MOVEY"){
      steppery.setSpeed(carriagespeed);  // set motor speed 
      steppery.setAcceleration(acceleration);  // set motor speed 
      int movestringindex=commandString.indexOf(' ');
      String movestring=commandString.substring(movestringindex+1);
      float newypos=movestring.toFloat();
      float ysteps=(newypos-ypos)*stepsPerRevolution/distance_per_rotation;  // required number of steps to move
      
      positions[0] = 0;
      positions[1] = -ysteps;
      steppers.moveTo(positions);
      
      if (ysteps < 0){
          steppery.setSpeed(carriagespeed);  // set motor speed 
      }
      else{
          steppery.setSpeed(-carriagespeed);  // set motor speed 
      }

      // delay(2000);     
      digitalWrite(enablepin,HIGH);  //enable outputs
      // delay(2000);

      
      //steppery.move(+5);   // reverse direction of stepper motor - give a smoother pull as it prevents motor from moving in other direction initially (backlash)
      //while (abs(steppery.distanceToGo()) > 0 && digitalRead(yhomepin) == LOW && digitalRead(yendpin) == LOW){
        steppers.runSpeedToPosition();
      //} 

      if (ysteps < 0){
        ypos=ypos-(abs(ysteps)-abs(steppery.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
      }
      else{
        ypos=ypos+(abs(ysteps)-abs(steppery.distanceToGo()))*distance_per_rotation/stepsPerRevolution;
      }
       Serial.println(ypos);
       steppery.setCurrentPosition(0); // stops the stepper turning in the wrong direction when a new move is called
       stepperx.setCurrentPosition(0);        
     }
     else if (comstring=="MOVXY"){
      stepperx.setMaxSpeed(carriagespeed);  // set motor speed 
      steppery.setMaxSpeed(carriagespeed);  // set motor speed 
      int movestringindex=commandString.indexOf(' ');
      String movestring=commandString.substring(movestringindex+1);

      int movestringindex2=movestring.indexOf(',');
      String xmovestring=movestring.substring(0,movestringindex2);
      String ymovestring=movestring.substring(movestringindex2+1);
      
      int powerstringindex=movestring.lastIndexOf(',');    // get the laser power setting
      String powerstring=movestring.substring(powerstringindex+1);
      int laserpower=powerstring.toInt();
      //Serial.print(movestring);
      float newxpos=xmovestring.toFloat();
      float newypos=ymovestring.toFloat();
      int xsteps=(newxpos-xpos)*stepsPerRevolution/distance_per_rotation;  // required number of steps to move
      int ysteps=(newypos-ypos)*stepsPerRevolution/distance_per_rotation;
      //Serial.println(xsteps);
      //Serial.println(ysteps);
       
      positions[0] = -xsteps;
      positions[1] = -ysteps;
      steppers.moveTo(positions);
      
      if (xsteps < 0){
          stepperx.setSpeed(carriagespeed);  // set motor speed 
      }
      else{
          stepperx.setSpeed(-carriagespeed);  // set motor speed 
      }
      if (ysteps < 0){
          steppery.setSpeed(carriagespeed);  // set motor speed 
      }
      else{
          steppery.setSpeed(-carriagespeed);  // set motor speed 
      }

      digitalWrite(enablepin,HIGH);  //enable outputs
      // delay(2000);
      analogWrite(laserpin,laserpower);  // power the laser
      steppers.runSpeedToPosition();   // move the motors by the desired number of steps
      //while ((abs(stepperx.distanceToGo()) > 0 || (abs(steppery.distanceToGo()) > 0)) && digitalRead(xhomepin) == LOW && digitalRead(xendpin) == LOW && digitalRead(yhomepin) == LOW && digitalRead(yendpin) == LOW){
        
        //steppers.runSpeedToPosition();
      //} 

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
          
      Serial.print(xpos);
      Serial.print(',');
      Serial.print(ypos);
      Serial.print(',');
      Serial.println(laserpower);
       //Serial.println((xsteps-abs(stepperx.distanceToGo())));
       stepperx.setCurrentPosition(0); // stops the stepper turning in the wrong direction when a new move is called    
       steppery.setCurrentPosition(0); // stops the stepper turning in the wrong direction when a new move is called     
    }
    }
      
    
  


