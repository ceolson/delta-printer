#include <AccelStepper.h>
#include <MultiStepper.h>

// Define pin numbers, other global stuff
const int stepPin1 = 9; 
const int dirPin1 = 8; 
const int stepPin2 = 7; 
const int dirPin2 = 6; 
const int stepPin3 = 5; 
const int dirPin3 = 4; 
const int stepPin4 = 3;
const int dirPin4 = 2;
const int thermistorPin = A0;
const int heaterPin = 10;
const int max_store = 128;
byte heater_low = 64;
byte heater_high = 255;
const float legLength = 150;
const float radius = 80;
const float topSpeed = 500;
const float printTemp = 210;

// Set up some global variables to track position

float carriage1position = sqrt(sq(legLength)+sq(radius));
float carriage2position = sqrt(sq(legLength)+sq(radius));
float carriage3position = sqrt(sq(legLength)+sq(radius));
float axis1length = radius;
float axis2length = radius;
float axis3length = radius;
float xposition = 0;
float yposition = 0;
float zposition = 0;
int stepper1steps = 0;
int stepper2steps = 0;
int stepper3steps = 0;

// Set up steppers in AccelStepper

//AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
//AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);
//AccelStepper stepper3(AccelStepper::DRIVER, stepPin3, dirPin3);
//AccelStepper stepper4(AccelStepper::DRIVER, stepPin4, dirPin4);

AccelStepper stepper1(1, stepPin1, dirPin1);
AccelStepper stepper2(1, stepPin2, dirPin2);
AccelStepper stepper3(1, stepPin3, dirPin3);
AccelStepper stepper4(1, stepPin4, dirPin4);



MultiStepper steppers;
  
void setup() {

// AccelStepper stuff

  stepper1.setMaxSpeed(100);
  stepper1.setAcceleration(500);
  stepper1.setMinPulseWidth(50);
  stepper2.setMaxSpeed(100);
  stepper2.setAcceleration(500);
  stepper2.setMinPulseWidth(50);
  stepper3.setMaxSpeed(100);
  stepper3.setAcceleration(500);
  stepper3.setMinPulseWidth(50);
  stepper4.setMaxSpeed(100);
  stepper4.setAcceleration(500);
  stepper4.setMinPulseWidth(50);
  
// Open serial port
  
  Serial.begin(9600);

// Give steppers to MultiStepper

  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);  
}

// Global variables that are used in G-Code interpreter

float xgoal;
float ygoal;
float zgoal;
float feedrate;
float extrusion;

void loop() {
  //linearTo(0,10,0,2,10);
  //linearTo(10,10,0,2,10);
  //linearTo(10,0,0,2,10);
  //linearTo(0,0,0,2,10);
  //delay(1000);
  
  
  // Gather G-Code lines from serial port and send them to processLine
  
  int current_store = 0;
  char storage[max_store];
  for(int i=0;i<max_store;i++){
    storage[i] = 't';
  }
  int no_data = 0;
  char c;
  
  if(Serial.available()){
    c = Serial.read();
    storage[current_store] = c;
    current_store++;
  }
  
  if (c == '\n'){
    Serial.write("hi");
    Serial.write(storage);
    if (strchr(storage, "//") == NULL){
      processLine(storage, current_store);
      Serial.write(storage, current_store);
    }
    for(int i = 0; i< current_store; i++){
      storage[current_store] = 0;
    }
    current_store = 0;
  
  }

}

// Returns the number after a specific key character (ex. 'G') in the instruction

double search_string(char key, char instruction[], int string_length){
  char temp[10] = "";
  for (int i=0; i<string_length; i++){
    if(instruction[i] == key){
      i++;
      int k=0;
      while (i < string_length && k<10){
        if(instruction[i] == ' '){
          break;
        }
        temp[k] = instruction[i];
        i++;
        k++;
        
        
      }
      return strtod(temp, NULL);
      
    }
  }
  return 0;
}

// Searches a instruction character array for a key (ex. 'G')

bool has_command(char key, char instruction[], int string_length){
  for (int i=0; i<string_length; i++){
    if (instruction[i] == key){
      return true;
    }
  }
  return false;
}


void processLine(char instruction[], int instruction_length){

  if (has_command('G', instruction, instruction_length) || 
  has_command('X', instruction, instruction_length) ||
  has_command('Y', instruction, instruction_length) || 
  has_command('Z', instruction, instruction_length)){
    int gtype = (int)search_string('G', instruction, instruction_length);
    switch(gtype){
      case 0:
      case 1:
      case 2:
      case 3:
        if (has_command('X', instruction, instruction_length))
          xgoal = search_string('X', instruction, instruction_length);
        else
          xgoal = xposition;
        
        if (has_command('Y', instruction, instruction_length))
          ygoal = search_string('Y', instruction, instruction_length);
        else
          ygoal = yposition;
        
        if (has_command('Z', instruction, instruction_length))
          zgoal = search_string('Z', instruction, instruction_length);
        else
          zgoal = zposition;
        break;
    }
    switch(gtype){
      case 0:
      case 1:
        if(gtype == 1){
          if(has_command('F', instruction, instruction_length)){
            feedrate = search_string('F', instruction, instruction_length);
          }
          if(has_command('E', instruction, instruction_length)){
            extrusion = search_string('E', instruction, instruction_length);
          }
            
          if (feedrate<=0){
            feedrate = topSpeed;
          }
          else{
            feedrate = topSpeed;
            extrusion = 0;
          }
        }
        else{
          feedrate = topSpeed;
        }
        linearTo(xgoal, ygoal, zgoal, feedrate, extrusion);
        break;
      case 2:
      case 3:
      
      case 4:
        delay((int)search_string('P', instruction, instruction_length));
        break;
    }
  }
/*  if has_command('M', instruction, instruction_length){
    int mtype = (int)search_string('M', instruction, instruction_length);
    switch(mtype){
      case 
    }
  }*/
}

// Absolute linear movement, breaks up movement into small bits to mix movements of the carriages

void linearTo(float x, float y, float z, float feedrate, float extrusion){
  float distanceOfMove = sqrt(sq(x - xposition) + sq(y - yposition) + sq(z - zposition));
  int numSteps = round(distanceOfMove)/10;
  float timePerStep = (distanceOfMove / feedrate) / numSteps;
  float xTotal = x - xposition;
  float yTotal = y - yposition;
  float zTotal = z - zposition;
  float xStep = xTotal / numSteps;
  float yStep = yTotal / numSteps;
  float zStep = zTotal / numSteps;
  float extrudePerStep = extrusion / numSteps;
  float extrudeStepperSteps = extrudePerStep * 160;

  
  float incrementx = xposition + xStep;
  float incrementy = yposition + yStep;
  float incrementz = zposition + zStep;
  
  for(int i=0; i<numSteps; i++){
    goToCoordinates(incrementx, incrementy, incrementz, feedrate);
    stepper4.move(extrudeStepperSteps);
    stepper4.runSpeedToPosition();
    incrementx += xStep;
    incrementy += yStep;
    incrementz += zStep;
  }
}

// Absolute arc movement

void arcTo(float endx, float endy, float centerx, float centery, bool CW, float feedrate, float extrusion){
  float radius = sqrt(sq(endx - centerx) + sq(endy - centery));
  float startAngle = atan2((yposition - centery),(xposition - centerx)) + M_PI * (yposition - centery < 0 ? 1 : 0);
  float endAngle = atan2((endy - centery),(endx - centerx)) + M_PI * (endy - centery < 0 ? 1 : 0);
  float angle;
  if (CW) {angle = startAngle - endAngle;}
  if (!CW) {angle = endAngle - startAngle;}
  float distanceOfMove = 2 * M_PI * radius * abs(angle) / (2 * M_PI);
  int numSteps = round(distanceOfMove * 2);
  float stepAngle = angle / numSteps;
  float timePerStep = (distanceOfMove / feedrate) / numSteps;
  float incrementAngle;
  if (CW) {float incrementAngle = startAngle - stepAngle;}
  if (!CW) {float incrementAngle = startAngle + stepAngle;}
  float extrudePerStep = extrusion / numSteps;
  float extrudeStepperSteps = extrudePerStep * 160;
  stepper4.setSpeed(9600);

  for(int i=0; i<numSteps; i++){
    goToCoordinates(centerx + cos(incrementAngle), centery + sin(incrementAngle), zposition, feedrate);
    Serial.println(incrementAngle);
    stepper4.move(extrudeStepperSteps);
    stepper4.runSpeedToPosition();
    if (CW) {incrementAngle -= stepAngle;}
    if (!CW) {incrementAngle += stepAngle;}
  }
}

// This function turns a coordinate goal into axis length change

void goToCoordinates(float x, float y, float z, float feedrate){    
  
  // Calculates the distance needed to change the axes to get to the x and y coordinates 
  
  float axis1to = sqrt(sq(x) + sq(y - radius));
  float axis2to = sqrt(sq(y + radius/2) + sq(x + ((radius * sqrt(3))/2)));
  float axis3to = sqrt(sq(y + radius/2) + sq(x - ((radius * sqrt(3))/2)));

  // Calculating speeds according to proportional distances for each stepper
  
  float distanceOfMove = sqrt(sq(x - xposition) + sq(y - yposition));
  float timeForMove = distanceOfMove/feedrate;
  float carriage1Distance = sqrt(sq(legLength) - sq(axis1to)) - carriage1position;
  float carriage2Distance = sqrt(sq(legLength) - sq(axis2to)) - carriage2position;
  float carriage3Distance = sqrt(sq(legLength) - sq(axis3to)) - carriage3position;

  float totalCarriageDistances = carriage1Distance + carriage2Distance + carriage3Distance;

  float totalSteps = totalCarriageDistances * 4;
  float stepSpeed = totalSteps/timeForMove;

  changeLengths(axis1to, axis2to, axis3to, stepSpeed);

  moveCarriages(z - zposition, z - zposition, z - zposition, feedrate);

  xposition = x;
  yposition = y;
  zposition = z;
}

// This function turns axis length change into carriage movement. Runs absolutely. 

void changeLengths(float axis1to, float axis2to, float axis3to, float stepSpeed){  

  float carriageDistance1 = sqrt(sq(legLength) - sq(axis1to)) - carriage1position;

  float carriageDistance2 = sqrt(sq(legLength) - sq(axis2to)) - carriage2position;

  float carriageDistance3 = sqrt(sq(legLength) - sq(axis3to)) - carriage3position;
  
  moveCarriages(carriageDistance1, carriageDistance2, carriageDistance3, stepSpeed);

  axis1length = axis1to;
  axis2length = axis2to;
  axis3length = axis3to;
}

// This function turns carriage movement into stepper movement. Runs relatively.

void moveCarriages(float carriageDistance1, float carriageDistance2, float carriageDistance3, float stepSpeed){  

  long stepsToGo[3];
  stepsToGo[0] = 4 * carriageDistance1 + stepper1.currentPosition();
  stepsToGo[1] = 4 * carriageDistance2 + stepper2.currentPosition();
  stepsToGo[2] = 4 * carriageDistance3 + stepper3.currentPosition();

  stepper1.setSpeed(stepSpeed);
  stepper2.setSpeed(stepSpeed);
  stepper3.setSpeed(stepSpeed);

  steppers.moveTo(stepsToGo);
  steppers.runSpeedToPosition();
  
  carriage1position = carriage1position + carriageDistance1;
  carriage2position = carriage2position + carriageDistance2;
  carriage3position = carriage3position + carriageDistance3;
}

// Uses averageVoltageReading to calculate temperature

float getTemp(){
  float resistance = 10000*(5/averageVoltageReading()-1);
  return 1/(1/298.15+1/3950*log(resistance/100000))-273.15;
}

// Averages voltage reading to calcuate resistance of thermistor

float averageVoltageReading(){
  float total = 0;
  for(int i=0; i<5; i++){
    float voltage = analogRead(thermistorPin)*(5.0/1023.0);
    total += voltage;  
  }
  return total/5;
}

// Regulates temperature by switching hotend on and off

void setTemp(float desiredTemp){
  float temp = getTemp();
  if (desiredTemp > temp){
    while (desiredTemp > temp){
      digitalWrite(heaterPin, heater_high);
      delay(100);
    }
    digitalWrite(heaterPin, heater_low);
  } else if (desiredTemp == temp) {
    digitalWrite(heaterPin, heater_low);
  } else{
    while (desiredTemp < temp){
      digitalWrite(heaterPin, 0);
    }
    digitalWrite(heaterPin, heater_low);
  }
}
