/*

Chess playing robot code! This is the "dumb robot" part as the arduino doesn't know anything, just follows directions

Interface using the Serial port at 115200 refresh rate. Input values separated by a ,

  data[0] - RUN (0 is homing all axis, 1 is to run sequences)
  data[1] - X value initial position
  data[2] - Y value initial position
  data[3] - X value new position
  data[4] - Y value new position
  data[5] - Special Move
            0 - no special move
            1 - O-O
            2 - O-O-O
            3 - Weird pawn thing
            4 - Promote to queen
            5 - Promote to rook
            6 - Promote to knight
            7 - Promote to bishop
            8 - Test placement using cool maths
            9 - Test placement using coordinates

1,2,3,5,6,0,p,. -> Move arm from (2,3) to (5,6) with no special move, where pawn moves to empty spot

*/

#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define limitSwitch1 11
#define limitSwitch2 10
#define limitSwitch3 9
#define limitSwitch4 A3

// Define the stepper motors and the pins they will use
// (Type:driver, STEP, DIR)
AccelStepper stepper1(1, 2, 5);   // BASE
AccelStepper stepper2(1, 3, 6);   // ARM
AccelStepper stepper3(1, 4, 7);   // PHI
AccelStepper stepper4(1, 12, 13); // Z-AXIS

Servo gripperServo;

double xInitial = 10.0;
double yInitial = 10.0;
double xFinal = 10.0;
double yFinal = 10.0;
double L1 = 278; // L1 = 278mm
double L2 = 166.5; // L2 = 165.5mm
double theta1, theta2, phi, z;
double zAbove = 10; // 120 sorta works

double spacing = 38.5;
double yOffset = 123.57938;
double xOffset = 0.3;

// ORDER: Q, R, B, N, K, P
double gripperPositions[] = { 30, 30, 30, 30, 30, 30 };
double zGripHeight[] = { 8, 8, 8, 8, 8, 8 };
double zGripAbove[] = { 10, 10, 10, 10, 10, 10 };

int boardLocations[][8][2] = {{{-30,-3},  {-34,-5},   {-35,-5},   {-35,0},    {-37,0},    {-40,8},    {-40,10}, {-40,18}}, 
                              {{-32,-11}, {-32,-10},  {-35,-5},   {-40,-5},   {-43,-3},   {-43,5},    {-45,5},  {-47,14}},
                              {{-30,-10}, {-30,-15},  {-35,-10},  {-35,-10},  {-38,-5},   {-40,0},    {-45,5},  {-45,15}},
                              {{-20,-16}, {-25,-15},  {-30,-15},  {-35,-10},  {-35,-5},   {-40,-5},   {-40,8},  {-45,10}},
                              {{-17,-21}, {-18,-20},  {-28,-18},  {-25,-15},  {-30,-10},  {-35,-10},  {-35,0},  {-40,10}},
                              {{-11,-21}, {-13,-20},  {-15,-20},  {-15,-10},  {-25,-15},  {-30,-10},  {-35,-5}, {-40,5}},
                              {{0,-18},   {-2,-20},   {-5,-20},   {-5,-15},   {-13,-15},  {-20,-10},  {-30,-5}, {-35,5}},
                              {{0,-13},   {0,-10},    {5,-10},    {2,-8},     {-2,-10},   {-10,-10},  {-25,-5}, {-30,5}}};

char endZone[6][3] = {{'.', '.', '.'}, {'.', '.', '.'}, {'.', '.', '.'}, {'.', '.', '.'}, {'.', '.', '.'}, {'.', '.', '.'}};
bool capturedPiece = false;

double gripperOpen = 45;
double gripPos = 30;

int stepper1Position, stepper2Position, stepper3Position, stepper4Position;

const float theta1AngleToSteps = 44.444444;
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10;
const float zDistanceToSteps = 100;

double lastX = -50;
double lastY = 250;
bool firstLoop = true;
bool updateValues = false;

byte inputValue[5];
int k = 0;

String content = "";
int data[10];

// black 2B, green 2A, red 1A, blue 1B
// Order: Black - Green - Red - Blue

char piece[2] = {' ', ' '};

void setup() {
  Serial.begin(9600);
  Serial.println("Setting up");
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);

  // Stepper motors to max speed
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);
  stepper4.setMaxSpeed(4000);
  stepper4.setAcceleration(2000);

  // gripperServo.attach(A0, 600, 2500);
  gripperServo.attach(A0);
  // initial servo value - open gripper
  gripperServo.write(gripperOpen);
  delay(1000);
}

void loop() {
  data[0] = -1;
  if (Serial.available() > 0) {
    Serial.print("Reading data...");
    content = Serial.readString(); // Read and store incoming data from the Compuuuuter
    Serial.println("the data is :" + content);
    if (content.length() > 7) {
    // Extract the data from the string and parse
    for (int i = 0; i < 8; i++) {
      int index = content.indexOf(","); // Locate the first "," in the incoming data
      if (i < 6) {
      data[i] = atol(content.substring(0, index).c_str()); // Extract number from start to the ","
      } else {
        piece[i-6] = content.substring(0, index).charAt(0);
      }
      content = content.substring(index + 1); // Remove the number from the string
      Serial.print(i) + Serial.print( " = "); Serial.println(data[i]);
    }
  } else if (content.length() > 2) {
    Serial.println("Updating position...");
    // Extract the data from the string and parse
    for (int i = 1; i < 3; i++) {
      int index = content.indexOf(","); // Locate the first "," in the incoming data
      data[i] = atol(content.substring(0, index).c_str()); // Extract number from start to the ","
      content = content.substring(index + 1); // Remove the number from the string
      Serial.print(i) + Serial.print( " = "); Serial.println(data[i]); 
    }
    data[0] = 1;
    updateValues = true;
    lastX = lastX + data[1];
    lastY = lastY + data[2];
    }
  }
  
  if (data[0] == 0) {
    homing();
  } else if (data[0] == 1) {
    Serial.println("Running sequence");
    runSequence();
  } 

  data[0] = -1;

  if (digitalRead(limitSwitch1) == 1) {
    Serial.println("1");
  }
  if (digitalRead(limitSwitch2) == 1) {
    Serial.println("2");
  }
  if (digitalRead(limitSwitch3) == 1) {
    Serial.println("3");
  }
  if (digitalRead(limitSwitch4) == 1) {
    Serial.println("4");
  }

  delay(500);
}

// FORWARD KINEMATICS
void forwardKinematics() {
  float theta1F = theta1 * PI / 180;
  float theta2F = theta2 * PI / 180;
  double xP = round(L1 * cos(theta1F) + L2 * cos(theta1F + theta2F));
  double yP = round(L1 * sin(theta1F) + L2 * sin(theta1F + theta2F));
}

// INVERSE KINEMATICS
void inverseKinematics(float x, float y) {
  x = -x; // Experiment

  theta2 = acos((sq(x) + sq(y) - sq(L1) - sq(L2)) / (2 * L1 * L2));
  if (x < 0 & y < 0) {
    theta2 = (-1) * theta2;
  }

  theta1 = atan(x / y) - atan((L2 * sin(theta2)) / (L1 + L2 * cos(theta2)));

  theta2 = (-1) * theta2 * 180 / PI;
  theta1 = theta1 * 180 / PI;
  // Angles adjustment depending in which quadrant the final tool coordinate x,y is
  if (x >= 0 & y >= 0) {       // 1st quadrant
    theta1 = 90 - theta1;
  }
  if (x < 0 & y > 0) {       // 2nd quadrant
    theta1 = 90 - theta1;
  }
  if (x < 0 & y < 0) {       // 3d quadrant
    theta1 = 270 - theta1;
    phi = 270 - theta1 - theta2;
    phi = (-1) * phi;
  }
  if (x > 0 & y < 0) {       // 4th quadrant
    theta1 = -90 - theta1;
  }
  if (x < 0 & y == 0) {
    theta1 = 270 + theta1;
  }

  // Eperimental code
  theta1 = 180 - theta1; 
  theta2 = -theta2; 

  // Calculate "phi" angle so gripper is parallel to the X axis
  phi = 90 + theta1 + theta2;
  phi = (-1) * phi;
  phi = 180 + phi;
  Serial.println("Phi is at the moment: " + String(phi));
  // Angle adjustment depending in which quadrant the final tool coordinate x,y is
  if (x < 0 & y < 0) {       // 3d quadrant
    phi = 270 - theta1 - theta2;
  }
  if (abs(phi) > 165 || phi < -145) {
    phi = 180 + phi;
  }
  theta1 = round(theta1);
  theta2 = round(theta2);
  phi = round(phi);

}

// ZERO ALL AXIS
void homing() {
  Serial.println("HOMEING");
  // STEPPER 4
  while (digitalRead(limitSwitch4) != 1) {
    stepper4.setSpeed(1500);
    stepper4.runSpeed();
    stepper4.setCurrentPosition(17000);
  }
  delay(20);
  stepper4.moveTo(10000);
  while (stepper4.currentPosition() != 10000) {
    stepper4.run();
  }
    // STEPPER 3
    while (digitalRead(limitSwitch3) != 1) {
      stepper3.setSpeed(-1100);
      stepper3.runSpeed();
      stepper3.setCurrentPosition(-1465);
    }
    delay(20);
    stepper3.moveTo(0);
    while (stepper3.currentPosition() != 0) {
      stepper3.run();
    }
  // STEPPER 2
  while (digitalRead(limitSwitch2) != 1) {
    stepper2.setSpeed(-1300);
    stepper2.runSpeed();
    stepper2.setCurrentPosition(-5670);
  }
  delay(20);
  stepper2.moveTo(3000);
  while (stepper2.currentPosition() != 3000) {
    stepper2.run();
  }
  while (digitalRead(limitSwitch1) != 1) {
    stepper1.setSpeed(-1200);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(-3423); 
  }
  delay(20);
  stepper1.moveTo(0);
  while (stepper1.currentPosition() != 0) {
    stepper1.run();
  }
}

// MOVE ARM TO LOCATION
void moveArm(float x, float y) {
  inverseKinematics(x, y);
  double axis1Travel = theta1 - stepper1.currentPosition() / theta1AngleToSteps;
  double axis2Travel = theta2 - stepper2.currentPosition() / theta2AngleToSteps;
  double axis3Travel = phi - stepper3.currentPosition() / phiAngleToSteps;

  double max1 = max(abs(axis1Travel), abs(axis2Travel));
  double max2 = max(abs(axis3Travel), max1);

  double axis1Mult = (double)axis1Travel / max2;
  double axis2Mult = (double)axis2Travel / max2;
  double axis3Mult = (double)axis3Travel / max2;

  stepper1Position = theta1 * theta1AngleToSteps;
  stepper2Position = theta2 * theta2AngleToSteps;
  stepper3Position = phi * phiAngleToSteps;

  stepper1.setSpeed(4000 * axis1Mult);
  stepper2.setSpeed(4000 * axis2Mult);
  stepper3.setSpeed(4000 * axis3Mult);

  stepper1.setAcceleration(2000);
  stepper2.setAcceleration(2000);
  stepper3.setAcceleration(2000);

  stepper1.moveTo(stepper1Position);
  stepper2.moveTo(stepper2Position);
  stepper3.moveTo(stepper3Position);

  Serial.print("Base moving at "); Serial.print(axis1Mult); Serial.print(" speed and moving to "); Serial.print(theta1); Serial.println(" angle");
  Serial.print("Arm moving at "); Serial.print(axis2Mult); Serial.print(" speed and moving to "); Serial.print(theta2); Serial.println(" angle");
  Serial.print("Phi moving at "); Serial.print(axis3Mult); Serial.print(" speed and moving to "); Serial.print(phi); Serial.println(" angle");

  while (stepper1.currentPosition() != stepper1Position || stepper2.currentPosition() != stepper2Position || stepper3.currentPosition() != stepper3Position) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  Serial.println("Base at : " + String(stepper1.currentPosition()) + ", Arm at " + String(stepper2.currentPosition()) + ", phi at " + String(stepper3.currentPosition()) + ", z-axis at " + String(stepper4.currentPosition()));
  Serial.println("Trying to move to X:" + String(x) + " Y:" + String(y));
  delay(100);
}

// GRIP OR DROP
void grip(char piece, boolean drop) {
  Serial.println("Gripping piece " + String(piece));
  switch (piece) {
    case 'q': case 'Q':
      z = zGripHeight[0];
      gripPos = gripperPositions[0];
      zAbove = zGripAbove[0];
      break;
    case 'r': case 'R':
      z = zGripHeight[1];
      gripPos = gripperPositions[1];
      zAbove = zGripAbove[1];
      break;
    case 'b': case 'B':
      z = zGripHeight[2];
      gripPos = gripperPositions[2];
      zAbove = zGripAbove[2];
      break;
    case 'n': case 'N':
      z = zGripHeight[3];
      gripPos = gripperPositions[3];
      zAbove = zGripAbove[3];
      break;
    case 'k': case 'K':
      z = zGripHeight[4];
      gripPos = gripperPositions[4];
      zAbove = zGripAbove[4];
      break;
    case 'p': case 'P':
      z = zGripHeight[5];
      gripPos = gripperPositions[5];
      zAbove = zGripAbove[5];
      break;
    case 't':
      break;
    default:
      Serial.println("Impropper piece");
      z = 10;
      gripPos = 20;
      zAbove = 120;
      break;
  }

  gripPos = (drop) ? gripPos : gripperOpen;

  if (capturedPiece) {
    z = z - 20;
  }

  stepper4.setSpeed(3000);
  Serial.print("Grip - Move z to "); Serial.println(z);
  stepper4.moveTo(z * zDistanceToSteps);
  while (stepper4.currentPosition() != z * zDistanceToSteps) {
    stepper4.run();
  }
  gripperServo.write(gripPos);
  delay(3000);
  Serial.print("Grip - Move z to "); Serial.println(zAbove);
  stepper4.moveTo(zAbove * zDistanceToSteps);
  while (stepper4.currentPosition() != zAbove * zDistanceToSteps) {
    stepper4.run();
  }
}

// COMPUTE MOVE AND RUN FULL SEQUENCE
void runSequence() {
  char initialPiece = piece[0];
  char finalPiece = piece[1];

  if (updateValues) {
    xInitial = lastX;
    yInitial = lastY;
    updateValues = false;
    Serial.println("Updating values with new numbers and stuff");
  } else {
    firstLoop = true;
    xInitial = convert(data[1], data[2], true);
    yInitial = convert(data[1], data[2], false);
    firstLoop = false;
    xFinal = convert(data[3], data[4], true);
    yFinal = convert(data[3], data[4], false);
  }

  Serial.print("Initial piece: "); Serial.print(initialPiece); Serial.print(" Final piece "); Serial.println(finalPiece);
  Serial.println("Initial coordinates:(" + String(xInitial) + "," + String(yFinal) + " and (" + String(xFinal) + "," + String(yFinal));
  
  if (data[5] == 0 && finalPiece != '.') {  // No special event, capture piece then move piece
    Serial.println("Capturing piece first, then moving");
    moveArm(xFinal, yFinal);
    grip(finalPiece, true);
    // FIGURE OUT END ZONE *************************************************
    moveEndZone(finalPiece); 
    moveArm(xInitial, yInitial);
    grip(initialPiece, true);
    moveArm(xFinal, yFinal);
    grip(initialPiece, false);
  } else if (finalPiece == '.') {           // No special event, no capture
    Serial.println("Just moving piece");
    moveArm(xInitial, yInitial);
    grip(initialPiece, true);
    moveArm(xFinal, yFinal);
    grip(initialPiece, false);
  } else if (data[5] == 1) {                // Queen side castling
    Serial.println("Queen side castling");
    moveArm(convertX(4), convertY(0));
    grip('Q', true);
    moveArm(convertX(2), convertY(0));
    grip('Q', false);
    moveArm(convertX(0), convertY(0));
    grip('R', true);
    moveArm(convertX(3), convertY(0));
    grip('R', false);
  } else if (data[5] == 2) {                // King side castling
    Serial.println("King side castling");
    moveArm(convertX(4), convertY(0));
    grip('Q', true);
    moveArm(convertX(6), convertY(0));
    grip('Q', false);
    moveArm(convertX(7), convertY(0));
    grip('R', true);
    moveArm(convertX(5), convertY(0));
    grip('R', false);
  } else if (data[5] == 8) {                // Custom event, no move piece, just go to location and pick if data[3] + data[4] > -1
    Serial.println("Moving pieces to chess coordinates");
    moveArm(xInitial, yInitial);
    if ((data[3] + data[4]) > -1) {
      z = data[3];
      gripPos = data[4];
      zAbove = data[3];
      grip(initialPiece,true);
      delay(2000);
      zAbove = 10;
      grip(initialPiece,false);
    }
  } else if (data[5] == 9) {              // Custom event, no move piece, just move to coordinates and pick if data[3] + data[4] > -1
    Serial.println("Moving to your coordinates");
    moveArm(data[1], data[2]);
    if ((data[3] + data[4]) > -1) {
      z = data[3];
      gripPos = data[4];
      zAbove = data[3];
      grip(initialPiece,true);
      delay(2000);
      zAbove = 10;
      grip(initialPiece,false);
    }
  } 
}

double convertX(double x) {
  return (x - 3.5) * spacing + xOffset;
}

double convertY(double y) {
  return y * spacing + yOffset;
}

double convert(double x, double y, bool isX) {
  double converted[] = {(x - 3.5) * spacing + xOffset + boardLocations[abs(7-int(y))][int(x)][0], y * spacing + yOffset + boardLocations[abs(7-int(y))][int(x)][1]};
  Serial.println("Converted[0] is : " + String(converted[0]) + " converted[1] is: " + String(converted[1])); 
  if (firstLoop && isX) { lastX = converted[0]; }
  else if (firstLoop && !isX) { lastY = converted[1];}
  return (isX) ? converted[0] : converted[1];
}

void moveEndZone(char piece) {
  float placementX = 10;
  float placementY = 4.1;
  for (int i = 0; i < sizeof(endZone); i++) {
    for (int j = 0; j < sizeof(endZone[0]); j++) {
      if (endZone[i][j] == '.') {
        placementX = i + 10;
        placementY = j;
        endZone[i][j] = piece;
      }
    }
    if (placementX != 10 || placementY != 4.1) {
      break;
    }
  }

  moveArm(placementX, placementY); 
  capturedPiece = true;
  grip(piece, false);
  capturedPiece = false;
}