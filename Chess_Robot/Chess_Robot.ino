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
/*
   35 Straight Down
   30 Pick up
   55 Drop
*/

double xInitial = 10.0;
double yInitial = 10.0;
double xFinal = 10.0;
double yFinal = 10.0;
double L1 = 278; // L1 = 278mm
double L2 = 165.5; // L2 = 165.5mm
double theta1, theta2, phi, z;
double zDefault = 10;
double zAbove = 13;

double spacing = 39.52875;
double yOffset = 128.57938;
double xOffset = 0.0;

double gripperPositions[] = { 10, 10, 10, 10, 10, 10 };
double zGripHeight[] = { 10, 10, 10, 10, 10, 10 };
double zGripAbove[] = { 10, 10, 10, 10, 10, 10 };

double gripperOpen = 50;
double gripperClosed = 20;

int stepper1Position, stepper2Position, stepper3Position, stepper4Position;

const float theta1AngleToSteps = 44.444444;
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10;
const float zDistanceToSteps = 100;

byte inputValue[5];
int k = 0;

String content = "";
int data[10];

int theta1Array[100];
int theta2Array[100];
int phiArray[100];
int zArray[100];
int gripperArray[100];
int positionsCounter = 0;

// black 2B, green 2A, red 1A, blue 1B
// Order: Black - Green - Red - Blue


char board[8][8] = { {'r', 'n', 'b', 'q', 'k', 'b', 'n', 'r'},
  {'p', 'p', 'p', 'p', 'p', 'p', 'p', 'p'},
  {'0', '0', '0', '0', '0', '0', '0', '0'},
  {'0', '0', '0', '0', '0', '0', '0', '0'},
  {'0', '0', '0', '0', '0', '0', '0', '0'},
  {'0', '0', '0', '0', '0', '0', '0', '0'},
  {'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'},
  {'R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R'}
};

String piece[] = {"",""};

void setup() {
  Serial.begin(115200);
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

  gripperServo.attach(A0, 600, 2500);
  // initial servo value - open gripper
  data[6] = 180;
  gripperServo.write(gripperOpen);
  delay(1000);
  data[5] = 100;
  //  homing();
}

void loop() {

  if (Serial.available()) {
    content = Serial.readString(); // Read and store incoming data from the Compuuuuter
    // Extract the data from the string and parse
    for (int i = 0; i < 8; i++) {
      int index = content.indexOf(","); // Locate the first "," in the incoming data
      if (index < 6) {
      data[i] = atol(content.substring(0, index).c_str()); // Extract number from start to the ","
      } else {
        piece[i-6] = content.substring(0, index).c_str();
      }
      content = content.substring(index + 1); // Remove the number from the string
      Serial.print(i) + Serial.print( " = "); Serial.println(data[i]);
    }


    /*
      data[0] - RUN
      data[1] - X value initial position
      data[2] - Y value initial position
      data[3] - X value new position
      data[4] - Y value new position
    */
  }
  if (data[0] == 1) {
    runSequence();
    Serial.println("Running sequence");
  } else if (data[0] == 2) {
    storeData();
    Serial.println("Storing data");
  } else if (data[0] == 3) {
    zeroOneAxis();
    Serial.println("Zeroing one axis");
  } else if (data[0] == 4) {
    homing();
  }
  data[0] = 0;

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
  //Serial.println(digitalRead(limitSwitch1));
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

  // Calculate "phi" angle so gripper is parallel to the X axis
  phi = 90 + theta1 + theta2;
  phi = (-1) * phi;
  // Angle adjustment depending in which quadrant the final tool coordinate x,y is
  if (x < 0 & y < 0) {       // 3d quadrant
    phi = 270 - theta1 - theta2;
  }
  if (abs(phi) > 165) {
    phi = 180 + phi;
  }
  theta1 = round(theta1);
  theta2 = round(theta2);
  phi = round(phi);

  //  cp5.getController("j1Slider").setValue(theta1);
  //  cp5.getController("j2Slider").setValue(theta2);
  //  cp5.getController("j3Slider").setValue(phi);
  //  cp5.getController("zSlider").setValue(zP);
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
      stepper3.setCurrentPosition(-1662);
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
    stepper2.setCurrentPosition(-5420);
  }
  delay(20);
  stepper2.moveTo(0);
  while (stepper2.currentPosition() != 0) {
    stepper2.run();
  }
  // STEPPER 1
  while (digitalRead(limitSwitch1) != 1) {
    stepper1.setSpeed(-1200);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(-3955);
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
  int axis1Travel = theta1 - stepper1.currentPosition() / theta1AngleToSteps;
  int axis2Travel = theta2 - stepper2.currentPosition() / theta2AngleToSteps;
  int axis3Travel = phi - stepper3.currentPosition() / phiAngleToSteps;
  int axis4Travel = zDefault - stepper4.currentPosition() / zDistanceToSteps;

  int max1 = max(abs(axis1Travel), abs(axis2Travel));
  int max2 = max(abs(axis3Travel), abs(axis4Travel));

  int axis1Mult = axis1Travel / max(max1, max2);
  int axis2Mult = axis2Travel / max(max1, max2);
  int axis3Mult = axis3Travel / max(max1, max2);
  int axis4Mult = axis4Travel / max(max1, max2);

  stepper1Position = theta1 * theta1AngleToSteps;
  stepper2Position = theta2 * theta2AngleToSteps;
  stepper3Position = phi * phiAngleToSteps;
  stepper4Position = zDefault * zDistanceToSteps;

  stepper1.setSpeed(4000 * axis1Mult);
  stepper2.setSpeed(4000 * axis2Mult);
  stepper3.setSpeed(4000 * axis3Mult);
  stepper4.setSpeed(4000 * axis4Mult);

  stepper1.setAcceleration(2000);
  stepper2.setAcceleration(2000);
  stepper3.setAcceleration(2000);
  stepper4.setAcceleration(2000);

  stepper1.moveTo(stepper1Position);
  stepper2.moveTo(stepper2Position);
  stepper3.moveTo(stepper3Position);
  stepper4.moveTo(stepper4Position);

  Serial.print("Base moving at "); Serial.print(axis1Mult); Serial.print(" speed and moving to "); Serial.print(theta1); Serial.println(" angle");
  Serial.print("Arm moving at "); Serial.print(axis2Mult); Serial.print(" speed and moving to "); Serial.print(theta2); Serial.println(" angle");
  Serial.print("Phi moving at "); Serial.print(axis3Mult); Serial.print(" speed and moving to "); Serial.print(phi); Serial.println(" angle");
  Serial.print("Z-axis moving at "); Serial.print(axis4Mult); Serial.print(" speed and moving to "); Serial.print(zDefault); Serial.println(" angle");

  while (stepper1.currentPosition() != stepper1Position || stepper2.currentPosition() != stepper2Position || stepper3.currentPosition() != stepper3Position || stepper4.currentPosition() != stepper4Position) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }
  delay(100);
}

// GRIP OR DROP
void grip(char piece, boolean drop) {
  Serial.println("Gripping");
  switch (piece) {
    case 'q': case 'Q':
      z = 8;
      gripperClosed = 40;
      break;
    case 'r': case 'R':
      z = 8;
      gripperClosed = 40;
      break;
    case 'b': case 'B':
      z = 8;
      gripperClosed = 40;
      break;
    case 'n': case 'N':
      z = 8;
      gripperClosed = 40;
      break;
    case 'k': case 'K':
      z = 8;
      gripperClosed = 40;
      break;
    case 'p': case 'P':
      z = 8;
      gripperClosed = 40;
      break;
    default:
      Serial.println("Impropper piece");
      z = 10;
      gripperClosed = 40;
      break;
  }

  gripperClosed = (drop) ? gripperClosed : 55;

  stepper4.setSpeed(3000);
  Serial.print("Grip - Move z to "); Serial.println(z);
  stepper4.moveTo(z * zDistanceToSteps);
  while (stepper4.currentPosition() != z * zDistanceToSteps) {
    stepper4.run();
  }
  gripperServo.write(gripperClosed);
  delay(300);
  Serial.print("Grip - Move z to "); Serial.println(zAbove);
  stepper4.moveTo(zAbove * zDistanceToSteps);
  while (stepper4.currentPosition() != zAbove * zDistanceToSteps) {
    stepper4.run();
  }
}

// COMPUTE MOVE AND RUN FULL SEQUENCE
/*
  data[0] - RUN
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
*/
void runSequence() {
  char initialPiece = board[data[2]][data[1]];
  char finalPiece = board[data[4]][data[3]];
  xInitial = convertX(data[1]);
  yInitial = convertY(data[2]);
  xFinal = convertX(data[3]);
  yFinal = convertY(data[4]);
  Serial.print("Initial piece: "); Serial.print(initialPiece); Serial.print(" Final piece "); Serial.println(finalPiece);
  if (finalPiece == '0') {
    moveArm(xInitial, yInitial);
    grip(initialPiece, true);
    moveArm(xFinal, yFinal);
    grip(initialPiece, false);
    board[data[2]][data[1]] = '0';
    board[data[4]][data[3]] = initialPiece;
  } else if (data[5] == 1) {
    // Queen Side Castling
    moveArm(convertX(4), convertY(0));
    grip('Q', true);
    moveArm(convertX(2), convertY(0));
    grip('Q', false);
    moveArm(convertX(0), convertY(0));
    grip('R', true);
    moveArm(convertX(3), convertY(0));
    grip('R', false);
    board[0][4] = '0';
    board[0][0] = '0';
    board[0][2] = 'Q';
    board[0][3] = 'R';
  } else if (data[5] == 2) {
    // King Side Castling
    moveArm(convertX(4), convertY(0));
    grip('Q', true);
    moveArm(convertX(6), convertY(0));
    grip('Q', false);
    moveArm(convertX(7), convertY(0));
    grip('R', true);
    moveArm(convertX(5), convertY(0));
    grip('R', false);
    board[0][4] = '0';
    board[0][7] = '0';
    board[0][6] = 'Q';
    board[0][5] = 'R';
  } else if (data[5] == 0) {
    // Capture piece first, then move our intended piece
    moveArm(xFinal, yFinal);
    grip(finalPiece, true);
    // FIGURE OUT END ZONE *************************************************
    moveArm(xInitial, yInitial);
    grip(initialPiece, true);
    moveArm(xFinal, yFinal);
    grip(initialPiece, false);
    board[data[2]][data[1]] = '0';
    board[data[4]][data[3]] = initialPiece;
  }

  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      Serial.print(board[i][j]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
}

double convertX(double x) {
  return (x - 3.5) * spacing + xOffset;
}

double convertY(double y) {
  return y * spacing + yOffset;
}

void storeData() {
  char piece[2][7] = {{'0', 'k', 'q', 'n', 'b', 'p', 'r'}, {'0', 'K', 'Q', 'N', 'B', 'P', 'R'}};
  /*
      data[0] - STORE DATA
      data[1] - X value position
      data[2] - Y value position
      data[3] - Piece
                0 - Nothing
                1 - King
                2 - Queen
                3 - Knight
                4 - Bishop
                5 - Pawn
                6 - Rook
      data[4] - 0 lowercase, 1 uppercase
  */

  board[data[2]][data[1]] = piece[data[4]][data[3]];

}

void zeroOneAxis() {
  // STEPPER 1
  Serial.println("Zeroing Axis #1");
  while (digitalRead(limitSwitch1) != 1) {
    stepper1.setSpeed(-1200);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(-3955);
  }
  Serial.println("Hit limit switch");
  delay(20);
  stepper1.moveTo(0);
  while (stepper1.currentPosition() != 0) {
    stepper1.run();
  }
  //  Serial.println("Done");

  //   STEPPER 2
  //  Serial.println("Zeroing Axis #2");
  //  while (digitalRead(limitSwitch2) != 1) {
  //    stepper2.setSpeed(-1300);
  //    stepper2.runSpeed();
  //    stepper2.setCurrentPosition(-5420);
  //  }
  //  Serial.println("Hit limit switch");
  //  delay(20);
  //  stepper2.moveTo(0);
  //  while (stepper2.currentPosition() != 0) {
  //    stepper2.run();
  //  }

  // STEPPER 3
  //  while (digitalRead(limitSwitch3) != 1) {
  //    stepper3.setSpeed(-1100);
  //    stepper3.runSpeed();
  //    stepper3.setCurrentPosition(-1662);
  //  }
  //  delay(20);
  //  stepper3.moveTo(0);
  //  while (stepper3.currentPosition() != 0) {
  //    stepper3.run();
  //  }

  // STEPPER 4
  //  while (digitalRead(limitSwitch4) != 1) {
  //    stepper4.setSpeed(3000);
  //    stepper4.runSpeed();
  //    stepper4.setCurrentPosition(17000);
  //  }
  //  delay(20);
  //  stepper4.moveTo(10000);
  //  while (stepper4.currentPosition() != 10000) {
  //    stepper4.run();
  //  }
  Serial.println("Done");
}
