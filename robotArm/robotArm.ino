// https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/
// https://forum.arduino.cc/t/increase-steppe-motor-speed/486907/8#msg3455713

// speed in AccelStepper is in steps/sec
// in the beginning, only add support for updating positions with MoveJoints
// i think the way we can do it is by not interpolating in the arduino code
// we just let the arduino send a message to the raspi when it has reached it's first
// position, and then the raspi can send a new command

// or, the raspi calculates all the points in the path,
// and sends all of it to the arduino

// Largest motor in AX_6: 38*38*38

// 90\n45\n80\n90\n90\n90\n20\n1000


// Include libraries
#include <AccelStepper.h>
#include <Servo.h>

// Define pins from the Ramps 1.4 shield
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define Q_STEP_PIN         36
#define Q_DIR_PIN          34
#define Q_ENABLE_PIN       30

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define FAN_PIN            9

#define PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       10
#define HEATER_1_PIN       8
#define TEMP_0_PIN         13   // ANALOG NUMBERING
#define TEMP_1_PIN         14   // ANALOG NUMBERING

#define SERVO_1_PIN        11
#define SERVO_2_PIN        6
#define SERVO_3_PIN        5
#define SERVO_4_PIN        4

// Other constants
#define jyb28408_Pin1  59     // IN1 on the ULN2003 driver
#define jyb28408_Pin2  64     // IN2 on the ULN2003 driver
#define jyb28408_Pin3  44     // IN3 on the ULN2003 driver
#define jyb28408_Pin4  66     // IN4 on the ULN2003 driver

#define nema17_steps_per_rev  3200
#define MOTOR_PULLEY_TEETH  16.0
#define jyb28408_steps_per_rev  2058

#define nema17_motorInterfaceType 1
#define jyb28408_motorInterfaceType 8

// Notes on each joints range of motion
// Positive for all joints are away from their limit switch
// Their home position is so that the arm points in a 90 degree from ax3
// Home position is relative to the endstop, and will be set as zero
// Min position is towards the endstop with home as 0
// Max position is away from the endstop with home as 0

#define axis_1_home        90
#define axis_1_min         -90
#define axis_1_max         190

#define axis_2_home        42
#define axis_2_min         -42
#define axis_2_max         110

#define axis_3_home        95
#define axis_3_min         -95
#define axis_3_max         95

// Min/max in frequence because servo
#define axis_4_home        0 // In degrees
#define axis_4_min         550 
#define axis_4_max         2540 // Same as 180 on the servo

#define axis_5_home        102
#define axis_5_min         -102
#define axis_5_max         102

#define axis_6_home        90 
#define axis_6_min         -90 
#define axis_6_max         240

// Min/max in frequence because servo
#define gripper_home       180 // in dgrees
#define gripper_min        1300 // Open 
#define gripper_max        2300 // Close


// Ax 1, 2, 3 ... (bottom to top) Max postions in deg. Convert to rad later
int minJointPositon[7] =  {axis_1_min, axis_2_min, axis_3_min, axis_4_min, axis_5_min, axis_6_min, gripper_min};
int maxJointPosition[7] = {axis_1_max, axis_2_max, axis_3_max, axis_4_max, axis_5_max, axis_6_max, gripper_max};
int homeJointPosition[7] = {axis_1_home, axis_2_home, axis_3_home, axis_4_home, axis_5_home, axis_6_home, gripper_home};

// The pins for the endstops. The axes driven by servos do not have endstops
int endstops[7] = {X_MIN_PIN, X_MAX_PIN, Y_MIN_PIN, 0, Y_MAX_PIN, Z_MIN_PIN, 0};

// 
int jointPosition[7] = {0, 0, 0, 0, 0, 0, 0};
// HAVE TO MAKE A FIX FOR THE MOTION LINK

// The number of steps needed from the motor to move the output shaft 360 deg
float stepsPerDeg[7] = {(160.0 / MOTOR_PULLEY_TEETH)*nema17_steps_per_rev, // Axis 1 with nema 17 stepper
                        (140.0 / MOTOR_PULLEY_TEETH)*nema17_steps_per_rev, // Axis 2 with nema 17 stepper
                        ((140.0 / MOTOR_PULLEY_TEETH) * (110.0 / 100.0))*nema17_steps_per_rev, // Axis 3 with nema 17 stepper
                        1, // Axis 4 with servo
                        (70.0 / MOTOR_PULLEY_TEETH)*nema17_steps_per_rev, // Axis 5 with nema 17 stepper
                        2 * jyb28408_steps_per_rev, // Axis 6 with 28BYJ-48 stepper
                        1  // Gripper with small servo
                       };

// Create a new instance of the AccelStepper or Servo class for each joint
AccelStepper ax_1_stepper = AccelStepper(nema17_motorInterfaceType, X_STEP_PIN, X_DIR_PIN);
AccelStepper ax_2_stepper = AccelStepper(nema17_motorInterfaceType, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper ax_3_stepper = AccelStepper(nema17_motorInterfaceType, Z_STEP_PIN, Z_DIR_PIN);
Servo ax_4_servo;
AccelStepper ax_5_stepper = AccelStepper(nema17_motorInterfaceType, E_STEP_PIN, E_DIR_PIN);
AccelStepper ax_6_stepper = AccelStepper(jyb28408_motorInterfaceType, jyb28408_Pin1, jyb28408_Pin3, jyb28408_Pin2, jyb28408_Pin4);
Servo gripper_servo;

// Create a list of the instances
AccelStepper stepper_joints[] = {ax_1_stepper, ax_2_stepper, ax_3_stepper, 0, ax_5_stepper, ax_6_stepper, 0};
Servo servo_joints[] = {ax_4_servo, gripper_servo};

void setup() {
  Serial.begin(9600);
  Serial.println("Robotarm running");

  // Initialize the motors
  stepper_joints[0].setMaxSpeed(1000);
  stepper_joints[0].setAcceleration(200);
  stepper_joints[0].setEnablePin(X_ENABLE_PIN);
  stepper_joints[0].setPinsInverted(true, false, true);
  stepper_joints[0].enableOutputs();

  stepper_joints[1].setMaxSpeed(1000);
  stepper_joints[1].setAcceleration(200);
  stepper_joints[1].setEnablePin(Y_ENABLE_PIN);
  stepper_joints[1].setPinsInverted(false, false, true);
  stepper_joints[1].enableOutputs();

  stepper_joints[2].setMaxSpeed(1000);
  stepper_joints[2].setAcceleration(200);
  stepper_joints[2].setEnablePin(Z_ENABLE_PIN);
  stepper_joints[2].setPinsInverted(true, false, true);
  stepper_joints[2].enableOutputs();

  servo_joints[0].attach(SERVO_1_PIN, axis_4_min, axis_4_max);

  stepper_joints[4].setMaxSpeed(1000);
  stepper_joints[4].setAcceleration(200);
  stepper_joints[4].setEnablePin(E_ENABLE_PIN);
  stepper_joints[4].setPinsInverted(true, false, true);
  stepper_joints[4].enableOutputs();

  stepper_joints[5].setMaxSpeed(80);
  stepper_joints[5].setAcceleration(20);

  servo_joints[1].attach(SERVO_2_PIN, gripper_min, gripper_max);

  // ENDSTOP SETUP
  for (int i = 0; i <= 6; i++){
    pinMode(endstops[i], INPUT_PULLUP); // Configure as input
  }

  // Reset the robot
  TriggerEndstops();
  Home();
  Serial.println("ok init");
}

void loop() {
  ReadSerial();
}


void MoveJoint(int joint, float angle, int vel) {
  // Moves one joint and blocks the program until the joint has reached it's position
  // Velocity is the absolute vaule of the travel speed
  // Angle is the absolute positon of the joint in degrees compared to the home positon

  // Check that the joint is valid
  if (joint >= 0 && joint <= 6) {
    
    // Check if the joint is driven by a servo
    if (joint == 3 || joint == 6) {
      servo_joints[JointToServo(joint)].writeMicroseconds(AngleToPulse(angle, joint));
      delay(2000);  

    // The joint is driven by a stepper motor
    } else {
      int old_pos;
      stepper_joints[joint].setMaxSpeed(vel);
      stepper_joints[joint].setAcceleration(vel/4);
      int steps = stepsPerDeg[joint] * (angle / 360.0);
      stepper_joints[joint].moveTo(steps);

      // Because joint 1 and 2 are connected, we have to move
      // joint 2 the same distance as joint 1 in order for it
      // to stay in the same position relative to joint 1
      if (joint == 1){
        // have to save the actual angle of the joint
        stepper_joints[2].setMaxSpeed(vel);
        stepper_joints[2].setAcceleration(vel/4);
        int steps_ax2 = stepper_joints[1].distanceToGo()*-1;
        old_pos = stepper_joints[2].currentPosition();
        stepper_joints[2].move(steps_ax2);
      }
      // Do a step until the motor has reached its goal
      while (stepper_joints[joint].isRunning() ) {
        stepper_joints[joint].run();
        if (joint == 1){
          stepper_joints[2].run();
        }
      }
      // Set the position of joint 2 to what it was before
      // since it has not moved relative to joint 1
      if (joint == 1){
        stepper_joints[2].setCurrentPosition(old_pos);
      }
    }
  }
}

// Vi antar at det alltid er en stepper motor som skal lengst

void MoveJoints(int pos[], float vel) {
  // Moves all joints to the postions given in the array
  // The speed is for the stepper that travels the furthest distance
  
  float travel_dist[7] = {0, 0, 0, 0, 0, 0, 0};
  float step_size[7] = {0, 0, 0, 0, 0, 0, 0};
  int longest_dist[2] = {0, 0}; // The joint position and the joint
  float interpolation_factor = 1000.0;
  int ax_2_pos; // The positon of the stepper before we do anything
  int ax_2_travel_without_link;
  
  // Calcualte the steps and travel lengths for all joints
  int steps; // The number of units the motor would have to travel from its zero position
  int dist; // The number of units the motor actually has to travel, considering its current position
  for (int i = 0; i <= 6; i++) {
    // Servo code
    if (i == 3 || i == 6){
      // The servos steps is the number of degrees (1:1 ratio) to reach its position from zero
      steps = pos[i];
      // Find the current position of the servo to find the distance it has to travel
      dist = steps - servo_joints[JointToServo(i)].read();

    // Adjust for the motion link of joint 2
    } else if (i == 2) { 
      // Calculate the steps it would have to travel without the motion link
      ax_2_pos = stepper_joints[2].currentPosition(); 
      steps = stepsPerDeg[2] * (pos[2] / 360.0);
      stepper_joints[2].moveTo(steps);
      // This is the travel length without the motion link, and
      // the length we need to say that the motor has travelled
      // in order for the steps to match the angle of the joint
      ax_2_travel_without_link = stepper_joints[2].distanceToGo(); 

      // Account for the motion link
      steps = stepsPerDeg[2] * (pos[2] / 360.0) - stepper_joints[1].distanceToGo();
      stepper_joints[i].moveTo(steps);

      // Find the real distance it has to travel
      dist = stepper_joints[i].distanceToGo();

    } else {

      steps = stepsPerDeg[i] * (pos[i] / 360.0);
      stepper_joints[i].moveTo(steps);

      // Find the real distance it has to travel
      dist = stepper_joints[i].distanceToGo();
    }
    // Add the distance to the array and calculate the step size
    travel_dist[i] = dist;
    step_size[i] = travel_dist[i] / interpolation_factor;

    // Find the the length longest distance one of the motors has to travel
    if (abs(dist) > abs(longest_dist[0])) {
      longest_dist[0] = abs(dist);
      longest_dist[1] = i;
    }
  }

  // Adjust the speed of each joint
  float joint_vel;
  for (int i = 0; i <= 6; i++) {
    if (i == 3 || i == 6) {
      // Servo code
      // Implement servo speed code
    } else {
      // Let's say that we want all acceleartion to be done in 4 seconds
      joint_vel = (travel_dist[i] / longest_dist[0]) * vel;
      stepper_joints[i].setMaxSpeed(joint_vel);
      stepper_joints[i].setAcceleration(joint_vel/4);
      
    }
  }


  // Run each joint
  int counter = 1;
  float servo_start[2] = {0,0};
  servo_start[0] = servo_joints[0].read();
  delay(10);
  servo_start[1] = servo_joints[1].read();
  delay(10);


// Antar at servoen aldri kjører lengst
  while (stepper_joints[longest_dist[1]].isRunning()) { // this will not work if the loop runs and a step is not finished
    for (int i = 0; i <= 6; i++) {
      if ((i == 3 || i == 6) && counter % 3 == 0) {
        // Move the servo to the startposition plus x*step size. The step size can be negative
        float new_pos = servo_start[JointToServo(i)] + (step_size[i]*counter);
        servo_joints[JointToServo(i)].writeMicroseconds(AngleToPulse(new_pos, i));
        delay(0.5);

      } else {
        // If the stepper with the longest travel distance has reached its position, increase the counter
        // addde <=, could cause probelms
        if (i == longest_dist[1] && abs(stepper_joints[i].distanceToGo()) <= abs(travel_dist[i] - (counter * step_size[i]))) {
          counter ++;
        }
          stepper_joints[i].run();
      }

    }
  }
  // Set the correct position of joint 2
  int real_pos = ax_2_pos + ax_2_travel_without_link;
  stepper_joints[2].setCurrentPosition(real_pos);
}
/*
Replace MoveJoints() by two functions
We would have to make longest_dist, travel_dist and step_size global vars
We would set the interpolation to longest_dist/200
SetJoints() would do everything in MoveJoints before the while(), and have the same parameters as MoveJoints
StepJoints() would do the while loop, and have zero parameters
By giving SetJoints small intervals (maybe 2 degrees on the biggest joint) at a time
we will travel in small arcs from each point, creating a quite straight line,
depending on the number of points
*/

void TriggerEndstops() {
  // Trigger the endstop for each axis one
  // joint at a time. Then, move all of the joints
  // to their home position

  int steps;
  int joint;
  int home_sequence[7] = {1, 0, 2, 3, 5, 4, 6};
  for (int i = 0; i <= 6; i++){
    joint = home_sequence[i];
    if (joint == 3 || joint == 6){
      MoveJoint(joint, 0, 100);
      // Do not do anythin if it is a servo
      Serial.println("Servo joint, no endstop to trigger");
    } else {
      // Get the max distance the joint potentially has to travel
      // Make it negative because we have defined negative as twoards the endstop
      steps = stepsPerDeg[joint] * -1; 
      stepper_joints[joint].move(steps);
      
      stepper_joints[joint].setMaxSpeed(1000);
      stepper_joints[joint].setAcceleration(250);
      // Move joint 2 if we move joint 1 due to the moition link
      if (joint == 1){
        stepper_joints[2].move(steps*-1);
        stepper_joints[2].setMaxSpeed(1000);
        stepper_joints[2].setAcceleration(250);
      }
      // Move the joint until the endstop is triggered
      while(digitalRead(endstops[joint]) == 0){
        stepper_joints[joint].run();
        if (joint == 1){
          stepper_joints[2].run();
        }
      }
      Serial.println("Endstop triggered");
      // Stop the motor from moving and reset the postion
      stepper_joints[joint].stop();
      stepper_joints[joint].setCurrentPosition(0);
      if (joint == 1){
          stepper_joints[2].stop();
          stepper_joints[2].setCurrentPosition(0);
        }
    }
  }
}

void Home(){
  /* Move the motors to their home position
   * defined by the kinematics script in python
  */

  MoveJoints(homeJointPosition, 2000);
  stepper_joints[0].setCurrentPosition(0);
  stepper_joints[1].setCurrentPosition(0);
  stepper_joints[2].setCurrentPosition(0);
  stepper_joints[4].setCurrentPosition(0);
  stepper_joints[5].setCurrentPosition(0);
}

int AngleToPulse(int ang, int joint) {
  // Converts angle to a pulselength for servo motors
  int pulse = map(ang, 0, 180, minJointPositon[joint], maxJointPosition[joint]);
  return pulse;
}

void ReadSerial(){
  // Allows the user to control the robot arm with serial commands

  // Define the different commands
  const char* a = "MoveJoint";
  const char* b = "MoveJoints";
  const char* h = "Home";
  const char* e = "TriggerEndstops";

  // Get the command from the user 
  Serial.println("Enter your command (MoveJoint, MoveJoints, Home, TriggerEndstops)");
  while (Serial.available() == 0){
    // Wait for user input
  }
  String command = Serial.readStringUntil('\n');
  command.trim();
  Serial.println(command);

  // Temporary holder for the user input
  String tmp_in;
  if (command == a){
    // Try sending something like 0,90,1000, with readStringUntil set to ','
    // According to documentation the , will be removed,
    // and we can then call the same function all over again
    // https://www.arduino.cc/reference/en/language/functions/communication/serial/readstringuntil/
    
    Serial.println("Enter joint, angle and velocity on three different lines");

    float input_data[3] = {0, 0, 0};
    // Get the joint, angle and velocity
    for (int i = 0; i < 3; i++){
      while (Serial.available() == 0){
      // Wait for user input
      }
      tmp_in = Serial.readStringUntil('\n');
      if (tmp_in == "q"){
        Serial.println("Quit");
        return;
      }
      input_data[i] = tmp_in.toInt();
      Serial.print(input_data[i]); Serial.print(", ");
    }
    Serial.println("");
  
    // Call the function with the given parameters
    MoveJoint(input_data[0], input_data[1], input_data[2]);
    Serial.println("finish");

  } else if (command == b){
    Serial.println("Enter angle of joint 0-6 (inculding the gripper) and the top velocity on eight different lines");
    // Input data. Axis 0-5 + gripper
    int input_angles[7] = {0, 0, 0, 0, 0, 0, 0};
    int input_vel;

    for (int i = 0; i < 8; i++){
      while (Serial.available() == 0){
      // Wait for user input, do the stepping
      }
      tmp_in = Serial.readStringUntil('\n');
      if (tmp_in == "q"){
        Serial.println("Quit");
        return;
      }
      if (i != 7) {
        // The input is one of the seven angles for the arm
        input_angles[i] = tmp_in.toInt();
        Serial.print(input_angles[i]); Serial.print(", ");
      } else {
        // The input is the last element, which is the top velocity
        input_vel = tmp_in.toInt();
        Serial.println(input_vel);
      }
    }
    MoveJoints(input_angles, input_vel);
    Serial.println("finish");

  } else if (command == h){
    Home();
    Serial.println("finish");
    
  } else if (command == e){
    TriggerEndstops();
    Serial.println("finish");
  }
  else {
    Serial.println("The input did not match any commands");
  }
}

int JointToServo(int joint){
  if (joint == 3){
    return 0;
  } else if (joint == 6){
    return 1;
  }
}
