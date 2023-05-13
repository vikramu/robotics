// File:          kick_the_ball.cpp
// Date:          
// Description:   Kick the Soccer Ball with Keyboard
// Author:        Vikram Upadhyaya
// Modifications: A combination from walk.cpp and soccer.cpp

// Include Files from both walk.cpp and soccer.cpp
#include "Walk.hpp"  

#include "RobotisOp2GaitManager.hpp"
#include "RobotisOp2MotionManager.hpp"

#include "Accelerometer.hpp"
#include "Gyro.hpp"
#include "Keyboard.hpp"
#include "LED.hpp"
#include "Motor.hpp"
#include "PositionSensor.hpp"

#include "RobotisOp2VisionManager.hpp"
#include "Camera.hpp"
#include "Speaker.hpp"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

//-----------------------------

using namespace webots;
using namespace managers;
using namespace std;

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

Walk::Walk() : Robot() {
  mTimeStep = getBasicTimeStep();
  
  mEyeLED = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  mHeadLED->set(0x00FF00);
  mBackLedRed = getLED("BackLedRed");
  mBackLedGreen = getLED("BackLedGreen");
  mBackLedBlue = getLED("BackLedBlue");
  //getLED("HeadLed")->set(0xFF0000);
  //getLED("EyeLed")->set(0x00FF00);
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);
  mCamera = getCamera("Camera");
  mCamera->enable(2 * mTimeStep);

  getGyro("Gyro")->enable(mTimeStep);

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }

  mKeyboard = getKeyboard();
  mKeyboard->enable(mTimeStep);

  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
  mVisionManager = new RobotisOp2VisionManager(mCamera->getWidth(), mCamera->getHeight(), 28, 20, 50, 45, 0, 30);
}

Walk::~Walk() {
}

void Walk::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

bool Walk::getBallCenter(double &x, double &y) {
  static int width = mCamera->getWidth();
  static int height = mCamera->getHeight();

  const unsigned char *im = mCamera->getImage();
  bool find = mVisionManager->getBallCenter(x, y, im);

  if (!find) {
    x = 0.0;
    y = 0.0;
    return false;
  } else {
    x = 2.0 * x / width - 1.0;
    y = 2.0 * y / height - 1.0;
    return true;
  }
}




// function containing the main feedback loop
void Walk::run() {
  cout << "-------Walk example of ROBOTIS OP2-------" << endl;
  cout << "This example illustrates Gait Manager" << endl;
  cout << "Press the space bar to start/stop walking" << endl;
  cout << "Use the arrow keys to move the robot while walking" << endl;
  cout << "After approaching near Soccer Ball stop and to kick the Ball" << endl;
  cout << "position well and use \"L\" for Left Foot OR use \"R\" for Right Foot" << endl;

  // First step to update sensors values
  myStep();

  // set eye led to green
  mEyeLED->set(0x00FF00);

  // play the hello motion
  mMotionManager->playPage(9);  // init position
  wait(200);

  // main loop
  bool isWalking = false;

  while (true) {
    checkIfFallen();

    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);

    // get keyboard key
    int key = 0;
    while ((key = mKeyboard->getKey()) >= 0) {
      switch (key) {
        case ' ':  // Space bar
          if (isWalking) {
            mGaitManager->stop();
            isWalking = false;
            //wait(200);
          } else {
            mGaitManager->start();
            isWalking = true;
            //wait(200);
          }
          break;
        case 'L': //L for LeftKick
          mMotionManager->playPage(13);  // left kick
          break;
        case 'R': //L for RightKick
          mMotionManager->playPage(12);  // left kick
          break;
        case Keyboard::UP:
          mGaitManager->setXAmplitude(1.0);
          break;
        case Keyboard::DOWN:
          mGaitManager->setXAmplitude(-1.0);
          break;
        case Keyboard::RIGHT:
          mGaitManager->setAAmplitude(-0.5);
          break;
        case Keyboard::LEFT:
          mGaitManager->setAAmplitude(0.5);
          break;
      }
    }

    mGaitManager->step(mTimeStep);
    //kick();
    // step
    myStep();
  }
}

void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;

  // count how many steps the accelerometer
  // says that the robot is down
  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;

  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;

  // the robot face is down
  if (fup > acc_step) {
    mMotionManager->playPage(10);  // f_up
    mMotionManager->playPage(9);   // init position
    fup = 0;
  }
  // the back face is down
  else if (fdown > acc_step) {
    mMotionManager->playPage(11);  // b_up
    mMotionManager->playPage(9);   // init position
    fdown = 0;
  }
}

