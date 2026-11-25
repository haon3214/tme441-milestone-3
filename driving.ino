//when calling put in cm we want in function
const float TRACK_WIDTH_CM = 16.9;
const float WHEEL_DIAMETER_CM = 8.0;
const float MOTOR_CPR = 48.0;
const float GEAR_RATIO = 34.01;

// Encoder ticks per centimeter of wheel travel
const float TICKS_PER_CM =
  (MOTOR_CPR * GEAR_RATIO) / (PI * WHEEL_DIAMETER_CM);

void driveDistance(float distanceCm) {
  long target = lround(distanceCm * TICKS_PER_CM);


  if (target == 0) {
    return;
  }


  long targetTicks = abs(target);
  long sumStart = enc_l.read() + enc_r.read();


  // Desired speeds in encoder ticks per second
  const long minSpeed = 500;
  const long maxSpeed = 1250;


  // Each ramp covers 20% of the total distance
  long rampTicks = targetTicks / 5;


  // Prevent a zero-width ramp
  if (rampTicks < 1) {
    rampTicks = 1;
  }
  // Variables for recording speed and time
  unsigned long startTime = millis();
  unsigned long lastPrint = startTime;


  long previousL = enc_l.read();
  long previousR = enc_r.read();


  // Serial.println("time_ms,target_speed,actual_left,actual_right");


  resetPID();


  while (true) {
    updateOdometry();  // Keep (x, y, theta) current
    // Average encoder movement from the starting position
    long moved = abs(
      ((enc_l.read() + enc_r.read()) - sumStart) / 2);


    // Stop the loop when the target is reached
    if (moved >= targetTicks) {
      break;
    }


    long speed;


    // PHASE 1: ACCELERATE
    if (moved < rampTicks) {
      speed = map(
        moved,
        0,
        rampTicks,
        minSpeed,
        maxSpeed);
    }


    // PHASE 2: CRUISE
    else if (moved < targetTicks - rampTicks) {
      speed = maxSpeed;
    }


    // PHASE 3: DECELERATE
    else {
      long remaining = targetTicks - moved;


      speed = map(
        remaining,
        0,
        rampTicks,
        minSpeed,
        maxSpeed);
    }


    speed = constrain(speed, minSpeed, maxSpeed);


    // Reverse both motors for a negative distance
    if (target < 0) {
      speed = -speed;
    }


    updatePID(speed, speed, false);


    // Record data approximately every 100 milliseconds
    unsigned long now = millis();
    unsigned long dt = now - lastPrint;


    if (dt >= 100) {
      long currentL = enc_l.read();
      long currentR = enc_r.read();


      // Actual speed in encoder ticks per second
      float actualL = (currentL - previousL) * 1000.0 / dt;
      float actualR = (currentR - previousR) * 1000.0 / dt;


      // Serial.print(now - startTime);
      // Serial.print(",");
      // Serial.print(speed);
      // Serial.print(",");
      // Serial.print(actualL);
      // Serial.print(",");
      // Serial.println(actualR);


      previousL = currentL;
      previousR = currentR;
      lastPrint = now;
    }
  }


  motorHalt(LMOTOR);
  motorHalt(RMOTOR);
  resetPID();
}




/* Rotate the robot by a specified angle.
 *
 * Positive angle = counterclockwise
 * Negative angle = clockwise
 *
 * @param angle_deg Angle in degrees.
 */
void rotateCCW(int angle_deg) {
  long target = lround(
    abs(radians(angle_deg)) * (TRACK_WIDTH_CM / 2.0) * TICKS_PER_CM);


  if (target == 0) {
    return;
  }


  long startL = enc_l.read();
  long startR = enc_r.read();


  // Desired speeds in encoder ticks per second
  const long minSpeed = 200;
  const long maxSpeed = 1000;


  // Each ramp covers 20% of the total turn
  long rampTicks = target / 5;


  // Prevent a zero-width ramp
  if (rampTicks < 1) {
    rampTicks = 1;
  }


  resetPID();


  while (true) {
    updateOdometry();  // Keep (x, y, theta) current
    // Average distance traveled by the two wheels
    long moved = (abs(enc_l.read() - startL) + abs(enc_r.read() - startR)) / 2;


    // Stop the loop when the target is reached
    if (moved >= target) {
      break;
    }


    long speed;


    // PHASE 1: ACCELERATE
    if (moved < rampTicks) {
      speed = map(
        moved,
        0,
        rampTicks,
        minSpeed,
        maxSpeed);
    }


    // PHASE 2: CRUISE
    else if (moved < target - rampTicks) {
      speed = maxSpeed;
    }


    // PHASE 3: DECELERATE
    else {
      long remaining = target - moved;


      speed = map(
        remaining,
        0,
        rampTicks,
        minSpeed,
        maxSpeed);
    }


    speed = constrain(speed, minSpeed, maxSpeed);


    // Reverse the turn direction for a negative angle
    if (angle_deg < 0) {
      speed = -speed;
    }


    // Wheels move in opposite directions to turn
    updatePID(-speed, speed, false);
  }


  motorHalt(LMOTOR);
  motorHalt(RMOTOR);
  resetPID();
}
//Constant driving mode for comparision
void driveConstant(float distanceCm) {
  long target = lround(distanceCm * TICKS_PER_CM);


  if (target == 0) {
    return;
  }


  long targetTicks = abs(target);
  long sumStart = enc_l.read() + enc_r.read();


  long speed = 1500;


  if (target < 0) {
    speed = -speed;
  }


  unsigned long startTime = millis();
  unsigned long lastPrint = startTime;


  long previousL = enc_l.read();
  long previousR = enc_r.read();


  // Serial.println("time_ms,target_speed,actual_left,actual_right");


  resetPID();


  while (true) {
    updateOdometry();  // Keep (x, y, theta) current
    long moved = abs(
      ((enc_l.read() + enc_r.read()) - sumStart) / 2);


    if (moved >= targetTicks) {
      break;
    }


    updatePID(speed, speed, false);


    // Record data approximately every 100 milliseconds
    unsigned long now = millis();
    unsigned long dt = now - lastPrint;


    if (dt >= 100) {
      long currentL = enc_l.read();
      long currentR = enc_r.read();


      float actualL = (currentL - previousL) * 1000.0 / dt;
      float actualR = (currentR - previousR) * 1000.0 / dt;


      // Serial.print(now - startTime);
      // Serial.print(",");
      // Serial.print(speed);
      // Serial.print(",");
      // Serial.print(actualL);
      // Serial.print(",");
      // Serial.println(actualR);


      previousL = currentL;
      previousR = currentR;
      lastPrint = now;
    }
  }


  motorHalt(LMOTOR);
  motorHalt(RMOTOR);
  resetPID();
}



/* ======================================================================
*  DEAD RECKONING (odometry) + RETURN HOME
*
*  Tracks the robot's position (x, y) and heading (theta) from the wheel
*  encoders. (0, 0) is where the robot starts, +x is the direction it
*  starts facing, +y is to its left, and theta is counterclockwise
*  positive in radians.
*
*  Each update:
*    ds     = (dL + dR) / 2               distance the center moved [cm]
*    dtheta = (dR - dL) / TRACK_WIDTH     change in heading [rad]
*    theta += dtheta
*    x     += ds * cos(theta)
*    y     += ds * sin(theta)
*
*  updateOdometry() is called at the top of every driving loop (here and
*  in pointToTag() in navigation.ino).
* ====================================================================== */

float pose_x = 0, pose_y = 0, pose_theta = 0;  // [cm], [cm], [rad]
long odo_last_l = 0, odo_last_r = 0;           // encoder counts at last update

// Keep an angle between -pi and +pi
float wrapAngle(float a) {
  while (a > PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

// Make the robot's current spot the origin (0, 0), facing 0
void initOdometry() {
  odo_last_l = enc_l.read();
  odo_last_r = enc_r.read();
  pose_x = pose_y = pose_theta = 0;
}

// Add the wheel motion since the last call to the position estimate
void updateOdometry() {
  long l = enc_l.read(), r = enc_r.read();
  float dL = (l - odo_last_l) / TICKS_PER_CM;
  float dR = (r - odo_last_r) / TICKS_PER_CM;
  odo_last_l = l;
  odo_last_r = r;

  float ds = (dL + dR) / 2;
  pose_theta = wrapAngle(pose_theta + (dR - dL) / TRACK_WIDTH_CM);
  pose_x += ds * cos(pose_theta);
  pose_y += ds * sin(pose_theta);
}

// Turn toward the start, drive straight to it, then face the start direction
void returnHome() {
  updateOdometry();
  float turn = wrapAngle(atan2(-pose_y, -pose_x) - pose_theta);
  rotateCCW(round(degrees(turn)));  // face the start point

  updateOdometry();
  driveDistance(hypot(pose_x, pose_y));  // drive straight to it

  updateOdometry();
  rotateCCW(round(degrees(-pose_theta)));  // face the original direction
}
