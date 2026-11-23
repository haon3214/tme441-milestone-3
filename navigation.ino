/* navigation.ino
*  Functions for navigating the robot to April tags using the HuskyLens camera
*
*  @author Matthew Ford
*/


/* Find and approach the specified April tag
*
*  @param tag_id ID number of the desired tag
*  @return 
*     1 = Success
*    -1 = Searched unsuccessfully for a tag for 20 sec
*    -2 = Other error, or lost connection with HuskyLens
*/
int driveToTag(int tag_id) {

  int status;
  int range;

  status = pointToTag(tag_id);  // First search
  if (status == -2) { return -2; }
  if (status == -1) { return -1; }

  // Check range to tag
  range = rangeToTag(tag_id);

  // Drive to tag, stopping 10 cm short
  driveDistance(range - 10);
  // driveWithCorrection(tag_id);  //AI function listed at bottom
  return 1;
}

/* Rotate robot to point to the specified April tag
*
*  @param tag_id ID number of the desired tag
*  @return
*     1 = Success
*    -1 = Search unsuccessfully for 20 sec
*    -2 = Other error, or lost connection with HuskyLens
*/

//Replaced pointToTag with version that is compatable with rangeAndHeadingToTag
//This version should move smoothely
int pointToTag(int tag_id) {

  unsigned long start_time = millis();

  const long searchSpeed = 300;
  const long minTurnSpeed = 100;
  const float turnK = 25;

  resetPID();

  while (millis() - start_time < 30000) {
    updateOdometry();  // Keep (x, y, theta) current

    TagPose pose = rangeAndHeadingToTag(tag_id);

    if (pose.status == -2) {
      motorHalt(LMOTOR);
      motorHalt(RMOTOR);
      resetPID();
      return -2;
    }

    // Tag not found: continuously rotate
    if (pose.status != 1) {
      updatePID(-searchSpeed, searchSpeed, false);
      continue;
    }

    // Tag centered within 3 degrees
    if (abs(pose.heading) <= 3) {
      motorHalt(LMOTOR);
      motorHalt(RMOTOR);
      resetPID();

      Serial.println("Tag centered!");
      return 1;
    }

    // Gradually reduce rotation speed near center
    long turnSpeed = constrain(
      abs(pose.heading) * turnK,
      minTurnSpeed,
      searchSpeed);

    // Positive heading = counterclockwise
    if (pose.heading < 0) {
      turnSpeed = -turnSpeed;
    }

    updatePID(-turnSpeed, turnSpeed, false);
  }

  motorHalt(LMOTOR);
  motorHalt(RMOTOR);
  resetPID();

  return -1;
}
