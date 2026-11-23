/* camera.ino/* camera.ino
*  Functions for communicating with the HuskyLens camera
*
*  @author Matthew Ford
*/

/* A struct is a C++ data type that groups related variables together.
*  Here, TagPose represents the status, range, and heading of a detected tag.
*  You may delete this comment and replace it with your own.
*/
struct TagPose {
  int status;  // 1 if tag is found, 0 if not found, -1 if error
  int range;   // [cm] Range to tag
  //int XCenter; // added to readjust the x center
  int heading;  // [deg] Heading to tag relative to camera
};
TagPose rangeAndHeadingToTag(int tag_id);
// Print a HuskyLens result
void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  } else {
    Serial.println("Object unknown!");
  }
}

/* Get the X coordinate of a specified object in the HuskyLens view.
*
*  This function requires HUSKLENS cam to be defined as a global variable.
*
*  @param obj_id ID of the target object
*  @return
*     x coordinate (between 0-360) of desired object, if found.
*    -1 if object not found
*    -2 if other communication error
*/
int getObjCenterX(int obj_id) {
  HUSKYLENSResult result;

  if (!cam.request()) {
    return -2;  // Communication error
  }

  while (cam.available()) {
    result = cam.read();  // Check next available block
    if (result.ID == obj_id) {
      return result.xCenter;
    }
  }

  return -1;  // Object not found
}

/* Estimate distance to tag based on apparent size
*
*  @param tag_id ID of the desired tag
*  @return
*     Estimated distance to desired tag (cm), if found
*     -1 if tag not found
*     -2 if other communication error
*/
int rangeToTag(int tag_id) {
  rangeAndHeadingToTag(tag_id);  //added so print statements would work
  HUSKYLENSResult result;

  if (!cam.request()) {
    return -2;  // Communication error
  }

  while (cam.available()) {
    result = cam.read();  // Check next available block
    if (result.ID == tag_id) {
      return 3851.0 / result.height - 15.7;  // Based on experimental data
    }
  }

  return -1;  // Object not found
}
// AI Gen Code Below
TagPose rangeAndHeadingToTag(int tag_id) {
  TagPose pose = { -1, 0, 0 };

  if (!cam.request()) {
    pose.status = -2;
    return pose;
  }
  //AI Gen Code to obtain heading and range
  //heading is calculated using (Xtag - 160)*.136 which was found experimentally
  while (cam.available()) {
    HUSKYLENSResult result = cam.read();

    if (result.ID == tag_id && result.height > 0) {
      pose.status = 1;
      pose.range = 3851.0 / result.height - 15.7;     //new experimetnal data
      pose.heading = (160 - result.xCenter) * 0.136;  // adjusted from .18 to .136 deg per pixle
      Serial.print("Range: ");
      Serial.print(pose.range);
      Serial.print(" cm, Heading: ");
      Serial.println(pose.heading);
      return pose;
      delay(3000);
    }
  }

  return pose;
}