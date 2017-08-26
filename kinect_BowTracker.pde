import SimpleOpenNI.*;
SimpleOpenNI  kinect;

import oscP5.*;
import netP5.*;

OscP5 osc;
NetAddress addr;

OscMessage SHOULDER_ANGLE;
OscMessage ELBOW_ANGLE;
OscMessage RIGHTHAND_XYZ;
OscMessage RIGHTHAND_ANGLE;
OscMessage BOW_STROKE_LENGTH;
OscMessage BOW_SPEED;

PVector pRightHand; // store the previous location of the hand
float bowSpeed;
int receivePort = 6000; // Port for receive data
int sendPort = 12000; // Port for sending data
int userId;

void setup() { 
  size(640, 480);
  kinect = new SimpleOpenNI(this);
  kinect.enableDepth();
  kinect.enableUser(SimpleOpenNI.SKEL_PROFILE_ALL);
  kinect.setMirror(true);
  
  // init osc messages
  osc = new OscP5(this, receivePort);
  addr = new NetAddress("127.0.0.1", sendPort);

  
  
}
void draw() {
  kinect.update();
  PImage depth = kinect.depthImage();
  image(depth, 0, 0);
  IntVector userList = new IntVector();
  kinect.getUsers(userList);
  textSize(12);
  fill(255);
  text("sending osc data on port 12000", 8, 10);
  
  if (userList.size() > 0) {
    userId = userList.get(0);
    if ( kinect.isTrackingSkeleton(userId)) {
      // draw the right arm
      drawSkel(userId);
      
      // get the positions of the three joints of our arm and left hand
      PVector rightHand = new PVector();
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_HAND, rightHand);
      
      PVector convertedRightHand = new PVector();
      kinect.convertRealWorldToProjective(rightHand, convertedRightHand);
      convertedRightHand.normalize();
            
      PVector leftShoulder = new PVector();
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, leftShoulder);         
      PVector rightElbow = new PVector();
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, rightElbow); 
      PVector rightShoulder = new PVector();
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER,rightShoulder); 
 
      float shoulderAngle = angleOf(rightElbow.x, rightElbow.y, rightShoulder.x, rightShoulder.y, 0);  
      float elbowAngle = angleOf(rightElbow.x, rightElbow.y, rightHand.x, rightHand.y, HALF_PI);     
      float rightHandAngle = angleOf(rightHand.x, rightHand.y, leftShoulder.x, leftShoulder.y, 0); 
    
      // calculate the distance between the right hand of the left shoulder
      // we can use this to figure out the bow stroke length and direction 
      PVector diff = PVector.sub(leftShoulder, rightHand);       
      float bowStrokeLength = diff.mag();
      bowStrokeLength = constrain(bowStrokeLength, 150, 600);
      bowStrokeLength = map(bowStrokeLength, 150, 600, 0, 1);
      
      // calculate the bow speed    
      if (pRightHand != null) {
        PVector d = PVector.sub(rightHand, pRightHand);
        bowSpeed = d.mag() / frameRate;
        bowSpeed = constrain(bowSpeed, 0.0, 3.0);
        bowSpeed /= 3;
        bowSpeed *= 100;          
      }
      

      // send the data to max
      SHOULDER_ANGLE = new OscMessage("/kinect/shoulderAngle");
      SHOULDER_ANGLE.add(shoulderAngle);
      ELBOW_ANGLE = new OscMessage("/kinect/elbowAngle");
      ELBOW_ANGLE.add(elbowAngle);
      RIGHTHAND_ANGLE = new OscMessage("/kinect/rightHandAngle");
      RIGHTHAND_ANGLE.add(rightHandAngle);
      BOW_STROKE_LENGTH = new OscMessage("/kinect/bowStrokeLength");
      BOW_STROKE_LENGTH.add(bowStrokeLength);
      BOW_SPEED = new OscMessage("/kinect/bowSpeed");
      BOW_SPEED.add(bowSpeed);  
      RIGHTHAND_XYZ = new OscMessage("/kinect/rightHandXYZ");
      RIGHTHAND_XYZ.add(convertedRightHand.x);
      RIGHTHAND_XYZ.add(convertedRightHand.y);
      RIGHTHAND_XYZ.add(convertedRightHand.z);
      
      osc.send(SHOULDER_ANGLE, addr);
      osc.send(ELBOW_ANGLE, addr);
      osc.send(RIGHTHAND_ANGLE, addr);
      osc.send(BOW_STROKE_LENGTH, addr);
      osc.send(BOW_SPEED, addr);
      osc.send(RIGHTHAND_XYZ, addr);
      
      // set the previous right hand position
      pRightHand = rightHand;
      // show the angles on the screen for debugging
      fill(255, 0, 0);
      textSize(14);
      text("/kinect/shoulderAngle "   + int(shoulderAngle), 8, 30); 
      text("/kinect/elbowAngle "      + int(elbowAngle), 8, 45); 
      text("/kinect/rightHandAngle "  + int(rightHandAngle), 8, 60);  
      text("/kinect/bowStrokeLength "  + bowStrokeLength, 8, 75);
      text("/kinect/bowSpeed "        + int(bowSpeed), 8, 90); 
    }
  }
}


float angleOf(float X1, float Y1, float X2, float Y2, float OFFSET) {
  float a = OFFSET - atan2(Y1 - Y2, X1 - X2);
  a = a % TWO_PI;
  return degrees(a);
}

void drawSkel(int userId) {
  // draw the outshline of the player's bowing arm 
  stroke(0);
  strokeWeight(5);
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND);
  
  stroke(#3127CB);
  strokeWeight(2);
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HAND, SimpleOpenNI.SKEL_LEFT_SHOULDER);               
  
  // Draw points at each of the arm's joints. 
  drawJointAngle(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  drawJointAngle(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  drawJointAngle(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_HAND);


}

void drawJoint(int userId, int jointID) {
  PVector joint = new PVector();

  float confidence = kinect.getJointPositionSkeleton(userId, jointID, joint);
  if (confidence < 0.5) {
    return;
  }
  PVector convertedJoint = new PVector();
  kinect.convertRealWorldToProjective(joint, convertedJoint);
  
  noStroke();
  fill(#F7F002);
  ellipse(convertedJoint.x, convertedJoint.y, 15, 15);
}

void drawJointAngle(int userId, int jointID) {
  PVector joint = new PVector();

  float confidence = kinect.getJointPositionSkeleton(userId, jointID, joint);
  if (confidence < 0.5) {
    return;
  }
  PVector convertedJoint = new PVector();
  kinect.convertRealWorldToProjective(joint, convertedJoint);
  
  noStroke(); 
  fill(#B827CB);
  ellipse(convertedJoint.x, convertedJoint.y, 15, 15);
    
  stroke(0);
  strokeWeight(2);
  noFill();
  ellipse(convertedJoint.x, convertedJoint.y, 50, 50);
}


// user-tracking callbacks!
void onNewUser(int userId) {
  println("start pose detection");
  ellipse(width/2, height/2, 100, 100);
  kinect.startPoseDetection("Psi", userId);
}
void onEndCalibration(int userId, boolean successful) {
  if (successful) {
    println("  User calibrated !!!");
    kinect.startTrackingSkeleton(userId);
  } else {
    println("  Failed to calibrate user !!!");
    kinect.startPoseDetection("Psi", userId);
  }
}
void onStartPose(String pose, int userId) {
  println("Started pose for user");
  kinect.stopPoseDetection(userId);
  kinect.requestCalibrationSkeleton(userId, true);
}

