/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "application.h"
#line 1 "/Users/rob/Downloads/git/simple_robot_arm_with_ros/src/simple_robot_arm.ino"
/******************************************************/
//                                                    //
// simple_robot_arm                                   //
// designed to work with a simple 4DOF robot arm      //
// with 4 servo motors                                //
// which receives a joint angle published via ROS for //
// one or more servos as <servo>/<angle> on /servo    //
// MIT licence                                        //
// from https://github.com/mnbf9rca/simple_robot_arm  //
//                                                    //
/******************************************************/

#include <string.h>

// ROS includes
inline double degToRad(int degrees);
inline double radToDeg(int radians);
void updateCurrentAngles();
void publishInt(const char *name, int integer);
void publishChar(const char *name, const char *message);
int midpoint(int minimum, int maximum);
int getAdjustedServoAngle(const int servoID);
int calculateBaseAngle();
int calculateVerticalArmAngle();
int calculateForearmAngle();
void publishJointAngles();
void setServos();
void toggleServos();
int cb_particle_servo(String extra);
void setup();
bool didAttachServer(bool result, uint16_t pin);
int boundValue(const int *receivedValue, const int *minValue, const int *maxValue);
void loop();
#line 16 "/Users/rob/Downloads/git/simple_robot_arm_with_ros/src/simple_robot_arm.ino"
#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <std_msgs/String.h> // we publish servo angles (to hit) as a string (Currently) #/###
#include <sensor_msgs/JointState.h>
#define ROS_SERVO_TOPIC servo

// ROS parameters
// Server settings
IPAddress server(192, 168, 17, 29); // IP of ROS server
uint16_t serverPort = 11411;

// ROS timing
uint16_t period = 1000;
uint32_t last_time = 0;

// Servo parameters
Servo servo[4];                                                        // holds the servo objects
const int servoMinMax[][2] = {{0, 180}, {10, 85}, {40, 150}, {5, 40}}; // set of {min, max} values for each servo
const int servoPins[] = {D0, D1, D2, D3};                              // which pin us each servo attached to
const int servoDegreesOffset[] = {0, 10, 0, 0};                        // how many degrees is each servo off by
int targetAngle[] = {90, 30, 90, 20};                                  // what's the target angle for this servo
float currentAngle[] = {0, 0, 0, 0};                                   // what's the current angle reported by the servo. Take account of range capping.

// joints
#define J1_NAME "base_block_rotating_block_joint"
#define J2_NAME "rotating_block_vertical_arm_joint"
#define J3_NAME "vertical_arm_forearm_joint"
#define J4_NAME "actuator_forearm_joint"
#define NUMBER_OF_JOINTS 4

char *jointNames[NUMBER_OF_JOINTS]; // hold joint names

// misc
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

// ROS handlers
ros::NodeHandle nh;
/**
 * converts degrees to radians
 */
inline double degToRad(int degrees)
{
  return degrees * (M_PI / 180.0);
}
/**
 * converts radians to degrees
 */
inline double radToDeg(int radians)
{
  return radians * (180.0 / M_PI);
}
/**
 * stores the currently set joint angle in currentAngle
 * taking account of clamping
 */
void updateCurrentAngles()
{
  for (int i = 0; i < 4; i++)
  {
    currentAngle[i] = servo[i].read();
  }
}

/**
 * helper function to publish integers e.g. for debugging
 */
void publishInt(const char *name, int integer)
{
  char *buf;
  buf = (char *)malloc(snprintf(NULL, 0, "%d", integer));
  if (buf == NULL)
  {
    publishChar("Out of memory", "publishInt");
    return;
  }
  sprintf(buf, "%d", integer);
  publishChar(name, buf);
  free(buf);
}
/**
 * helper function to publish chars e.g. for debugging
 */
void publishChar(const char *name, const char *message)
{
  Particle.publish(name, message, 0, PRIVATE);
  delay(1000); // ensure not throttled
}
int midpoint(int minimum, int maximum)
{
  return minimum + ((maximum - minimum) / 2);
}
/**
 * checks that the received message is syntatically correct
 * i.e. is d/d[d[d]]
 * 
 * @instruction - the received text
 * @cx output of snprintf
 * @instructionAlloc size allocated to receive instruction
 */
bool validateReceivedMessage(char *const &instruction, int(*cx), const int(*instructionAlloc))
{
  if ((*cx < 0) || (*cx > *instructionAlloc - 1)) //less 1 for \n
  {
    publishInt("Message contains too many characters", *cx);
    return false;
  }

  // minimum instruction size is 3: 0/0
  if (*cx < 3)
  {
    publishInt("Message contains too few characters", *cx);
    return false;
  }
  // pull 1st char and cast to int
  if (!isdigit(instruction[0]))
  {
    publishChar("first character not digit", &instruction[0]);
    return false;
  }

  // check that 2nd char is /
  if (!(instruction[1] == '/'))
  {
    publishChar("Second character not /", &instruction[1]);
    return false;
  }
  // check that chars at 2 to cx are digits
  for (int i = 2; i < *cx; i++)
  {
    if (!isdigit(instruction[i]))
    {
      publishChar("character not digit", &instruction[i]);
      return false;
    }
  }
  return true;
}

/**
 * takes an instruction and executes it
 * 
 * @instruction - the received text
 * @cx output of snprintf
 * @instructionAlloc size allocated to receive instruction
 */
void extractServoAndSetAngle(char *const &instruction, int(*cx), const int(*instructionAlloc))
{

  char buf[*instructionAlloc - 2]; //hold the extracted characters in a buffer, maximum size is instructionAlloc-2
  char *p = instruction + 2;       // create pointer to start 2 chars in
  strncpy(buf, p, *cx - 2);

  buf[sizeof(buf) - 1] = '\0'; //remember to null terminate string
  int servoToSet = instruction[0] - '0';
  // publishInt("found servo", servoToSet);
  targetAngle[servoToSet] = atoi(buf);

  // publishInt("setting to", targetAngle[servoToSet]);
  setServos();
}

// ROS
/**
 * particle function callback
 * receives string with servo angle instruction and tries to execute it
 */
void servo_cb(const std_msgs::String &cmd_msg)
{
  // expecting it to be int/int e.g. 1/120 to set servo 1 to 120
  const int instructionAlloc = 6; // #/## +1 for \n
  char instruction[instructionAlloc];

  int cx;
  // safely capture the first instructionAlloc characters of the string
  //return is the number of characters printed (excluding the null byte used to end output to strings)
  // only when this returned value is non-negative and less than n, the string has been completely written
  // http://www.cplusplus.com/reference/cstdio/snprintf/
  cx = snprintf(instruction, instructionAlloc, cmd_msg.data);
  // check that the instruction fitted in to the buffer

  if (validateReceivedMessage(instruction, &cx, &instructionAlloc))
  {
    extractServoAndSetAngle(instruction, &cx, &instructionAlloc);
  }
}
ros::Subscriber<std_msgs::String> sub("servo", servo_cb);
sensor_msgs::JointState joint_msg;
std_msgs::String str_msg;

ros::Publisher joint_publisher("joint_states", &joint_msg);
/**
 * returns the current angle represnted by a servo adjusted for offset
 * typically, offset is measured empircally and configured
 * 
 * @servoID the ID of the specific servo to measure
 * 
 */
int getAdjustedServoAngle(const int servoID)
{
  return currentAngle[servoID] + servoDegreesOffset[servoID];
}
int calculateBaseAngle()
{
  return getAdjustedServoAngle(0) + 270;
}
/**
 * calculates the current angle of the vertical_arm
 * servo2 directly sets vertical_arm angle. It just needs to be rotated.
 */
int calculateVerticalArmAngle()
{
  // this is the raw angle set by servo 2 rotated
  return getAdjustedServoAngle(2) + 270;
}
/**
 * calculates the current angle of the forearm
 * This is based on the angle of servo1 adjusted for the angle of servo2
 * servo2 directly sets vertical_arm angle
 */
int calculateForearmAngle()
{
  // this is the angle of servo 1 adjusted for the angle of servo 2
  // where servo 2 = 90, the angle = servo1
  return getAdjustedServoAngle(1) + (calculateVerticalArmAngle() - 90);
}
/**
 * publishes the current joint angles to joint_state
 */
void publishJointAngles()
{
  if (!nh.connected())
  {
    return;
  }
  updateCurrentAngles();
  float computedAngle[NUMBER_OF_JOINTS];
  computedAngle[0] = degToRad(calculateBaseAngle());
  computedAngle[1] = degToRad(calculateVerticalArmAngle());
  computedAngle[2] = degToRad(calculateForearmAngle());
  computedAngle[3] = computedAngle[2];

  joint_msg.header.stamp = nh.now();
  joint_msg.name = jointNames;
  joint_msg.name_length = NUMBER_OF_JOINTS;
  joint_msg.position = computedAngle;
  joint_msg.position_length = NUMBER_OF_JOINTS;
  joint_publisher.publish(&joint_msg);
}

// servo control
void setServos()
{
  for (size_t i = 0; i < 4; i++)
  {
    servo[i].write(boundValue(&targetAngle[i],      // target
                              &servoMinMax[i][0],   //min
                              &servoMinMax[i][1])); //max
  }
  // toggle LED to indicate complete
  digitalWrite(D7, HIGH - digitalRead(D7));
}

void toggleServos()
{
  // when starting up, servos seem to get stuck. this moves them to one way then another.
  for (size_t i = 0; i < 4; i++)
  {
    targetAngle[i] = servoMinMax[i][0];
  }

  setServos();
  delay(1000); // wait for it to move
  for (size_t i = 0; i < 4; i++)
  {
    targetAngle[i] = midpoint(servoMinMax[i][0], servoMinMax[i][1]);
  }

  // now return all to midpoints

  setServos();
}
// Cloud functions must return int and take one String
int cb_particle_servo(String extra)
{

  // expecting it to be int/int e.g. 1/120 to set servo 1 to 120
  const int instructionAlloc = 6; // #/## +1 for \n
  char instruction[instructionAlloc];

  int cx;
  // safely capture the first instructionAlloc characters of the string
  //return is the number of characters printed (excluding the null byte used to end output to strings)
  // only when this returned value is non-negative and less than n, the string has been completely written
  // http://www.cplusplus.com/reference/cstdio/snprintf/
  cx = snprintf(instruction, instructionAlloc, extra);
  // check that the instruction fitted in to the buffer

  if (validateReceivedMessage(instruction, &cx, &instructionAlloc))
  {
    extractServoAndSetAngle(instruction, &cx, &instructionAlloc);
  }

  return 0;
}

void setup()
{
  for (size_t i = 0; i < 4; i++)
  {
    if (!didAttachServer(servo[i].attach(servoPins[i]), i))
    {
      return;
    }
  }

  Particle.function("particle_servo", cb_particle_servo);

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  char IP[] = "xxx.xxx.xxx.xxx";
  IPAddress ip = (nh.getHardware()->getLocalIP());
  ip.toString().toCharArray(IP, 16);

  publishChar("publishing from IP", IP);
  // subscribe to topic
  nh.subscribe(sub);
  nh.advertise(joint_publisher); // advertise that we publish joint states

  // to let us monitor the values
  Particle.variable("targetAngle[0]", targetAngle[0]);
  Particle.variable("targetAngle[1]", targetAngle[1]);
  Particle.variable("targetAngle[2]", targetAngle[2]);
  Particle.variable("targetAngle[3]", targetAngle[3]);

  // copy joint names to non const and store pointer in jointNames[]
  char *s;
  s = strdup(J1_NAME);
  jointNames[0] = s;
  s = strdup(J2_NAME);
  jointNames[1] = s;
  s = strdup(J3_NAME);
  jointNames[2] = s;
  s = strdup(J4_NAME);
  jointNames[3] = s;

  pinMode(D7, OUTPUT);
  toggleServos();
}
bool didAttachServer(bool result, uint16_t pin)
{
  // attach a servo to a pin and report success
  char servoUsed[3];
  sprintf(servoUsed, "%d", pin);
  if (result)
  {
    // publishChar("attached servo", servoUsed);
  }
  else
  {
    publishChar("Cannot attach to servo", servoUsed);
  }
  return result;
}

int boundValue(const int *receivedValue, const int *minValue, const int *maxValue)
{
  if (*receivedValue > *maxValue)
  {
    return *maxValue;
  }
  else if (*receivedValue < *minValue)
  {
    return *minValue;
  }
  return *receivedValue;
}

void loop()
{
  if (millis() - last_time >= period)
  {
    last_time = millis();
    if (nh.connected())
    {
      publishChar("Connected", "");
    }
    else
    {
      publishChar("Not Connected", "");
    }
  }
  publishJointAngles();
  nh.spinOnce();
  delay(1);
}