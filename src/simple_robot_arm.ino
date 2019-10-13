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

// ROS includes
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
Servo servo[4];                                                  // holds the servo objects
int servoMinMax[][2] = {{0, 180}, {30, 80}, {60, 150}, {5, 40}}; //set of {min, max} values for each servo
int servoPins[] = {D0, D1, D2, D3};                              // which pin us each servo attached to

int targetAngle[] = {90, 30, 90, 20};

// joints
char *jointNames[] = {"base_block_rotating_block_joint",
                      "rotating_block_vertical_arm_joint",
                      "vertical_arm_forearm_joint",
                      "actuator_forearm_joint"};

int number_of_joints = sizeof(jointNames);
float currentAngle[] = {0, 0, 0, 0};

// ROS handlers
ros::NodeHandle nh;

void updateCurrentAngles()
{
  // updates the currentAngle with the current servo value
  // NB this is not complete. Joint angles need to be computed
  for (int i = 0; i < 4; i++)
  {
    currentAngle[i] = servo[i].read();
  }
}

// helper function to publish integers e.g. for debugging
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

void publishChar(const char *name, const char *message)
{
  Particle.publish(name, message, 0, PRIVATE);
  delay(1000); // ensure not throttled
}
int midpoint(int minimum, int maximum)
{
  return minimum + ((maximum - minimum) / 2);
}

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

ros::Publisher joint_publisher("test", &joint_msg);
void publishJointAngles()
{
  if (!nh.connected())
  {
    return;
  }
  updateCurrentAngles();
  joint_msg.header.stamp = nh.now();
  joint_msg.name = jointNames;
  joint_msg.name_length = 4;
  joint_msg.position = currentAngle;
  joint_msg.position_length = 4;
  joint_publisher.publish(&joint_msg);
  publishInt("here", 8);
  // 1 = size of message queue
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
  /*
  Particle.variable("currentAngle[0]", currentAngle[0]);
  Particle.variable("currentAngle[1]", currentAngle[1]);
  Particle.variable("currentAngle[2]", currentAngle[2]);
  Particle.variable("currentAngle[3]", currentAngle[3]);
  */
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

int boundValue(int *receivedValue, int *minValue, int *maxValue)
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