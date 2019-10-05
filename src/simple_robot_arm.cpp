/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "application.h"
#line 1 "/Users/rob/Downloads/git/simple_robot_arm/src/simple_robot_arm.ino"
/******************************************************/
//                                                    //
// simple_robot_arm                                   //
// designed to work with a simple 4DOF robot arm      //
// with 4 servo motors                                //
// which receives a JSON instructing angle of any one //
// or more servos                                     //
// MIT licence                                        //
// from https://github.com/mnbf9rca/simple_robot_arm  //
//                                                    //
/******************************************************/

#include <ArduinoJson.h>

// JSON key names for the 4 servos
void publishInt(const char *name, int integer);
void publishChar(const char *name, const char *message);
int midpoint(int minimum, int maximum);
void toggleServos();
void setup();
bool didAttachServer(bool result, uint16_t pin);
int moveServoFunc(String extra);
bool safeSetTargetValue(int *target, int *receivedValue, int minValue, int maxValue);
void checkIfOutsideRange(int *target, int *receivedValue, const char *servoName);
void parseAndStoreJson(const char *json);
void setServos();
void writeToServo(Servo *servo, int target);
void loop();
#line 16 "/Users/rob/Downloads/git/simple_robot_arm/src/simple_robot_arm.ino"
#define SERVO_BASE_NAME "base"
#define SERVO_LEFT_NAME "left"
#define SERVO_RIGHT_NAME "right"
#define SERVO_GRIP_NAME "grip"

// pinout for each servo
#define SERVO_BASE_PIN D0
#define SERVO_LEFT_PIN D1
#define SERVO_RIGHT_PIN D2
#define SERVO_GRIP_PIN D3

//min and max ranges for each servo
#define SERVO_BASE_MIN 0
#define SERVO_BASE_MAX 180
#define SERVO_LEFT_MIN 30
#define SERVO_LEFT_MAX 80
#define SERVO_RIGHT_MIN 60
#define SERVO_RIGHT_MAX 150
#define SERVO_GRIP_MIN 5
#define SERVO_GRIP_MAX 40

// macros to accomodate different size key names
#define JSON_TEMPLATE "{\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d}"
#define LENGTH_OF_JSON_KEYS (sizeof(SERVO_BASE_NAME) + sizeof(SERVO_LEFT_NAME) + sizeof(SERVO_RIGHT_NAME) + sizeof(SERVO_GRIP_NAME))

Servo servoBase;  // base
Servo servoLeft;  // left
Servo servoRight; // right
Servo servoGrip;  // gripper

int targetBase = 90;
int targetLeft = 30;
int targetRight = 90;
int targetGrip = 20;

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

void toggleServos()
{
  // when starting up, servos seem to get stuck. this moves them to one way then another.

  char *buf;                                              //hold the JSON string
  buf = (char *)malloc(sizeof(JSON_TEMPLATE) -            //raw template
                       16 * sizeof(char) +                // 4 x %s and 4 x %d
                       LENGTH_OF_JSON_KEYS +              // the length of the text of the JSON keys
                       4 * snprintf(NULL, 0, "%d", 180)); // 4 integer values
  if (buf == NULL)
  {
    publishChar("Out of memory", "toggleServos");
    free(buf);
    return;
  }
  // first, let's set to the min values
  sprintf(buf,
          JSON_TEMPLATE,
          SERVO_BASE_NAME,
          SERVO_BASE_MIN,
          SERVO_LEFT_NAME,
          SERVO_LEFT_MIN,
          SERVO_RIGHT_NAME,
          SERVO_RIGHT_MIN,
          SERVO_GRIP_NAME,
          SERVO_GRIP_MIN);

  parseAndStoreJson(buf);
  setServos();
  delay(1000); // wait for it to move

  // now return all to midpoints
  sprintf(buf,
          JSON_TEMPLATE,
          SERVO_BASE_NAME,
          midpoint(SERVO_BASE_MIN, SERVO_BASE_MAX),
          SERVO_LEFT_NAME,
          midpoint(SERVO_LEFT_MIN, SERVO_LEFT_MAX),
          SERVO_RIGHT_NAME,
          midpoint(SERVO_RIGHT_MIN, SERVO_RIGHT_MAX),
          SERVO_GRIP_NAME,
          midpoint(SERVO_GRIP_MIN, SERVO_GRIP_MAX));

  parseAndStoreJson(buf);
  setServos();
  free(buf);
}

void setup()
{
  if (!didAttachServer(servoBase.attach(SERVO_BASE_PIN), 0))
  {
    return;
  } //base
  if (!didAttachServer(servoLeft.attach(SERVO_LEFT_PIN), 1))
  {
    return;
  } //left
  if (!didAttachServer(servoRight.attach(SERVO_RIGHT_PIN), 2))
  {
    return;
  } //right
  if (!didAttachServer(servoGrip.attach(SERVO_GRIP_PIN), 3))
  {
    return;
  } //gripper

  Particle.function("moveServoFunc", moveServoFunc);

  // to let us monitor the values
  Particle.variable("targetBase", targetBase);
  Particle.variable("targetLeft", targetLeft);
  Particle.variable("targetRight", targetRight);
  Particle.variable("targetGrip", targetGrip);
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

int moveServoFunc(String extra)
{
  parseAndStoreJson(extra);
  setServos();
  return 0;
}

/***************
/
/ parse + store JSON
/
***************/
bool parseJson(JsonDocument(*doc), const char *json)
// Parse the json document, return in *doc, bool if ok
{
  DeserializationError err = deserializeJson(*doc, json);
  if (err)
  {
    publishChar("DeserializationError", err.c_str());
    return false;
  }
  else
  {
    publishChar("deserialized json", json);
    return true;
  }
}

bool safeSetTargetValue(int *target, int *receivedValue, int minValue, int maxValue)
// compares the receivedValue to min + max, and if it's between them, assigns it to *target
{
  if ((*receivedValue >= minValue) && (*receivedValue <= maxValue))
  {
    *target = *receivedValue;
    return true;
  }
  return false;
}

void checkIfOutsideRange(int *target, int *receivedValue, const char *servoName)
// checks if a value is wihtin allowed range for this specific servo
{
  int servo_min = 0;
  int servo_max = 0;

  if (strcmp(servoName, SERVO_BASE_NAME) == 0)
  {
    servo_min = SERVO_BASE_MIN;
    servo_max = SERVO_BASE_MAX;
  }
  else if (strcmp(servoName, SERVO_LEFT_NAME) == 0)
  {
    servo_min = SERVO_LEFT_MIN;
    servo_max = SERVO_LEFT_MAX;
  }
  else if (strcmp(servoName, SERVO_RIGHT_NAME) == 0)
  {
    servo_min = SERVO_RIGHT_MIN;
    servo_max = SERVO_RIGHT_MAX;
  }
  else if (strcmp(servoName, SERVO_GRIP_NAME) == 0)
  {
    servo_min = SERVO_GRIP_MIN;
    servo_max = SERVO_GRIP_MAX;
  }
  if (!safeSetTargetValue(target, receivedValue, servo_min, servo_max))
  {
    char *buf;
    // "{s: %s, v: %d, min: %d, max: %d}";
    size_t buffersz = (32 - 8) * sizeof(char) +                 //template less %s + 3 x %d
                      strlen(servoName) +                       //name of the servo
                      snprintf(NULL, 0, "%d", *receivedValue) + // requested value
                      snprintf(NULL, 0, "%d", servo_min) +      // min value
                      snprintf(NULL, 0, "%d", servo_max) +      // max value
                      1;
    buf = (char *)malloc(buffersz);

    if (buf == NULL)
    {
      publishChar("Out of memory", "safeSetTargetValue");
      free(buf);
      return;
    }
    snprintf(buf, buffersz, "{s: %s, v: %d, min: %d, max: %d}", servoName, *receivedValue, servo_min, servo_max);

    publishChar("failed to set servo", buf);
    free(buf);
  }
}

// fetches a value if it exists in teh JSON
bool safeGetKeyValue(JsonObject(*obj), int *target, const char *(valueToFetch))
{
  if (obj->containsKey(valueToFetch))
  {
    *target = obj->getMember(valueToFetch).as<int>();
    return true;
  }
  return false;
}

// fetches the value if it exists, then checks if it's inside the valid range before setting
void fetchValueAndSet(JsonObject(*objptr), int(*targetPtr), const char *(valueToFetch))
{
  int *receivedValuePtr = (int *)malloc(sizeof(int));
  if (receivedValuePtr == NULL)
  {
    publishChar("Out of memory", "fetchValueAndSet");
    free(receivedValuePtr);
    return;
  }
  if (safeGetKeyValue(objptr, receivedValuePtr, valueToFetch))
  {
    checkIfOutsideRange(targetPtr, receivedValuePtr, valueToFetch);
  }
  free(receivedValuePtr);
}

void parseAndStoreJson(const char *json)
{
  // receive the JSON for the servo states and store

  const size_t capacity = JSON_OBJECT_SIZE(4) + LENGTH_OF_JSON_KEYS;
  StaticJsonDocument<capacity> doc;

  if (!parseJson(&doc, json))
  {
    // didnt get a document...
    return;
  }

  // fetch values and set servos
  JsonObject obj = doc.as<JsonObject>();

  fetchValueAndSet(&obj, &targetBase, SERVO_BASE_NAME);
  fetchValueAndSet(&obj, &targetLeft, SERVO_LEFT_NAME);
  fetchValueAndSet(&obj, &targetRight, SERVO_RIGHT_NAME);
  fetchValueAndSet(&obj, &targetGrip, SERVO_GRIP_NAME);

  doc.clear(); // release memory
}

/***************
/
/ servo control
/
***************/
void setServos()
{
  // sets all 4 servos to the currently set values
  publishChar("setting servos", "");
  publishInt("targetBase", targetBase);
  writeToServo(&servoBase, targetBase);
  publishInt("targetLeft", targetLeft);
  writeToServo(&servoLeft, targetLeft);
  publishInt("targetRight", targetRight);
  writeToServo(&servoRight, targetRight);
  publishInt("targetGrip", targetGrip);
  writeToServo(&servoGrip, targetGrip);

  // toggle LED to indicate complete
  digitalWrite(D7, HIGH - digitalRead(D7));
  publishChar("exiting", "setServos");
}
void writeToServo(Servo *servo, int target)
{
  //  writes a specific servo
  // separated out to allow e.g. a delay to be added etc.
  servo->write(target);
  delay(2);
}

void loop()
{
}