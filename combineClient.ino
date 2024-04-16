#include <CircularBuffer.hpp>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <SparkFun_STHS34PF80_Arduino_Library.h>

#include "WirelessCommunication.h"
#include "sharedVariable.h"
#include "Preferences.h"

// added
#define leftsnsr A0 //changes these
#define rightsnsr A3 //changes these

volatile bool leftonly = false;
volatile bool rightonly = false;
volatile int count = 0;

unsigned long lastDetectionTime = 0; // time of the last detection
unsigned long cooldownPeriod = 2500; // cooldown period in milliseconds (5 seconds)
unsigned long leftActivationTime = 0; // time when the left sensor was activated
unsigned long rightActivationTime = 0; // time when the right sensor was activated
// added

SparkFun_VL53L5CX imager;
STHS34PF80_I2C presence;

// Used for weighted averages of ToF output frame
const float asym_weights[16] = {1. / 24, 1. / 24, 1. / 24, 1. / 24,
                                1. / 12, 1. / 12, 1. / 12, 1. / 12,
                                -1. / 12, -1. / 12, -1. / 12, -1. / 12,
                                -1. / 24, -1. / 24, -1. / 24, -1. / 24};

const float distance_weights[16] = {0, 0, 0, 0,
                                    0, 1. / 4, 1. / 4, 0,
                                    0, 1. / 4, 1. / 4, 0,
                                    0, 0, 0, 0};

float asym = 0;
float distance = 0;
int16_t pres = 0;

const float asym_thresh = 20;
const float presence_thresh_1 = 100;
const float presence_thresh_2 = 5000;

bool new_asym = false;
int recent_pres = 0;

int n_entered = 0;
int n_exited = 0;
uint32_t people = 0;
shared_uint32 x;

const int imager_int_pin = 18;
volatile bool imagerReady = false;

CircularBuffer<int8_t, 3> asym_buf;
CircularBuffer<int8_t, 15> pres_buf;

void setup()
{
  // added
  pinMode(leftsnsr, INPUT); 
  pinMode(rightsnsr, INPUT); 
  Serial.begin(9600); 
  // added

  pinMode(imager_int_pin, INPUT);
  Wire.begin();           // This resets I2C bus to 100kHz
  Wire.setClock(1000000); // Sensor has max I2C freq of 1MHz

  // initialize WiFi
  init_wifi_task();
  INIT_SHARED_VARIABLE(x, people); // init shared variable used to tranfer info to WiFi core

  // Initialize ToF Imager
  if (imager.begin() == false)
  {
    Serial.println("Imaging sensor not found - check your wiring.");
    while (1)
      ;
  }

  // Initialize Presence Sensor
  if (presence.begin() == false)
  {
    Serial.println("Presence sensor not found - check your wiring.");
    while (1)
      ;
  }

  // Set resolution, sampling frequency, and interrupt for ToF imager
  imager.setResolution(4 * 4);
  imager.setRangingFrequency(60);
  attachInterrupt(digitalPinToInterrupt(imager_int_pin), imager_isr, FALLING);
  imager.startRanging();

  // Set sampling frequency for presence sensor
  presence.setTmosODR(STHS34PF80_TMOS_ODR_AT_15Hz);
}

void loop()
{

  // Read data from ToF imager when it is ready
  if (imagerReady)
  {
    updateImager();
  }

  // Read data from presence sensor when it is ready
  sths34pf80_tmos_drdy_status_t presenceReady;
  presence.getDataReady(&presenceReady);
  if (presenceReady.drdy)
  {
    updatePresence();
  }

  // Update room occupancy
  // updated here; 
  if (new_asym && human_presence())
  {
    updatePeople();
  }

  sendPeople();
}

// Called when ToF sensor triggers an interrupt saying a new frame is ready
void imager_isr()
{
  imagerReady = true;
}

// Processes data from the imager to determine if there is nothing in frame,
// something on the left of the frame, something on the right of the frame,
// or something in the middle of the frame
void updateImager()
{
  imagerReady = false;

  // read in raw data from ToF sensor
  VL53L5CX_ResultsData imagerData;
  imager.getRangingData(&imagerData);

  // Calculate distance and asymmetry using an average over space and a 1st order
  // low pass filter in time.
  float temp_distance = 0;
  float temp_asym = 0;
  for (size_t i = 0; i < 16; ++i)
  {
    temp_asym += constrain(imagerData.distance_mm[i], 0, 1200) * asym_weights[i];
    temp_distance += imagerData.distance_mm[i] * distance_weights[i];
  }
  asym = 0.8 * asym + 0.2 * temp_asym;
  distance = 0.5 * distance + 0.5 * temp_distance;

  // Set asymmetry flag
  // -1 means something on the far side of the frame
  // 0 means something in the middle of the frame
  // 1 means something on the near side of the frame
  // 2 means nothing in the frame
  int8_t asym_flag;
  if (distance > 2000)
  {
    asym_flag = 2;
  }
  else if (asym > asym_thresh)
  {
    asym_flag = 1;
  }
  else if (asym < -asym_thresh)
  {
    asym_flag = -1;
  }
  else
  {
    asym_flag = 0;
  }

  // Only add a new flag to the buffer if it is different from the previous flag.
  // Removes time dependence and makes it work for any walking speed (hopefully)
  if (asym_flag != asym_buf.last())
  {
    asym_buf.push(asym_flag);
    new_asym = true;
  }
}

// Process data from the presence sensor to determine if there are 0, 1, or 2 people
// in the doorway
void updatePresence()
{
  // read in raw data from presence sensor
  sths34pf80_tmos_func_status_t status;
  presence.getStatus(&status);

  if (status.pres_flag)
  {
    presence.getPresenceValue(&pres);
  }
  else
  {
    pres = 0;
  }

  // Determine if the presence passes the threshold for there currently
  // being 2, 1, or 0 people
  if (pres > presence_thresh_2)
  {
    pres_buf.push(2);
  }
  else if (pres > presence_thresh_1)
  {
    pres_buf.push(1);
  }
  else
  {
    pres_buf.push(0);
  }

  // Take the maximum reading from the last second. Supposed to help with the ToF and
  // presence sensors being slightly out of sync.
  recent_pres = 0;
  for (size_t i = 0; i < pres_buf.size() && recent_pres < 2; ++i)
  {
    if (pres_buf[i] == 2)
    {
      recent_pres = 2;
    }
  }
  for (size_t i = 0; i < pres_buf.size() && recent_pres < 1; ++i)
  {
    if (pres_buf[i] == 1)
    {
      recent_pres = 1;
    }
  }
}

// Update the counter for the number of people who have passed through the doorway
void updatePeople()
{
  new_asym = false;

  // Check if the asymmetry buffer matches any of the patterns for someone entering
  // or exiting. If it does, then increment by the counter by the maximum number of
  // people detected within the past second.
  if (asym_buf[0] == 1 && asym_buf[1] == 0 && asym_buf[2] == -1)
  {
    n_entered += recent_pres;
  }
  if (asym_buf[0] == -1 && asym_buf[1] == 0 && asym_buf[2] == 1)
  {
    n_exited += recent_pres;
  }
  if (asym_buf[1] == 1 && asym_buf[2] == -1)
  {
    n_entered += recent_pres;
  }
  if (asym_buf[1] == -1 && asym_buf[2] == 1)
  {
    n_exited += recent_pres;
  }
  people = n_entered - n_exited;
}

// Update the people counter shared with the server over a wireless connection
// x is unsigned, so if people is negative x will underflow to 2^32.
void sendPeople()
{
  LOCK_SHARED_VARIABLE(x);
  x.value = people;
  UNLOCK_SHARED_VARIABLE(x);
}

bool human_presence() {
  int leftSensorValue = analogRead(leftsnsr);
  int rightSensorValue = analogRead(rightsnsr);
  unsigned long currentTime = millis(); 

  Serial.print("[");
  Serial.print(leftSensorValue);
  Serial.print(",");
  Serial.print(rightSensorValue);
  Serial.println("]");
  delay(100);

  if(leftSensorValue > 1370 && rightonly) {
    if (currentTime - lastDetectionTime > cooldownPeriod) { // if the cooldown period has passed
      //printone();
      Serial.print("ENTER: ");
      Serial.println(++count);
      lastDetectionTime = currentTime; // update the time of the last detection
      rightonly = false;
      return true;
    }
  }

  else if(rightSensorValue > 2350 && leftonly) {
    if (currentTime - lastDetectionTime > cooldownPeriod) { // if the cooldown period has passed
      //printone();
      Serial.print("EXIT: ");
      if (count > 0){--count;}
      Serial.println(count);
      lastDetectionTime = currentTime; // update the time of the last detection
      leftonly = false;
      return true;
    }
  }


  else if(leftSensorValue > 1335 && !leftonly) {
    Serial.print("left only:  ");
    Serial.println(leftSensorValue);
    leftonly = true;
    leftActivationTime = currentTime;
    return true;
    print();
  }

  else if(rightSensorValue > 2580 && !rightonly) {
    rightonly = true;
    rightActivationTime = currentTime;
    return true;
    print();
  }

  if ((currentTime - leftActivationTime > 1700) && leftonly) { // if the left sensor has been on for more than 1 second
    leftonly = false;
    return true;
    print();
  }

  if ((currentTime - rightActivationTime > 1500) && rightonly) { // if the right sensor has been on for more than 1 second
      //this value was 1500
    rightonly = false;
    return true;
    print();
  }
return false;
}

