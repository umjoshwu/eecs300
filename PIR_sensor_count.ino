#define leftsnsr A0
#define rightsnsr A3
#define LED_PIN 2

volatile bool leftonly = false;
volatile bool rightonly = false;
volatile int count = 0;

unsigned long lastDetectionTime = 0; // time of the last detection
unsigned long cooldownPeriod = 5000; // cooldown period in milliseconds (5 seconds)
unsigned long leftActivationTime = 0; // time when the left sensor was activated
unsigned long rightActivationTime = 0; // time when the right sensor was activated

void setup() {  
  pinMode(leftsnsr, INPUT); 
  pinMode(rightsnsr, INPUT); 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // switch off the Arduino's in-built LED 
  Serial.begin(9600); // start serial connection to print out messages to the serial monitor
}

void loop() {
  int leftSensorValue = analogRead(leftsnsr);
  int rightSensorValue = analogRead(rightsnsr);

  unsigned long currentTime = millis(); // get the current time
      // Serial.print(leftSensorValue);
      // Serial.print("  |  ");
      // Serial.println(rightSensorValue);

  if(leftSensorValue > 1400 && rightonly) {
    if (currentTime - lastDetectionTime > cooldownPeriod) { // if the cooldown period has passed
      // Serial.print("human entered: ");
      // Serial.print(leftSensorValue);
      // Serial.print("  |  ");
      // Serial.print(rightSensorValue);
      Serial.print("ENTER: ");
      Serial.println(++count);
      lastDetectionTime = currentTime; // update the time of the last detection
      rightonly = false;
      Serial.println("Right sensor deactivated");
    }
  }

  else if(rightSensorValue > 2500 && leftonly) {
    if (currentTime - lastDetectionTime > cooldownPeriod) { // if the cooldown period has passed
      // Serial.print("human exited: ");
      // Serial.print(leftSensorValue);
      // Serial.print("  |  ");
      // Serial.print(rightSensorValue);
      Serial.print("EXIT: ");
      if (count > 0){
--count;
      }
      Serial.println(count);
      lastDetectionTime = currentTime; // update the time of the last detection
      leftonly = false;
      Serial.println("Left sensor deactivated");
    }
  }


  else if(leftSensorValue > 1400 && !leftonly) {
    leftonly = true;
    leftActivationTime = currentTime;
    Serial.println("Left sensor activated only");
  }

  else if(rightSensorValue > 2500 && !rightonly) {
    rightonly = true;
    rightActivationTime = currentTime;
    Serial.println("Right sensor activated only");
  }

  if ((currentTime - leftActivationTime > 1500) && leftonly) { // if the left sensor has been on for more than 1 second
    leftonly = false;
    Serial.println("Left sensor deactivated");
  }

  if ((currentTime - rightActivationTime > 1500) && rightonly) { // if the right sensor has been on for more than 1 second
    rightonly = false;
    Serial.println("Right sensor deactivated");
  }
}
