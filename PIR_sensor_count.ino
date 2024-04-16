#define leftsnsr A0
#define rightsnsr A3
#define LED_PIN 2

volatile bool leftonly = false;
volatile bool rightonly = false;
volatile int count = 0;

unsigned long lastDetectionTime = 0; // time of the last detection
unsigned long cooldownPeriod = 2500; // cooldown period in milliseconds (5 seconds)
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
      
    }
  }


  else if(leftSensorValue > 1335 && !leftonly) {
    Serial.print("left only:  ");
    Serial.println(leftSensorValue);
    leftonly = true;
    leftActivationTime = currentTime;
    print();
  }

  else if(rightSensorValue > 2580 && !rightonly) {
    rightonly = true;
    rightActivationTime = currentTime;
    print();
  }

  if ((currentTime - leftActivationTime > 1700) && leftonly) { // if the left sensor has been on for more than 1 second
    leftonly = false;
    print();
  }

  if ((currentTime - rightActivationTime > 1500) && rightonly) { // if the right sensor has been on for more than 1 second
      //this value was 1500
    rightonly = false;
    print();
  }


  // if(leftonly && !rightonly){
  //   Serial.print("right coming? : ");
  //   Serial.println(rightSensorValue);
  // }
}

void print(){
  // Serial.print("[");
  // Serial.print(leftonly);
  // Serial.print(",");
  // Serial.print(rightonly);
  // Serial.println("]");

  // Serial.print("[");
  // Serial.print(leftSensorValue);
  // Serial.print(",");
  // Serial.print(rightSensorValue);
  // Serial.println("]");
}

      // Serial.print("human exited: ");
      // Serial.print(leftSensorValue);
      // Serial.print("  |  ");
      // Serial.print(rightSensorValue);
