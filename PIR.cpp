#define leftsnsr 2
#define rightsnsr 3

volatile bool leftonly = false;
volatile bool rightonly = false;
volatile bool human = false;
volatile int count = 0;

unsigned long lastDetectionTime = 0; // time of the last detection
unsigned long cooldownPeriod = 2500; // cooldown period in milliseconds (5 seconds)
unsigned long leftActivationTime = 0; // time when the left sensor was activated
unsigned long rightActivationTime = 0; // time when the right sensor was activated

void setup() {  
  pinMode(leftsnsr, INPUT); 
  pinMode(rightsnsr, INPUT); 

  Serial.begin(9600); // start serial connection to print out messages to the serial monitor
}

void loop() {
  int leftSensorValue = analogRead(leftsnsr);
  int rightSensorValue = analogRead(rightsnsr);
  unsigned long currentTime = millis(); 
 

  if(leftSensorValue > 1200 ) {
    leftActivationTime = currentTime;
    human = true;

  }

  else if(rightSensorValue > 2200) {
    rightActivationTime = currentTime;
    human = true;
  }

  if ((currentTime - leftActivationTime > 1700)) { // if the left sensor has been on for more than 1 second
    human = false;
  }

  if (currentTime - rightActivationTime > 1500) { // if the right sensor has been on for more than 1 second
    human = false;

  }
}


