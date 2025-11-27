#include <Arduino_BMI270_BMM150.h>
#include <math.h>

// Activate buzzer for period (ms), checks against previous buzzer time (ms), at specific pitch (Hz)
bool buzzer(int period, unsigned long &previousBuzzerTime, int pitch);

// Return current pitch using IMU, with smoothing.
float getPitch(float oldPitch, bool initial);

// Updates buzzer state (run once per loop)
void updateBuzzer();

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  float ratio = (x - inMin) / (inMax - inMin);
  return outMin + ratio * (outMax - outMin);
}


bool buzzerMode = false;

unsigned long buzzerTimer = 0;
bool buzzerOn = false;


const int buzzerPin = 2;
const int potPin = A0;

const float firstThreshold = 5;
float secondThreshold = 10; // Adjusted by pot

const float alpha = 0.98; // Filter coefficient for pitch calc
float dt = 0.01; // Time step for pitch 100Hz

float x, y, z;
float gx, gy, gz; // Raw gyroscope data (deg/s)
long lastPitchTime;

float pitch = 0;
float initialPitch = 0;
float offsetPitch = 0;

int pointsTotal = 3;
int pointsAdded = 0;


void setup() {
  Serial.begin(9600);
  Serial.println("BEGIN");

  if (!IMU.begin()) {
    Serial.println("failed to init IMU");
    while (1);
  }

  pinMode(buzzerPin, OUTPUT);

  // Set the initial pitch on startup. Calculate offset from this value.
  while (!IMU.accelerationAvailable()) {
    Serial.println("waiting...");
  }
  IMU.readAcceleration(x, y, z);
  initialPitch = getPitch(0, true);
  pitch = initialPitch;
  lastPitchTime = micros();
}

void loop() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z); // g
    IMU.readGyroscope(gx, gy, gz); // deg/s
  } 

  float newPitch = getPitch(pitch, false);
  offsetPitch = newPitch - initialPitch;
  pitch = newPitch;

  pointsAdded++;
  if (pointsAdded >= pointsTotal) {
    pointsAdded = 0;
  }

  // Dynamically set value of second threhold. 
  int potVal = analogRead(potPin);
  secondThreshold = mapFloat(potVal, 0, 1023, 10, 50);

  if (offsetPitch >= secondThreshold) {
    buzzerMode = false;
  }
  else if (offsetPitch >= firstThreshold) {
    buzzerMode = true;
  }
  else {
    buzzerMode = false;
  }

  
  Serial.print(secondThreshold);
  Serial.print("  --  ");
  Serial.println(offsetPitch);
  
  updateBuzzer();
}

float getPitch(float oldPitch, bool initial) {
  
  // float mag = sqrt(x * x + y * y + z * z);
  // if (mag == 0) mag = 1;
  // float nP = acos(z / mag) * 180 / PI;
  // if (!initial) {
  //   nP = 0.9 * oldPitch + 0.1 * nP;
  // }
  // return nP;

  if (initial) {
    return atan2(x, z) * (180 / PI);
  }

  dt = (micros() - lastPitchTime) / 1000000.0;
  lastPitchTime = micros();

  float accAngle = atan2(x, z) * (180 / PI);
  float gyroComp = pitch + gy * dt;

  pitch = alpha * gyroComp + (1 - alpha) * accAngle;
  return pitch;
}

void updateBuzzer() {
  unsigned long now = millis();


  if (buzzerMode == false) {
    noTone(buzzerPin);
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);
    buzzerOn = false;
    return;
  }


  // Adaptive Beep Period
  float angle = offsetPitch;

  if (angle < firstThreshold) {
    angle = firstThreshold;
  }

  if (angle > secondThreshold) {
    angle = secondThreshold;
  }

  int minPeriod = 200; 
  int maxPeriod = 1000;

  int cyclePeriod = mapFloat(angle, firstThreshold, secondThreshold, maxPeriod, minPeriod);
  int pitch = 2300;

  int halfPeriod = cyclePeriod / 2;
  if (halfPeriod < 100) halfPeriod = 100;



  if ((now - buzzerTimer) >= halfPeriod) {
    buzzerTimer = now;
    buzzerOn = !buzzerOn;

    if (buzzerOn) {
      tone(buzzerPin, pitch);
    }
    else {
      noTone(buzzerPin);
      pinMode(buzzerPin, OUTPUT);
      digitalWrite(buzzerPin, LOW);
    }
  }
}