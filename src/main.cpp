#include <Arduino_BMI270_BMM150.h>
#include <math.h>

// Activate buzzer for period (ms), checks against previous buzzer time (ms), at specific pitch (Hz)
bool buzzer(int period, unsigned long &previousBuzzerTime, int pitch);

// Return current pitch using IMU, with smoothing.
float getPitch(float oldPitch, bool initial);

// Updates buzzer state (run once per loop)
void updateBuzzer();

float mapFloat(float input, float inMin, float inMax, float outMin, float outMax) {
  // Normalise input
  float k = 6; // Sharpness factor. Higher value - sharper increase earlier in graph. 
  float x = (input - inMin) / (inMax - inMin);
  if (x < 0) x = 0;
  if (x > 1) x = 1;

  float y = log(1 + k * x) / log(1 + k);

  return outMin + y * (outMax - outMin);
}

void getIMUData(float &ax, float &ay, float &az, float &mx, float &my, float &mz, float &gx, float &gy, float &gz) {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);
  }
}


bool buzzerMode = false;

unsigned long buzzerTimer = 0;
bool buzzerOn = false;


const int buzzerPin = 3;
const int potPin = A0;

const float firstThreshold = 2;
float secondThreshold; // Adjusted by pot

const float alpha = 0.98; // Filter coefficient for pitch calc
float dt = 0.01; // Time step for pitch 100Hz

float ax, ay, az, gx, gy, gz, mx, my, mz;
long lastPitchTime;

bool gotInitialPitch = false;
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
}

void loop() {
  getIMUData(ax, ay, az, mx, my, mz, gx, gy, gz);

  float newPitch = getPitch(pitch, false);
  offsetPitch = newPitch - initialPitch;
  pitch = newPitch;

  if (!gotInitialPitch) {
    initialPitch = pitch;
    gotInitialPitch = true;
  }

  offsetPitch = pitch - initialPitch;
  Serial.print(buzzerMode);
  Serial.print("\tinit:\t"); Serial.print(initialPitch);
  Serial.print("\tPitch:\t"); Serial.println(offsetPitch);  

  // Dynamically set value of second threhold. 
  int potVal = analogRead(potPin);
  secondThreshold = mapFloat(potVal, 0, 1023, 5, 20);

  if (offsetPitch >= secondThreshold) {
    buzzerMode = false;
  }
  else if (offsetPitch >= firstThreshold) {
    buzzerMode = true;
  }
  else {
    buzzerMode = false;
  }
  updateBuzzer();
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

  int minPeriod = 50; 
  int maxPeriod = 1000;

  int cyclePeriod = mapFloat(angle, firstThreshold, secondThreshold, maxPeriod, minPeriod);

  int halfPeriod = cyclePeriod / 2;
  if (halfPeriod < 100) halfPeriod = 100;



  if ((now - buzzerTimer) >= halfPeriod) {
    buzzerTimer = now;
    buzzerOn = !buzzerOn;

    if (buzzerOn) {
      digitalWrite(buzzerPin, HIGH);
    }
    else {      
      digitalWrite(buzzerPin, LOW);
    }
  }
}

float getPitch(float oldPitch, bool initial) {

  if (initial) {
    return atan2(ax, az) * (180 / PI);
  }

  dt = (micros() - lastPitchTime) / 1000000.0;
  lastPitchTime = micros();

  float accAngle = atan2(ax, az) * (180 / PI);
  float gyroComp = pitch + gy * dt;

  pitch = alpha * gyroComp + (1 - alpha) * accAngle;
  return pitch;
}