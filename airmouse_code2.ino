#include <BleMouse.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- PIN DEFINITIONS ---
#define LEFTBUTTON 19
#define RIGHTBUTTON 18

// --- CONFIGURATION SETTINGS ---
#define SENSITIVITY 17      
#define DEADZONE 0.05       // Increased slightly to ignore sensor noise/drift
#define SLEEP_TIMEOUT 10000 // 30 seconds (30000ms) - 1 second was too short!
#define SMOOTHING 0.9       // Adjusted for a better balance of lag vs jitter

Adafruit_MPU6050 mpu;
BleMouse bleMouse("ESP32 Air Mouse", "Maker", 100);

// --- GLOBAL VARIABLES ---
unsigned long lastActivityTime = 0;
float filteredX = 0;
float filteredZ = 0;
bool isSleep = false;

void goToSleep() {
  Serial.println("Inactivity detected. Going to Sleep...");
  delay(200);

  mpu.enableSleep(true);
  isSleep = true;
  
  // Wake up when LEFTBUTTON (GPIO 19) is pressed (goes LOW)
  // esp_sleep_enable_ext0_wakeup((gpio_num_t)LEFTBUTTON, 0); 
  
  // Power down the MPU6050 to save battery
  // if (digitalRead(LEFTBUTTON) == LOW) {
  //     mpu.enableSleep(false);
  // } else {
  //     mpu.enableSleep(true);
  // }

  // Enter Deep Sleep
  // esp_light_sleep_start();
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFTBUTTON, INPUT_PULLUP);
  pinMode(RIGHTBUTTON, INPUT_PULLUP);

  bleMouse.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  // Optimize MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 

  lastActivityTime = millis();
  Serial.println("Air Mouse Ready!");
}

void loop() {
  if (bleMouse.isConnected()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    bool isMoving = false;

    // 1. FILTERING
    filteredX = (g.gyro.x * (1.0 - SMOOTHING)) + (filteredX * SMOOTHING);
    filteredZ = (g.gyro.z * (1.0 - SMOOTHING)) + (filteredZ * SMOOTHING);

    // 2. MOVEMENT LOGIC
    if (abs(filteredX) > DEADZONE || abs(filteredZ) > DEADZONE) {
      int moveX = (int)(filteredZ * -SENSITIVITY);
      int moveY = (int)(filteredX * SENSITIVITY);
      
      bleMouse.move(moveX, moveY);
      isMoving = true; // Mark activity
    }

    // 3. BUTTON LOGIC (LEFT)
    if (digitalRead(LEFTBUTTON) == LOW) {
      if (!bleMouse.isPressed(MOUSE_LEFT)) {
        if (!isSleep) {
          bleMouse.press(MOUSE_LEFT);
          Serial.println("Left Click");
        } else {
          mpu.enableSleep(false);
          isSleep = false;
        }

      }
      isMoving = true; 
    } else if (bleMouse.isPressed(MOUSE_LEFT)) {
      bleMouse.release(MOUSE_LEFT);
    }

    // 4. BUTTON LOGIC (RIGHT)
    if (digitalRead(RIGHTBUTTON) == LOW) {
      if (!bleMouse.isPressed(MOUSE_RIGHT)) {
        bleMouse.press(MOUSE_RIGHT);
        Serial.println("Right Click");
      }
      isMoving = true;
    } else if (bleMouse.isPressed(MOUSE_RIGHT)) {
      bleMouse.release(MOUSE_RIGHT);
    }

    // 5. TIMER MANAGEMENT
    if (isMoving) {
      // If we are moving or clicking, keep resetting the timer
      lastActivityTime = millis();
    } else {
      // If NOT moving, check if we have exceeded the timeout
      unsigned long inactiveDuration = millis() - lastActivityTime;
      
      // Optional: Print countdown to serial every 5 seconds
      if (inactiveDuration % 5000 < 20) { 
        Serial.print("Inactive for: "); 
        Serial.print(inactiveDuration / 1000); 
        Serial.println("s"); 
      }

      if (inactiveDuration > SLEEP_TIMEOUT) {
        goToSleep();
      }
    }

    delay(10); 
  } else {
    // If BLE is disconnected, check for sleep after a while to save battery
    if (millis() - lastActivityTime > (SLEEP_TIMEOUT * 2)) {
      goToSleep();
    }
  }
}