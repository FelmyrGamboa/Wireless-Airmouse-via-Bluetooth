#include <BleMouse.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- PIN DEFINITIONS ---
#define LEFTBUTTON 18
#define RIGHTBUTTON 19
#define SCROLLUP_BUTTON 16
#define SCROLLDOWN_BUTTON 17

// --- CONFIGURATION SETTINGS ---
#define SENSITIVITY 18      
#define DEADZONE 0.05       
#define SLEEP_TIMEOUT 10000 // 10 seconds (in milliseconds)
#define SMOOTHING 0.9       
#define SCROLL_SPEED 1      

Adafruit_MPU6050 mpu;
BleMouse bleMouse("NAFR ClickAire", "Maker", 100);

// --- GLOBAL VARIABLES ---
unsigned long lastActivityTime = 0;
float filteredX = 0;
float filteredZ = 0;
bool isSleep = false;
bool bleConnectedLastFrame = false; // Prevents serial monitor flooding

// Function to put MPU6050 to sleep
void goToSleep() {
  if (!isSleep) {
    Serial.println(">> Entering Sleep Mode (MPU6050 Powered Down)");
    mpu.enableSleep(true);
    isSleep = true;
  }
}

// Function to wake MPU6050 up
void wakeUp() {
  if (isSleep) {
    Serial.println(">> Waking up!");
    mpu.enableSleep(false);
    delay(100); // Give MPU6050 time to stabilize its gyro
    isSleep = false;
    lastActivityTime = millis();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFTBUTTON, INPUT_PULLUP);
  pinMode(RIGHTBUTTON, INPUT_PULLUP);
  pinMode(SCROLLUP_BUTTON, INPUT_PULLUP);
  pinMode(SCROLLDOWN_BUTTON, INPUT_PULLUP);

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
  Serial.println("Air Mouse Ready and Pairing...");
}

void loop() {
  // 1. CHECK WAKE-UP TRIGGERS
  // If any button is pressed while asleep, wake up
  if (isSleep) {
    if (digitalRead(LEFTBUTTON) == LOW || digitalRead(RIGHTBUTTON) == LOW || 
        digitalRead(SCROLLUP_BUTTON) == LOW || digitalRead(SCROLLDOWN_BUTTON) == LOW) {
      wakeUp();
    } else {
      return; // Still asleep, skip the rest of the loop
    }
  }

  // 2. CONNECTION LOGIC & SERIAL PRINTING
  bool isCurrentlyConnected = bleMouse.isConnected();
  
  // Only print "Mouse Activated" once when connection status changes
  if (isCurrentlyConnected && !bleConnectedLastFrame) {
    Serial.println("BLE Connected: Mouse Activated");
    bleConnectedLastFrame = true;
    lastActivityTime = millis(); // Reset timer on connection
  } 
  else if (!isCurrentlyConnected && bleConnectedLastFrame) {
    Serial.println("BLE Disconnected");
    bleConnectedLastFrame = false;
  }

  if (isCurrentlyConnected) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    bool isMoving = false;

    // 3. MOUSE MOVEMENT (GYRO)
    filteredX = (g.gyro.x * (1.0 - SMOOTHING)) + (filteredX * SMOOTHING);
    filteredZ = (g.gyro.z * (1.0 - SMOOTHING)) + (filteredZ * SMOOTHING);

    if (abs(filteredX) > DEADZONE || abs(filteredZ) > DEADZONE) {
      int moveX = (int)(filteredZ * -SENSITIVITY);
      int moveZ = (int)(filteredX * -SENSITIVITY);
      
      bleMouse.move(moveX, moveZ);
      isMoving = true; 
    }

    // 4. BUTTONS
    if (digitalRead(LEFTBUTTON) == LOW) {
      if (!bleMouse.isPressed(MOUSE_LEFT)) {
        bleMouse.press(MOUSE_LEFT);
        Serial.println("Left Click");
      }
      isMoving = true; 
    } else if (bleMouse.isPressed(MOUSE_LEFT)) {
      bleMouse.release(MOUSE_LEFT);
    }

    if (digitalRead(RIGHTBUTTON) == LOW) {
      if (!bleMouse.isPressed(MOUSE_RIGHT)) {
        bleMouse.press(MOUSE_RIGHT);
        Serial.println("Right Click");
      }
      isMoving = true;
    } else if (bleMouse.isPressed(MOUSE_RIGHT)) {
      bleMouse.release(MOUSE_RIGHT);
    }

    // 5. SCROLLING
    if (digitalRead(SCROLLUP_BUTTON) == LOW) {
      bleMouse.move(0, 0, SCROLL_SPEED);
      isMoving = true;
      delay(50); // Small delay to control scroll speed
    }

    if (digitalRead(SCROLLDOWN_BUTTON) == LOW) {
      bleMouse.move(0, 0, -SCROLL_SPEED);
      isMoving = true;
      delay(50);
    }

    // 6. SLEEP TIMER MANAGEMENT
    if (isMoving) {
      lastActivityTime = millis(); // User is active, reset the clock
    } else {
      if (millis() - lastActivityTime > SLEEP_TIMEOUT) {
        goToSleep();
      }
    }

    delay(10); // Smooth loop timing
  } else {
    // If not connected, keep the timer reset so it doesn't sleep immediately after pairing
    lastActivityTime = millis();
  }
}