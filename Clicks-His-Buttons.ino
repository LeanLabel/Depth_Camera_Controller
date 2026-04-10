#include <Arduino.h>
#include <BleGamepad.h>

BleGamepad bleGamepad("Clicks-His-Buttons", "@joshiteoshi", 100);
BleGamepadConfiguration config;

#define MININT_16 0
#define MAXINT_16 32767

#define stickLX 36
#define stickLY 39
#define snip 1
#define push 2
#define lock 47

int stickMeanLX = 0;
int stickMeanLY = 32767;
const int HIDReportDelay = 50;
const int debounceDelay = 10;

int prevSnipState = HIGH;
int prevPushState = HIGH;
int prevLockState = HIGH;

void setup() {
  Serial.begin(115200);

  config.setAutoReport(false);
  config.setControllerType(CONTROLLER_TYPE_JOYSTICK);

  config.setButtonCount(8);
  config.setHatSwitchCount(0);
  config.setWhichAxes(true, true, false, true, true, false, false, false);
  
  pinMode(snip, INPUT_PULLUP);
  pinMode(push, INPUT_PULLUP);
  pinMode(lock, INPUT_PULLUP);
  bleGamepad.begin(&config);

  Serial.println("configured");
}

void loop() {
  if (bleGamepad.isConnected()) {

    // Handle button presses

    int currSnipState = digitalRead(snip);
    int currPushState = digitalRead(push);
    int currLockState = digitalRead(lock);

    if (currSnipState != prevSnipState) {
      if (currSnipState == HIGH)
        bleGamepad.press(BUTTON_1);
      else
        bleGamepad.release(BUTTON_1);
    }
    prevSnipState = currSnipState;

    if (currPushState != prevPushState) {
      if (currPushState == HIGH)
        bleGamepad.press(BUTTON_2);
      else
        bleGamepad.release(BUTTON_2);
    }
    prevPushState = currPushState;

    if (currLockState != prevLockState) {
      if (currLockState == HIGH)
        bleGamepad.press(BUTTON_3);
      else
        bleGamepad.release(BUTTON_3);
    }
    prevLockState = currLockState;

    // Dummy Joystick values

    if (stickMeanLX == 0) {
      stickMeanLX = 32767;
    } else {
      stickMeanLX = 0;
    }

    if (stickMeanLY == 0) {
      stickMeanLY = 32767;
    } else {
      stickMeanLY = 0;
    }

    bleGamepad.setAxes(stickMeanLX, stickMeanLY, 0, 0, 0, 0, DPAD_CENTERED);
    
    bleGamepad.sendReport();
    delay(HIDReportDelay);
  }
}