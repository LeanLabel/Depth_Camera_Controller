/*
 * ESP32 Bluetooth Gamepad
 * 
 * This sketch creates a BLE gamepad with:
 * - 2 analog sticks always reporting 0 (fully left/up)
 * - 8 buttons total: 3 functional + 5 dummy (always released)
 * 
 * The dummy axes and buttons ensure Windows/Unity sees a complete gamepad.
 */

#include <BleGamepad.h>

// Pin definitions for your functional buttons
// Adjust these to match your actual wiring
#define BUTTON_A_PIN 4
#define BUTTON_B_PIN 5
#define BUTTON_C_PIN 6

// Create a configuration object
BleGamepad bleGamepad("ESP32 Gamepad", "Espressif", 100);

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE Gamepad...");
    
    BleGamepadConfiguration config;
    config.setButtonCount(8);          // 8 total buttons
    config.setAutoReport(true);
    
    // Set axis range (0-32767)
    config.setAxesMin(0);
    config.setAxesMax(32767);
    
    bleGamepad.begin(&config);
    
    // Configure button pins as inputs
    pinMode(BUTTON_A_PIN, INPUT);
    pinMode(BUTTON_B_PIN, INPUT);
    pinMode(BUTTON_C_PIN, INPUT);
    
    // --- Set all axes to 0 (dummy sticks) ---
    bleGamepad.setAxes(0, 0, 0, 0, 0, 0, 0, 0);

    // --- Release all dummy buttons (4-8) once ---
    bleGamepad.release(BUTTON_4);
    bleGamepad.release(BUTTON_5);
    bleGamepad.release(BUTTON_6);
    bleGamepad.release(BUTTON_7);
    bleGamepad.release(BUTTON_8);

    Serial.println("Waiting for connection...");
}

void loop() {
    if (bleGamepad.isConnected()) {        

        if (digitalRead(BUTTON_A_PIN) == HIGH)
          bleGamepad.press(BUTTON_1);
        
        else 
          bleGamepad.release(BUTTON_1);
        
        
        if (digitalRead(BUTTON_B_PIN) == HIGH) 
          bleGamepad.press(BUTTON_2);
        else 
          bleGamepad.release(BUTTON_2);
        
        
        if (digitalRead(BUTTON_C_PIN) == HIGH) 
          bleGamepad.press(BUTTON_3);
        else 
          bleGamepad.release(BUTTON_3);
        
        
        delay(10);  // small delay to avoid flooding
    } else {
        delay(1000);
    }
}