#include <Wire.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
#include <ESP32Encoder.h>
#include <esp_sleep.h>

// Set up our LED display
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();
char displayBuffer[4];
uint8_t dimensionLetter = 'C';

// SDA: GPIO 21
// SCL: GPIO 22

// Set up the rotary encoder using the ESP32Encoder library
ESP32Encoder encoder;
int16_t last, value;
#define encoderPinA          32  // 'CLK' // Select appropriate ESP32 GPIO pins
#define encoderPinB          33  // 'DT' 
#define encoderButtonPin     25  // 'SW'

// Steps per notch can be 1, 2, or 4, depending on the encoder
#define stepsPerNotch        1  // Adjusted for more sensitivity (1 click = 1 step)

// Variables for refined acceleration
unsigned long lastTurnTime = 0;
unsigned long accelerationThreshold = 300;  // Time threshold (ms) for detecting rapid turns
int incrementCounter = 0;  // Counter for the number of increments within the threshold
int accelerationStartCount = 4;  // Acceleration kicks in after at least 4 increments
int baseAccelerationMultiplier = 3;  // Base acceleration multiplier


// Set up the Green LEDs
#define topBulbPin           15
#define frontRightPin        26
#define frontCenterPin       27
#define frontLeftPin         14
#define maximumBright        255
#define mediumBright         127
int topBulbBrightness = 255;

bool justWokeUp = false;
bool buttonState = HIGH;  // Track the current button state
bool lastButtonState = HIGH;  // Track the previous button state
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // 50 ms debounce time
bool buttonHeldDuringWake = false; // Track if the button was held during wakeup

void setup() {
  // Initialize UART communication for debugging
  Serial.begin(115200); // Start serial communication at a baud rate of 115200
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  // Check if the ESP32 woke up from deep sleep
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    // ESP32 woke up from deep sleep via the external pin (button)
    justWokeUp = true;
  } else {
    justWokeUp = false;
  }

  // Debug message: Setup initialization
  Serial.println("Initializing system...");

  // Set up pin modes for the LEDs
  pinMode(topBulbPin, OUTPUT);
  pinMode(frontRightPin, OUTPUT);
  pinMode(frontLeftPin, OUTPUT);
  pinMode(frontCenterPin, OUTPUT);

  digitalWrite(frontRightPin, HIGH);
  digitalWrite(frontLeftPin, HIGH);
  digitalWrite(frontCenterPin, HIGH);
  digitalWrite(topBulbPin, HIGH);

  // Initialize the display
  alpha4.begin(0x70);  // Pass in the address for the LED display
  encoderSetup();
  
  // Debug message: Setup complete
  Serial.println("Setup complete.");
}

void loop() {
  // Handle wake-up
  if (justWokeUp) {
    digitalWrite(frontRightPin, HIGH);
    digitalWrite(frontLeftPin, HIGH);
    digitalWrite(frontCenterPin, HIGH);
    digitalWrite(topBulbPin, HIGH);
    Serial.println("Woke up, all LEDs turned on.");
    justWokeUp = false;  // Reset flag after waking up and processing
    lastButtonState = HIGH;  // Assume the button is released when waking up
    buttonHeldDuringWake = true;  // Indicate the button was pressed during wake-up
  }
  
  // Read the button state
  int reading = digitalRead(encoderButtonPin);
  
  // Debouncing logic
  if (reading != lastButtonState) {
    lastDebounceTime = millis();  // Reset the debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button was held during wake-up, ignore it until it's released
    if (buttonHeldDuringWake) {
      if (reading == HIGH) {
        buttonHeldDuringWake = false;  // Button was released, can act on it again
        Serial.println("Button released after wake-up, ready for next press.");
      }
    } else {
      // Only act on falling edge (when the button is pressed down) if not in wake-up state
      if (reading == LOW && buttonState == HIGH) {
        Serial.println("Encoder button pressed.");
        handleEncoderClick();
      }
    }

    // Update button state
    buttonState = reading;
  }

  // Update the lastButtonState
  lastButtonState = reading;

  // Update the encoder position and display
  updateDimension();
}

void encoderSetup() {
  // Initialize the rotary encoder
  
  // Set pin modes with internal pull-ups for the encoder pins
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  encoder.attachSingleEdge(encoderPinA, encoderPinB);
  encoder.clearCount();
  encoder.setCount(137);  // Initial dimension value
  last = -1;
  value = 137;

  pinMode(encoderButtonPin, INPUT_PULLUP);  // Use pullup for the button

  // Debug message: Encoder initialized
  Serial.println("Encoder initialized.");
}

void updateDimension() {
  // Get the encoder raw count
  int rawCount = encoder.getCount();
  // Divide by steps per notch to get a click value
  int diff = (rawCount - last) / stepsPerNotch;

  // If the encoder was turned
  if (diff != 0 || last == -1) {

    if(last != -1){
      unsigned long now = millis();

      // Check if the movement is within the accelerationThreshold
      if (now - lastTurnTime < accelerationThreshold) {
        incrementCounter++;  // Count the number of increments
      } else {
        // Reset the increment counter if the time between movements exceeds the threshold
        incrementCounter = 1;
      }

      lastTurnTime = now;  // Update the last turn time

      // Apply acceleration if at least 4 increments were made
      if (incrementCounter >= accelerationStartCount) {
        // Calculate the acceleration multiplier based on the number of increments
        int accelerationMultiplier = baseAccelerationMultiplier + (incrementCounter - accelerationStartCount) * 3;

        diff *= accelerationMultiplier;  // Apply acceleration
        Serial.print("Acceleration applied: ");
        Serial.println(accelerationMultiplier);
      }

      // Update the value based on the encoder movement
      value -= diff;  // Reversed direction

      if (value > 999) {
        value = 0;
        if (dimensionLetter == 'Z') {
          dimensionLetter = 'A';
        } else {
          dimensionLetter++;
        }
      } else if (value < 0) {
        value = 999;
        if (dimensionLetter == 'A') {
          dimensionLetter = 'Z';
        } else {
          dimensionLetter--;
        }
      }
    }

    last = rawCount;  // Update the last raw count

    // Debug message: Dimension value updated
    Serial.print("Dimension updated to: ");
    Serial.print(dimensionLetter);
    Serial.print(value);
    Serial.println();

    // Update the display
    sprintf(displayBuffer, "%03i", value);
    alpha4.clear();
    alpha4.writeDigitAscii(0, dimensionLetter);
    alpha4.writeDigitAscii(1, displayBuffer[0]);
    alpha4.writeDigitAscii(2, displayBuffer[1]);
    alpha4.writeDigitAscii(3, displayBuffer[2]);
    alpha4.writeDisplay();
  }
}

void handleEncoderClick() {
  // Handle what happens when the encoder button is pressed

  Serial.println("Handling encoder button click, going to sleep...");

  alpha4.clear();
  alpha4.writeDigitAscii(0, 'R');
  alpha4.writeDigitAscii(1, 'I');
  alpha4.writeDigitAscii(2, 'C');
  alpha4.writeDigitAscii(3, 'K');
  alpha4.writeDisplay();
  
  digitalWrite(frontRightPin, LOW);
  digitalWrite(frontLeftPin, LOW);
  digitalWrite(frontCenterPin, LOW);
  digitalWrite(topBulbPin, LOW);

  delay(2000);
  
  alpha4.clear();
  alpha4.writeDisplay();

  goToSleep();
}

void goToSleep() {
  // Debug message: Going to deep sleep
  Serial.println("Entering deep sleep mode...");

  justWokeUp = true;

  // Set up GPIO25 (encoder button pin) to wake up ESP32 from deep sleep
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 0);  // Wake up on low signal from button press

  esp_deep_sleep_start();
}

/*
============== Testing Methods ==================
=================================================
*/

void displayTest() {
  alpha4.writeDigitRaw(3, 0x0);
  alpha4.writeDigitRaw(0, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(0, 0x0);
  alpha4.writeDigitRaw(1, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(1, 0x0);
  alpha4.writeDigitRaw(2, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(2, 0x0);
  alpha4.writeDigitRaw(3, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);

  alpha4.clear();
  alpha4.writeDisplay();

  // Display every character
  for (uint8_t i = '!'; i <= 'z'; i++) {
    alpha4.writeDigitAscii(0, i);
    alpha4.writeDigitAscii(1, i + 1);
    alpha4.writeDigitAscii(2, i + 2);
    alpha4.writeDigitAscii(3, i + 3);
    alpha4.writeDisplay();
    delay(300);
  }
}
