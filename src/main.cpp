#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define NUM_PIXELS  70           // The number of pixels

// Pin definitions
#define soilMoisturePin  A5  // Analog pin for soil moisture sensor
#define relayPin 2         // Digital pin for relay control
#define lightSensorPin  A4    // Analog pin for light sensor
#define ledPIN  3 // pin for LED control

Adafruit_NeoPixel pixels(NUM_PIXELS, ledPIN, NEO_RGB + NEO_KHZ800);

// Variable declarations
unsigned long lastSerialTime = 0;  // Timestamp for serial output
int motorState = 0;                 // Motor state (0: off, 1: on)
int soilMoistureReading;            // Soil moisture reading
int lightIntensityReading;          // Light intensity reading

void setup() {
  Serial.begin(9600);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);  // Initially turn off the motor
  pixels.begin(); // Initialize the NeoPixel strip.
  pixels.setBrightness(100);
  pixels.show();
}

void loop() {
  // Read sensor values
  soilMoistureReading = analogRead(soilMoisturePin);
  soilMoistureReading = map(soilMoistureReading, 0, 1023, 0, 100);
  lightIntensityReading = analogRead(lightSensorPin);

  // Control the motor based on soil moisture
  if (soilMoistureReading > 50) {  // Soil is dry
    digitalWrite(relayPin, LOW);  // Turn on the motor
    motorState = 1;
  } else {
    digitalWrite(relayPin, HIGH);  // Turn off the motor
    motorState = 0;
  }

  if (lightIntensityReading > 500)
  {
    for ( uint16_t i=0; i < NUM_PIXELS; i++ ) {
      // Set the color value of the i pixel.
      pixels.setPixelColor( i, 0x005998); // GRB
    }

  }else
  {
    for ( uint16_t i=0; i < NUM_PIXELS; i++ ) {
      // Set the color value of the i pixel.
      pixels.setPixelColor( i, 0x000000); // GRB
    }
  }
  

  // Print sensor readings and motor state to serial monitor
  if (millis() - lastSerialTime >= 10000) {
    Serial.println("=");
    Serial.println(soilMoistureReading);
    Serial.println(lightIntensityReading);
    Serial.println(motorState);
    lastSerialTime = millis();
  }
  pixels.show();
  delay(1000);  // Delay for 1 second
}