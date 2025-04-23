#include <FastLED.h>

#define LED_PIN        6
#define NUM_LEDS       300       // UNO SRAM limit—adjust as needed
#define LED_TYPE       WS2812B
#define COLOR_ORDER    GRB
#define DOOR_PIN       5

#define GPIO_Read(pin) digitalRead(pin)

CRGB leds[NUM_LEDS];
uint8_t hue = 0;

// Timing
const unsigned long DOOR_CHECK_INTERVAL = 20; // 20 ms
unsigned long lastDoorCheck = 0;
int doorState = LOW;

void setup() {
  pinMode(DOOR_PIN, INPUT);
  Serial.begin(9600);
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
}

void loop() {
  unsigned long now = millis();

  // 1) Check door every 20 ms
  if (now - lastDoorCheck >= DOOR_CHECK_INTERVAL) {
    lastDoorCheck = now;
    doorState = GPIO_Read(DOOR_PIN);
    Serial.print("Door State: ");
    Serial.println(doorState == HIGH ? 1 : 0);
  }

  // 2) Update LEDs based on last read doorState
  if (doorState == HIGH) {
    // Door open → solid warm white
    fill_solid(leds, NUM_LEDS, CRGB(255, 100, 30));
    FastLED.show();
  } else {
    // Door closed → smooth rainbow
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(hue + i / 3, 255, 255);
    }
    FastLED.show();
    hue++;  // advance the palette
  }

  // 3) Keep gradient smooth (~30 FPS)
  FastLED.delay(33);
}
