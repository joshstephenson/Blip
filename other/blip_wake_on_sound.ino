/*
 * blip_wake_on_sound.ino
 *
 * ESP32 deep-sleep + wake-on-sound skeleton for blip.
 *
 * HARDWARE:
 *  - Comparator-style sound sensor module (e.g. LM393-based "sound sensor"),
 *    digital output (DO) pin wired to WAKE_PIN. Adjust the module's onboard
 *    trimpot so DO idles LOW and pulses HIGH on a loud noise.
 *  - MAX9814 (or similar) analog mic module:
 *      VCC  -> MIC_POWER_PIN   (ESP32 GPIO powers it directly, ~2-4mA draw)
 *      GND  -> GND
 *      OUT  -> MIC_ADC_PIN     (analog input)
 *
 * BEHAVIOR:
 *  - ESP32 sits in deep sleep (~10-150uA) almost all the time.
 *  - The comparator module stays powered and watches the room continuously.
 *  - When it detects a loud enough sound, it pulls WAKE_PIN HIGH, which
 *    wakes the ESP32 via ext0.
 *  - On wake, ESP32 powers the real mic, samples it briefly, decides
 *    whether this was a "real" event worth reacting to, reacts, then
 *    goes back to sleep.
 *
 * NOTE ON PINS:
 *  ext0 wakeup requires an RTC-capable GPIO. Common choices on most ESP32
 *  dev boards: 32, 33, 34, 35, 36, 39 (34/35/36/39 are input-only, which is
 *  fine here since we only need to read HIGH/LOW from the comparator).
 *  Double-check which RTC GPIOs are broken out on your specific board.
 */

#include <Arduino.h>
#include "esp_sleep.h"

// ---- Pin configuration ----
#define WAKE_PIN        GPIO_NUM_33   // comparator DO -> here (must be RTC GPIO)
#define MIC_POWER_PIN   25            // powers the MAX9814 only when awake
#define MIC_ADC_PIN     34            // MAX9814 OUT (ADC1 channel, input-only pin)

// ---- Tuning ----
#define SAMPLE_WINDOW_MS      300     // how long to listen once awake
#define SAMPLE_INTERVAL_US    1000    // ~1kHz sampling
#define REACT_THRESHOLD        600    // peak-to-peak ADC counts to count as "real" sound
                                       // ESP32 ADC is 12-bit (0-4095) -- tune this by testing
#define AWAKE_TIMEOUT_MS      4000    // safety: max time to stay awake before forcing back to sleep

RTC_DATA_ATTR int bootCount = 0;      // survives deep sleep, resets on power-cycle

void reactToSound(int loudness) {
  // Placeholder -- wire this up to blip's actual response
  // (servo twitch, LED, sound clip, whatever blip does)
  Serial.print("blip reacts! loudness score: ");
  Serial.println(loudness);
}

int sampleMicPeakToPeak(unsigned long windowMs) {
  int minVal = 4095;
  int maxVal = 0;
  unsigned long start = millis();

  while (millis() - start < windowMs) {
    int v = analogRead(MIC_ADC_PIN);
    if (v < minVal) minVal = v;
    if (v > maxVal) maxVal = v;
    delayMicroseconds(SAMPLE_INTERVAL_US);
  }
  return maxVal - minVal;
}

void goToSleep() {
  Serial.println("Going back to sleep...");
  Serial.flush();

  // Make sure the mic is powered off before sleeping
  digitalWrite(MIC_POWER_PIN, LOW);

  // Re-arm ext0 wakeup on WAKE_PIN, trigger on HIGH
  esp_sleep_enable_ext0_wakeup(WAKE_PIN, 1);

  esp_deep_sleep_start();
  // execution never returns from here -- chip resets on wake
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(MIC_POWER_PIN, OUTPUT);
  digitalWrite(MIC_POWER_PIN, LOW);   // mic off by default

  bootCount++;

  esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();

  if (wakeReason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.printf("Woke up from sound (boot #%d)\n", bootCount);

    // Power up the real mic and give it a moment to settle
    digitalWrite(MIC_POWER_PIN, HIGH);
    delay(20);

    int loudness = sampleMicPeakToPeak(SAMPLE_WINDOW_MS);
    Serial.printf("Measured loudness: %d\n", loudness);

    if (loudness > REACT_THRESHOLD) {
      reactToSound(loudness);
      // Optional: stay awake a bit longer here if blip's reaction
      // (servo moves, sound playback, etc.) takes time.
    } else {
      Serial.println("Not loud enough -- false trigger, ignoring.");
    }

  } else {
    Serial.println("Fresh boot / power-on (not a sound wake).");
  }

  goToSleep();
}

void loop() {
  // Never reached -- everything happens in setup() before sleeping again.
}
