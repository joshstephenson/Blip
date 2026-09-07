#include "Sound.h"
#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

#define I2S_PORT I2S_NUM_0

#define I2S_DIN  26
#define I2S_BCLK 27
#define I2S_LRC  14

#define SAMPLE_RATE 44100
#define TWO_PI 6.28318530718f

// Change this to 1 if you want the audio task on core 1 instead.
#define AUDIO_CORE 1

namespace Sound {

  static TaskHandle_t audioTaskHandle = nullptr;

  static float phase = 0.0f;
  static float globalVolume = 0.15f;

  // Single-slot sound request state.
  // This replaces the queue. It stores only the latest requested sound.
  static portMUX_TYPE soundMux = portMUX_INITIALIZER_UNLOCKED;
  static bool hasPendingEvent = false;
  static Event pendingEvent = TICK;
  static bool heldMode = false;

  static void writeSampleBuffer(int16_t *buffer, int sampleCount);
  static void audioTask(void *param);
  static void playEvent(Event event);

  static bool takePendingEvent(Event &event);
  static bool isHeldModeEnabled();

  static void heldPurrStep();

  void begin() {
    i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCLK,
      .ws_io_num = I2S_LRC,
      .data_out_num = I2S_DIN,
      .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);

    BaseType_t ok = xTaskCreatePinnedToCore(
      audioTask,
      "audioTask",
      8192,
      nullptr,
      1,
      &audioTaskHandle,
      AUDIO_CORE
    );

    Serial.printf("audio task create = %d, handle = %p\n", ok, audioTaskHandle);

    // Seed random number generator
    srand(time(0));
  }

  void play(Event event) {
    portENTER_CRITICAL(&soundMux);
    pendingEvent = event;
    hasPendingEvent = true;
    portEXIT_CRITICAL(&soundMux);

    if (audioTaskHandle) {
      xTaskNotifyGive(audioTaskHandle);
    }
  }

  void setHeldMode(bool enabled) {
    portENTER_CRITICAL(&soundMux);
    heldMode = enabled;
    portEXIT_CRITICAL(&soundMux);

    if (audioTaskHandle) {
      xTaskNotifyGive(audioTaskHandle);
    }
  }

  bool getHeldMode() {
    return isHeldModeEnabled();
  }

  void clearPending() {
    portENTER_CRITICAL(&soundMux);
    hasPendingEvent = false;
    portEXIT_CRITICAL(&soundMux);
  }

  static bool takePendingEvent(Event &event) {
    bool found = false;

    portENTER_CRITICAL(&soundMux);

    if (hasPendingEvent) {
      event = pendingEvent;
      hasPendingEvent = false;
      found = true;
    }

    portEXIT_CRITICAL(&soundMux);

    return found;
  }

  static bool isHeldModeEnabled() {
    bool enabled;

    portENTER_CRITICAL(&soundMux);
    enabled = heldMode;
    portEXIT_CRITICAL(&soundMux);

    return enabled;
  }

  static void audioTask(void *param) {
    Event event;

    while (true) {
      if (takePendingEvent(event)) {
        playEvent(event);
        continue;
      }

      if (isHeldModeEnabled()) {
        heldPurrStep();
        continue;
      }

      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
    }
  }

  static void heldPurrStep() {
    const int bufferSize = 128;
    int16_t buffer[bufferSize * 2];

    const float baseFreq = 800.0f;
    const float vibratoDepth = 20.0f;
    const float vibratoRate = 40.0f;
    const float breathRate = 1.0f;

    static uint32_t sampleClock = 0;

    for (int i = 0; i < bufferSize; i++) {
      float t = (float)sampleClock / SAMPLE_RATE;

      float breath = 0.5f + 0.5f * sinf(TWO_PI * breathRate * t);
      float vibratoSignal = sinf(TWO_PI * vibratoRate * t);
      float freq = baseFreq + vibratoDepth * breath * vibratoSignal;

      phase += TWO_PI * freq / SAMPLE_RATE;
      if (phase > TWO_PI) phase -= TWO_PI;

      float sample =
        0.8f * sinf(phase) +
        0.2f * sinf(phase * 2.0f);

      sample *= breath * globalVolume;

      int16_t s = (int16_t)(sample * 32767.0f);

      buffer[i * 2] = s;
      buffer[i * 2 + 1] = s;

      sampleClock++;
    }

    writeSampleBuffer(buffer, bufferSize);
  }

  static void playEvent(Event event) {
    switch (event) {
      case TICK:
        tick();
        break;

      case TONE:
        tone(440.0f, 0.25f);
        break;

      case CLICK:
        click();
        break;

      case CURIOUS:
        curious();
        break;

      case ALERT:
        alert();
        break;

      case PURR:
        purr(800, 1.0);
        break;

      case SAD:
        sad();
        break;

      case CONFUSED:
        confused();
        break;

      case EXCITED:
        excited();
        break;

      default:
        break;
    }
  }

  static void writeSampleBuffer(int16_t *buffer, int sampleCount) {
  size_t bytesWritten = 0;

  esp_err_t err = i2s_write(
    I2S_PORT,
    buffer,
    sampleCount * 2 * sizeof(int16_t),
    &bytesWritten,
    pdMS_TO_TICKS(100)
  );

  if (err != ESP_OK) {
    Serial.printf("i2s_write failed: %d\n", err);
    i2s_zero_dma_buffer(I2S_PORT);
  }

  taskYIELD();
}

  void setVolume(float volume) {
    if (volume < 0.0f) volume = 0.0f;
    if (volume > 1.0f) volume = 1.0f;
    globalVolume = volume;
  }

  float getVolume() {
    return globalVolume;
  }

  void tone(float freq, float durationSec) {
    const int bufferSize = 256;
    int16_t buffer[bufferSize * 2];

    int totalSamples = durationSec * SAMPLE_RATE;
    int samplesWritten = 0;

    while (samplesWritten < totalSamples) {
      int samplesThisBuffer = min(bufferSize, totalSamples - samplesWritten);

      for (int i = 0; i < samplesThisBuffer; i++) {
        phase += TWO_PI * freq / SAMPLE_RATE;
        if (phase > TWO_PI) phase -= TWO_PI;

        float sample = sinf(phase) * globalVolume;
        int16_t s = (int16_t)(sample * 32767.0f);

        buffer[i * 2] = s;
        buffer[i * 2 + 1] = s;

        samplesWritten++;
      }

      writeSampleBuffer(buffer, samplesThisBuffer);
    }
  }

  void glide(float startFreq, float endFreq, float durationSec) {
    const int bufferSize = 256;
    int16_t buffer[bufferSize * 2];

    int totalSamples = durationSec * SAMPLE_RATE;
    int samplesWritten = 0;

    while (samplesWritten < totalSamples) {
      int samplesThisBuffer = min(bufferSize, totalSamples - samplesWritten);

      for (int i = 0; i < samplesThisBuffer; i++) {
        float progress = (float)samplesWritten / totalSamples;
        float freq = startFreq + (endFreq - startFreq) * progress;

        phase += TWO_PI * freq / SAMPLE_RATE;
        if (phase > TWO_PI) phase -= TWO_PI;

        float envelope = 1.0f;
        if (progress < 0.08f) envelope = progress / 0.08f;
        else if (progress > 0.85f) envelope = (1.0f - progress) / 0.15f;

        float sample = sinf(phase) * envelope * globalVolume;
        int16_t s = (int16_t)(sample * 32767.0f);

        buffer[i * 2] = s;
        buffer[i * 2 + 1] = s;

        samplesWritten++;
      }

      writeSampleBuffer(buffer, samplesThisBuffer);
    }
  }

  void vibrato(float baseFreq, float durationSec, float vibratoDepth, float vibratoRate) {
    const int bufferSize = 256;
    int16_t buffer[bufferSize * 2];

    int totalSamples = durationSec * SAMPLE_RATE;
    int samplesWritten = 0;

    while (samplesWritten < totalSamples) {
      int samplesThisBuffer = min(bufferSize, totalSamples - samplesWritten);

      for (int i = 0; i < samplesThisBuffer; i++) {
        float t = (float)samplesWritten / SAMPLE_RATE;
        float freq = baseFreq + vibratoDepth * sinf(TWO_PI * vibratoRate * t);

        phase += TWO_PI * freq / SAMPLE_RATE;
        if (phase > TWO_PI) phase -= TWO_PI;

        float progress = (float)samplesWritten / totalSamples;
        float envelope = 1.0f;
        if (progress < 0.08f) envelope = progress / 0.08f;
        else if (progress > 0.85f) envelope = (1.0f - progress) / 0.15f;

        float sample = sinf(phase) * envelope * globalVolume;
        int16_t s = (int16_t)(sample * 32767.0f);

        buffer[i * 2] = s;
        buffer[i * 2 + 1] = s;

        samplesWritten++;
      }

      writeSampleBuffer(buffer, samplesThisBuffer);
    }
  }

  void chirp(float startFreq, float endFreq, float durationSec, float vibratoDepth, float vibratoRate) {

    const int bufferSize = 256;
    int16_t buffer[bufferSize * 2];
    int totalSamples = durationSec * SAMPLE_RATE;
    int samplesWritten = 0;

    while (samplesWritten < totalSamples) {
        int samplesThisBuffer = min(bufferSize, totalSamples - samplesWritten);

        for (int i = 0; i < samplesThisBuffer; i++) {
            float t = (float)samplesWritten / SAMPLE_RATE;
            float progress = (float)samplesWritten / totalSamples;
            float baseFreq = startFreq + (endFreq - startFreq) * progress;
            float vibrato = sinf(TWO_PI * vibratoRate * t);
            float freq = baseFreq + vibratoDepth * vibrato;

            phase += TWO_PI * freq / SAMPLE_RATE;

            if (phase > TWO_PI) phase -= TWO_PI;
            float envelope = 1.0f;

            if (progress < 0.08f) {
                envelope = progress / 0.08f;
            } else if (progress > 0.85f) {
                envelope = (1.0f - progress) / 0.15f;
            }

            float sample = 0.85f * sinf(phase) + 0.15f * sinf(phase * 2.0f);

            sample *= envelope * globalVolume;
            int16_t s = (int16_t)(sample * 32767.0f);
            buffer[i * 2]     = s;
            buffer[i * 2 + 1] = s;
            samplesWritten++;
        }
        writeSampleBuffer(buffer, samplesThisBuffer);
    }
  }

  void purr(float baseFreq, float durationSec, float vibratoDepth, float vibratoRate, float breathRate) {
    const int bufferSize = 256;
    int16_t buffer[bufferSize * 2];

    int totalSamples = durationSec * SAMPLE_RATE;
    int samplesWritten = 0;

    while (samplesWritten < totalSamples) {
      int samplesThisBuffer = min(bufferSize, totalSamples - samplesWritten);

      for (int i = 0; i < samplesThisBuffer; i++) {
        float t = (float)samplesWritten / SAMPLE_RATE;

        float breath = 0.5f + 0.5f * sinf(TWO_PI * breathRate * t);
        float vibratoSignal = sinf(TWO_PI * vibratoRate * t);
        float freq = baseFreq + vibratoDepth * breath * vibratoSignal;

        phase += TWO_PI * freq / SAMPLE_RATE;
        if (phase > TWO_PI) phase -= TWO_PI;

        float sample =
          0.8f * sinf(phase) +
          0.2f * sinf(phase * 2.0f);

        sample *= breath * globalVolume;

        int16_t s = (int16_t)(sample * 32767.0f);

        buffer[i * 2] = s;
        buffer[i * 2 + 1] = s;

        samplesWritten++;
      }

      writeSampleBuffer(buffer, samplesThisBuffer);
    }
  }

  void click() {
    const int durationMs = 2;
    int sampleCount = SAMPLE_RATE * durationMs / 1000;

    int16_t buffer[256 * 2];
    int generated = 0;

    while (generated < sampleCount) {
      int chunk = min(256, sampleCount - generated);

      for (int i = 0; i < chunk; i++) {
        float sample = ((float)::random(-1000, 1000) / 1000.0f);
        float env = 1.0f - ((float)(generated + i) / sampleCount);

        sample *= env * globalVolume * 0.4;

        int16_t s = (int16_t)(sample * 32767.0f);

        buffer[i * 2] = s;
        buffer[i * 2 + 1] = s;
      }

      writeSampleBuffer(buffer, chunk);
      generated += chunk;
    }
  }

  void clicks(int count, float pauseSec) {
    for (int i = 0; i < count; i++) {
      click();
      delay((int)(pauseSec * 1000.0f));
    }
  }

  void curious() {
    //glide(1000, 1400, 0.04f);
    //delay(10);
    //vibrato(1400, 0.1, 40.0, 20.0);
    chirp();
  }

  // Playful cat "meow!" attention cue: quick rise-then-fall contour with a
  // vibrato waver (like a real meow), capped with a short bright "mew".
  void alert() {
    if (rand() % 2 == 0) {
      chirp(750.0f, 1350.0f, 0.09f, 40.0f, 30.0f);   // "me-"
      chirp(1350.0f, 900.0f, 0.16f, 35.0f, 25.0f);   // "-ow"
      delay(60);
      chirp(1100.0f, 1600.0f, 0.08f, 55.0f, 45.0f);  // "mew!"
    }else{
      chirp(750.0f, 1350.0f, 0.09f, 40.0f, 30.0f);   // "me-"
      delay(40);
      chirp(1100.0f, 1600.0f, 0.08f, 55.0f, 45.0f);  // "mew!"
    }
  }

  // Descending whimper: same drooping shape as before, but a slow vibrato
  // wobble (via chirp() instead of a clean glide()) makes it read as an
  // animal's sigh rather than a linear pitch sweep, which sounded more
  // like a mechanical power-down.
  void sad() {
    int result = rand() % 3;
    if (result == 0) {
      chirp(1000.0f, 620.0f, 0.35f, 15.0f, 6.0f);
    }else if (result == 1) {
      chirp(800.0f, 680.0f, 0.55f, 20.0f, 4.0f);
    }
  }

  // Rapid rising trio of chirps, each higher and more energetic than the
  // last, like a cat bouncing with excitement.
  void excited() {
    chirp(900.0f, 1500.0f, 0.06f, 50.0f, 60.0f);
    delay(30);
    chirp(1000.0f, 1700.0f, 0.06f, 60.0f, 70.0f);
    delay(30);
    chirp(1200.0f, 1900.0f, 0.07f, 70.0f, 80.0f);
  }

  // Wavering up-down chirp, like a questioning "huh?"
  void confused() {
    chirp(700.0f, 1050.0f, 0.15f, 30.0f, 25.0f);
    delay(40);
    chirp(1000.0f, 650.0f, 0.2f, 30.0f, 25.0f);
  }

  void squeak() {
    //chirp(800, 1200);
    glide(1800.0f, 2400.0f, 0.05f);
    delay(50);
    glide(2300.0f, 2800.0f, 0.05f);
    //tone(2200.f, 0.035f);
  }

  void tick() {
    float vol = getVolume();
    setVolume(0.1);
    glide(800, 1000, 0.05f);
    setVolume(vol);
  }

  void random() {
    float randStart = (float)(800 + (::random(0, 400)));
    float randEnd = (float)(randStart + (::random(0, 200)));
    glide(randStart, randEnd, 0.05f);
  }
}
