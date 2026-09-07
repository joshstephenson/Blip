/*
 * blip_mood_classifier.ino
 *
 * Builds on blip_wake_on_sound.ino, adding:
 *   - A rolling ring buffer of recent sound events (survives deep sleep)
 *   - Simple per-category confidence scoring from audio features
 *   - Clear-cut vs. ambiguous decision via confidence margin
 *   - A mood layer that decays over time and biases ambiguous calls
 *
 * All the scoring weights and thresholds below are starting points --
 * expect to tune them against real recordings of blip's environment.
 */

#include <Arduino.h>
#include "esp_sleep.h"
#include <time.h>

// ---- Pin configuration (same as before) ----
#define WAKE_PIN        GPIO_NUM_33
#define MIC_POWER_PIN   25
#define MIC_ADC_PIN     34

// ---- Sampling ----
#define SAMPLE_WINDOW_MS      300
#define SAMPLE_INTERVAL_US    1000

// ---- Categories ----
enum Category {
  CAT_STARTLED = 0,
  CAT_EXCITED,
  CAT_ANNOYED,
  CAT_CURIOUS,
  CAT_PLAYFUL,
  CAT_CONFUSED,   // fallback / low-confidence catch-all
  CAT_COUNT
};

const char* categoryName(uint8_t c) {
  switch (c) {
    case CAT_STARTLED: return "startled";
    case CAT_EXCITED:  return "excited";
    case CAT_ANNOYED:  return "annoyed";
    case CAT_CURIOUS:  return "curious";
    case CAT_PLAYFUL:  return "playful";
    default:           return "confused";
  }
}

// ---- Rolling event history (survives deep sleep, not power loss) ----
#define HISTORY_SIZE 8

struct SoundEvent {
  time_t  timestamp;     // seconds, monotonic across deep sleep (not power-loss safe)
  uint8_t category;
  uint8_t confidence;     // 0-100, winning category's score
};

RTC_DATA_ATTR SoundEvent history[HISTORY_SIZE];
RTC_DATA_ATTR uint8_t historyHead = 0;   // next write index
RTC_DATA_ATTR uint8_t historyCount = 0;  // how many valid entries so far

void pushEvent(uint8_t category, uint8_t confidence) {
  history[historyHead].timestamp  = time(nullptr);
  history[historyHead].category   = category;
  history[historyHead].confidence = confidence;
  historyHead = (historyHead + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
}

// Rhythm score: how evenly spaced are the last few "loud" events?
// Low variance in inter-event gaps -> looks like clapping/calling -> higher score.
float rhythmScore() {
  if (historyCount < 3) return 0.0;

  // Walk the last few entries in chronological order and collect gaps.
  float gaps[HISTORY_SIZE];
  int gapCount = 0;
  int idx = (historyHead - historyCount + HISTORY_SIZE) % HISTORY_SIZE;
  time_t prev = history[idx].timestamp;

  for (int i = 1; i < historyCount; i++) {
    idx = (idx + 1) % HISTORY_SIZE;
    time_t cur = history[idx].timestamp;
    float gap = difftime(cur, prev);
    if (gap > 0 && gap < 3.0) {   // only count gaps that could plausibly be one "call"
      gaps[gapCount++] = gap;
    }
    prev = cur;
  }
  if (gapCount < 2) return 0.0;

  float mean = 0;
  for (int i = 0; i < gapCount; i++) mean += gaps[i];
  mean /= gapCount;

  float variance = 0;
  for (int i = 0; i < gapCount; i++) variance += (gaps[i] - mean) * (gaps[i] - mean);
  variance /= gapCount;

  // Low variance -> high score. Tune the divisor against real clap timing.
  float score = 100.0 - (variance * 40.0);
  return constrain(score, 0.0, 100.0);
}

// ---- Mood state (survives deep sleep, decays over time) ----
struct Mood {
  uint8_t category;
  float   strength;        // 0-100
  time_t  lastUpdate;
};

RTC_DATA_ATTR Mood currentMood = { CAT_CONFUSED, 0.0, 0 };

#define MOOD_DECAY_PER_MIN     4.0    // strength lost per minute of no reinforcement
#define MOOD_REINFORCE_AMOUNT  15.0   // strength gained when mood repeats
#define MOOD_BIAS_WEIGHT        0.6   // how much mood nudges ambiguous scores
#define MARGIN_THRESHOLD        20.0  // top1-top2 gap needed to call it "clear-cut"

void decayMood() {
  time_t now = time(nullptr);
  float elapsedMin = difftime(now, currentMood.lastUpdate) / 60.0;
  currentMood.strength -= MOOD_DECAY_PER_MIN * elapsedMin;
  if (currentMood.strength < 0) currentMood.strength = 0;
  currentMood.lastUpdate = now;
}

void updateMood(uint8_t winner, float winnerConfidence) {
  decayMood();
  if (winner == currentMood.category) {
    currentMood.strength = min(100.0f, currentMood.strength + MOOD_REINFORCE_AMOUNT);
  } else if (winnerConfidence > currentMood.strength) {
    // A stronger new signal displaces a fading old mood.
    currentMood.category = winner;
    currentMood.strength = winnerConfidence;
  }
  // else: keep the old (already-decayed) mood -- the new event wasn't strong enough to unseat it.
}

// ---- Audio feature extraction ----
struct AudioFeatures {
  int   peakToPeak;
  float attackMs;          // time from baseline to peak
  float durationAboveMs;   // time spent above a mid threshold
};

AudioFeatures sampleFeatures(unsigned long windowMs) {
  AudioFeatures f = { 0, 0, 0 };
  int minVal = 4095, maxVal = 0;
  unsigned long start = millis();
  unsigned long peakTime = 0;
  unsigned long aboveStart = 0;
  bool wasAbove = false;
  const int MID_THRESHOLD = 300; // tune against your noise floor

  while (millis() - start < windowMs) {
    int v = analogRead(MIC_ADC_PIN);
    if (v < minVal) minVal = v;
    if (v > maxVal) { maxVal = v; peakTime = millis() - start; }

    bool above = (v - minVal) > MID_THRESHOLD;
    if (above && !wasAbove) aboveStart = millis();
    if (!above && wasAbove) f.durationAboveMs += (millis() - aboveStart);
    wasAbove = above;

    delayMicroseconds(SAMPLE_INTERVAL_US);
  }
  if (wasAbove) f.durationAboveMs += (millis() - aboveStart);

  f.peakToPeak = maxVal - minVal;
  f.attackMs   = peakTime;  // rough proxy: how quickly the peak was reached
  return f;
}

// ---- Category scoring ----
// Each function returns 0-100. These are simple heuristics -- expect to
// reshape them once you have real recordings to test against.
float scoreStartled(AudioFeatures& f) {
  if (f.peakToPeak < 500) return 0;
  float attackScore = constrain(100 - f.attackMs, 0, 100); // faster attack -> higher
  return (f.peakToPeak / 4095.0 * 60) + (attackScore * 0.4);
}
float scoreExcited(AudioFeatures& f) {
  if (f.peakToPeak < 400) return 0;
  return constrain(f.peakToPeak / 4095.0 * 100, 0, 100);
}
float scoreAnnoyed(AudioFeatures& f) {
  return constrain(f.durationAboveMs / (SAMPLE_WINDOW_MS * 0.01), 0, 100);
}
float scoreCurious(AudioFeatures& f) {
  if (f.peakToPeak < 150 || f.peakToPeak > 450) return 0;
  return 60.0; // flat mid-score for "soft, unremarkable" sounds
}
float scorePlayful(AudioFeatures& f) {
  return rhythmScore();
}

void classify(AudioFeatures& f, uint8_t& winner, float& winnerScore) {
  float scores[CAT_COUNT];
  scores[CAT_STARTLED] = scoreStartled(f);
  scores[CAT_EXCITED]  = scoreExcited(f);
  scores[CAT_ANNOYED]  = scoreAnnoyed(f);
  scores[CAT_CURIOUS]  = scoreCurious(f);
  scores[CAT_PLAYFUL]  = scorePlayful(f);
  scores[CAT_CONFUSED] = 30.0; // baseline fallback score, always in the running

  // Find top two
  uint8_t top1 = 0, top2 = 0;
  for (uint8_t i = 1; i < CAT_COUNT; i++) {
    if (scores[i] > scores[top1]) { top2 = top1; top1 = i; }
    else if (scores[i] > scores[top2] || top2 == top1) top2 = i;
  }

  float margin = scores[top1] - scores[top2];

  if (margin >= MARGIN_THRESHOLD) {
    // Clear-cut: mood doesn't get a vote.
    winner = top1;
    winnerScore = scores[top1];
  } else {
    // Ambiguous: nudge toward the current mood, then re-decide.
    scores[currentMood.category] += currentMood.strength * MOOD_BIAS_WEIGHT;
    uint8_t biasedTop = 0;
    for (uint8_t i = 1; i < CAT_COUNT; i++) {
      if (scores[i] > scores[biasedTop]) biasedTop = i;
    }
    winner = biasedTop;
    winnerScore = scores[biasedTop];
  }
}

// ---- Reaction + sleep cycle ----
void reactToSound(uint8_t category, float confidence) {
  Serial.printf("blip reacts: %s (confidence %.0f, mood was %s @ %.0f)\n",
                categoryName(category), confidence,
                categoryName(currentMood.category), currentMood.strength);
  // TODO: wire this into actual servo/sound/LED output.
}

void goToSleep() {
  Serial.flush();
  digitalWrite(MIC_POWER_PIN, LOW);
  esp_sleep_enable_ext0_wakeup(WAKE_PIN, 1);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(50);
  pinMode(MIC_POWER_PIN, OUTPUT);
  digitalWrite(MIC_POWER_PIN, LOW);

  esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();

  if (wakeReason == ESP_SLEEP_WAKEUP_EXT0) {
    digitalWrite(MIC_POWER_PIN, HIGH);
    delay(20);

    AudioFeatures f = sampleFeatures(SAMPLE_WINDOW_MS);

    uint8_t winner;
    float winnerScore;
    classify(f, winner, winnerScore);

    reactToSound(winner, winnerScore);
    updateMood(winner, winnerScore);
    pushEvent(winner, (uint8_t)winnerScore);

  } else {
    Serial.println("Fresh boot / power-on.");
    // On a real power-loss reboot, RTC memory (history + mood) is gone --
    // this naturally resets blip to a neutral, confused-by-default mood.
  }

  goToSleep();
}

void loop() {}
