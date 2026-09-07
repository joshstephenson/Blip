#include <ESP32Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <VL53L1X.h>
#include <driver/adc.h>
#include "Sound.h"

Servo leftEar;
Servo rightEar;

// Two 1.28" round GC9A01 displays, 240x240, driven over the same SPI bus
// -- one per eye. "SCL"/"SDA" on the panel silkscreen are SPI clock/MOSI,
// not I2C. Both panels share SCLK/MOSI/RST; CS and DC are wired separately
// per panel so each can be addressed independently.
const int TFT_SCLK = 18; // SCL
const int TFT_MOSI = 23; // SDA
const int TFT_DC   = 22;
const int LEFT_TFT_CS = 21;
const int LEFT_TFT_RST  = 19;
const int RIGHT_TFT_CS = 4;
const int RIGHT_TFT_RST  = 5;

Adafruit_GC9A01A leftEyeTft(&SPI, TFT_DC, LEFT_TFT_CS, LEFT_TFT_RST);
Adafruit_GC9A01A rightEyeTft(&SPI, TFT_DC, RIGHT_TFT_CS, RIGHT_TFT_RST);

// VL53L1X time-of-flight range sensor over I2C.
const int RANGE_SENSOR_SCL = 15;
const int RANGE_SENSOR_SDA = 13;

VL53L1X rangeSensor;

const unsigned long RANGE_CHECK_INTERVAL_MS = 1000;
unsigned long lastRangeCheckMs = 0;
const uint16_t RANGE_ALERT_THRESHOLD_MM = 500;

// MAX9814 electret mic amp -- analog envelope output on GPIO25, which is an
// ADC2 (not ADC1) pin. Read via the legacy adc2_get_raw() API rather than
// analogRead(): Sound.cpp already pulls in the legacy I2S driver (which on
// classic ESP32 pulls in the legacy ADC driver too), and mixing that with
// Arduino's analogRead() -- which uses the newer IDF5 adc_oneshot driver --
// aborts at boot with "ADC: CONFLICT! driver_ng is not allowed to be used
// with the legacy driver". Staying on the legacy API throughout avoids the
// conflict. ADC2 is shared with the WiFi radio, so this only works because
// WiFi.mode(WIFI_OFF) runs before the ADC is configured in setup().
const adc2_channel_t MIC_ADC_CHANNEL = ADC2_CHANNEL_8; // GPIO25

// The output is an audio waveform centered around mid-supply, not a DC level,
// so "how loud" is measured as peak-to-peak swing over a short sampling
// window rather than a single reading.
const unsigned long MIC_SAMPLE_WINDOW_MS = 50;
unsigned long micWindowStartMs = 0;
int micWindowMin = 4095;
int micWindowMax = 0;

// Peak-to-peak ADC counts (12-bit, 0-4095) that counts as "someone's talking".
// Depends on the MAX9814's GAIN pin wiring and mic placement -- measured
// ~800-900 for room noise and 1200+ when talking on GPIO25/ADC2.
const int MIC_CURIOUS_THRESHOLD = 2000;

// Require this many back-to-back over-threshold windows before triggering
// CURIOUS, to reject brief single-window spikes (e.g. electrical noise)
// rather than sustained sound. At MIC_SAMPLE_WINDOW_MS=50ms this is a
// ~150ms debounce -- short enough to not noticeably delay real detection.
const int MIC_CURIOUS_DEBOUNCE_WINDOWS = 3;
int micConsecutiveOverThreshold = 0;

// Once the mic has triggered CURIOUS, how long to wait with no further
// threshold crossing before easing back to ALERT on its own. Each new
// crossing pushes this back out, so it holds CURIOUS for as long as the
// sound keeps happening rather than a fixed duration from when it started.
const unsigned long CURIOUS_SILENCE_MS = 5000;
unsigned long micLastTriggerMs = 0;

const int LEFT_SERVO_PIN  = 33;
const int RIGHT_SERVO_PIN = 32;

// MG90S typical pulse range.
// Adjust if the servo doesn't reach full range cleanly.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Mechanical mounting offset.
// Left servo is mounted 25 degrees left of center.
// Right servo is mounted 25 degrees right of center.
const int EAR_OFFSET_DEG = -25;

const int EAR_UPRIGHT = 127;

// Final servo command clamps (after leftVisualToServo()/rightVisualToServo()
// convert a visual angle). Widened from the original 0-110/91-180 to give
// headroom for the +55deg total "more upright" shift
// applied to every mood except SAD (baseline) -- CURIOUS and EXCITED's 
// angles were landing outside the old range and clipping to the same 
// values, making them look identical.
const int LEFT_SERVO_COMMAND_MIN = 0;
const int LEFT_SERVO_COMMAND_MAX = 145;
const int RIGHT_SERVO_COMMAND_MIN = 35;
const int RIGHT_SERVO_COMMAND_MAX = 180;

// Optional trim values for fine-tuning after mounting.
// Example: if left is slightly crooked at upright, try +2 or -2.
int leftTrim  = 0;
int rightTrim = -4;

// The SAD purr starts after a beat and then loops seamlessly for a while,
// rather than firing as an instant one-shot like the other moods.
const unsigned long SAD_PURR_DELAY_MS    = 10;
const unsigned long SAD_PURR_DURATION_MS = 5000;

bool sadPurrDelayPending = false;
bool sadPurrHeldActive   = false;
unsigned long sadPurrTimerStart = 0;

enum Mood {
  ALERT,
  CURIOUS,
  CONFUSED,
  SAD,
  EXCITED
};

// Tracks whichever mood was most recently set, so the screen can be
// restored to the right face (relaxed eyelid or not) after leaving
// Matrix mode.
Mood currentMood = ALERT;

bool rangeSensorEnabled = false;

// No-iris face: black ring around the edge, white background inside it,
// a single black pupil -- no colored iris ring. Pupil size (and, for the
// bounce animation, its vertical offset) is what carries the mood, so
// this takes whichever display to draw plus the pupil's radius and
// offset from center.
void drawEyeFrame(Adafruit_GC9A01A &display, int pupilRadius, int offsetX, int offsetY) {
  int centerX = display.width() / 2;
  int centerY = display.height() / 2;
  int whiteRadius = min(display.width(), display.height()) / 2 - 20;

  display.fillScreen(GC9A01A_BLACK);
  display.fillCircle(centerX, centerY, whiteRadius, display.color565(255, 255, 255));
  display.fillCircle(centerX + offsetX, centerY + offsetY, pupilRadius, GC9A01A_BLACK);
}

// Static per-mood pupil sizes. CURIOUS is asymmetric on purpose (one
// small pupil, one large) -- everything else matches on both eyes.
const int ALERT_PUPIL_RADIUS = 35;
const int SAD_PUPIL_RADIUS = 90;
const int CURIOUS_SMALL_PUPIL_RADIUS = 22;
const int CURIOUS_LARGE_PUPIL_RADIUS = 80;

// CONFUSED and EXCITED are animated rather than static -- see
// updateEyeAnimationFrame() below, driven from loop().
const unsigned long EYE_ANIM_TICK_MS = 40;
unsigned long lastEyeAnimTickMs = 0;

// CONFUSED and EXCITED are momentary reactions rather than resting states:
// after MOOD_TIMEOUT_MS they revert to ALERT on their own (see loop()),
// silently -- no ALERT sound, since the robot didn't "decide" to be
// alert, it just stopped being confused/excited.
const unsigned long MOOD_TIMEOUT_MS = 5000;
unsigned long moodTimeoutStartMs = 0;

// State for startRevertToAlert()/updateRevertToAlert() (defined further
// down, near setMood()): eases the ears back to ALERT over
// EAR_REVERT_DURATION_MS instead of snapping there instantly.
bool revertingToAlert = false;
unsigned long revertStartMs = 0;
int revertStartLeftCommand = 0;
int revertStartRightCommand = 0;
int revertTargetLeftCommand = 0;
int revertTargetRightCommand = 0;
const unsigned long EAR_REVERT_DURATION_MS = 800;
const unsigned long EAR_REVERT_TICK_MS = 20;
unsigned long lastEarRevertTickMs = 0;

const int CONFUSED_PUPIL_MIN_RADIUS = 25;
const int CONFUSED_PUPIL_MAX_RADIUS = 55;
const float CONFUSED_PHASE_STEP = 0.157f; // ~1.6s per breathing cycle at EYE_ANIM_TICK_MS
float confusedPhase = 0.0f;

// Ears twitch back and forth +-10deg around the mood's base position, in
// sync with the same phase driving each mood's eye animation above.
// Written directly to the servos (not through setEars()) so it doesn't
// spam serial or retrigger a static burst 25x/sec.
const int EAR_WIGGLE_DEG = 10;

const int CONFUSED_RIGHT_EAR_VISUAL_ANGLE = 85; // also used by setEarShape()

const int EXCITED_PUPIL_RADIUS = 40;
const int EXCITED_BOUNCE_RANGE = 20; // pixels above/below center
const float EXCITED_PHASE_STEP = 0.5f; // ~0.5s per bounce cycle at EYE_ANIM_TICK_MS
float excitedPhase = 0.0f;
const int EXCITED_EAR_VISUAL_ANGLE = 150; // also used by setEarShape()

// "Watery" SAD eyes: a soft bluish-white highlight glistens inside the
// pupil, slowly pulsing in brightness -- like light catching welling
// moisture -- rather than literal cartoon teardrops, to keep it robotic
// rather than literally creature-like.
const float SAD_PHASE_STEP = 0.05f; // slow, ~5s glisten cycle at EYE_ANIM_TICK_MS
float sadPhase = 0.0f;
const int SAD_HIGHLIGHT_RADIUS = 14;

// Redraws just the small highlight circle in place (same position/size
// every time, only the color changes), rather than the whole eye.
void updateSadHighlight(Adafruit_GC9A01A &display) {
  float brightness = 0.5f + 0.5f * sin(sadPhase); // 0..1
  uint8_t shade = (uint8_t)(120 + brightness * 135); // 120..255
  uint16_t highlightColor = display.color565(shade, shade, 255);

  int centerX = display.width() / 2;
  int centerY = display.height() / 2;
  int hx = centerX - SAD_PUPIL_RADIUS / 3;
  int hy = centerY - SAD_PUPIL_RADIUS / 2;
  display.fillCircle(hx, hy, SAD_HIGHLIGHT_RADIUS, highlightColor);
}

void drawSadEyeFrame(Adafruit_GC9A01A &display) {
  drawEyeFrame(display, SAD_PUPIL_RADIUS, 0, 0);
  updateSadHighlight(display);
}

// Whether this mood's eyes need per-frame animation from loop().
bool moodHasEyeAnimation(Mood mood) {
  return mood == CONFUSED || mood == EXCITED || mood == SAD;
}

// CONFUSED/EXCITED are momentary reactions that time out back to ALERT on
// their own (see MOOD_TIMEOUT_MS below); SAD is a resting mood like
// ALERT/CURIOUS and stays until explicitly changed, even though its eyes
// now animate too.
bool moodShouldAutoRevert(Mood mood) {
  return mood == CONFUSED || mood == EXCITED;
}

// CONFUSED/EXCITED redraw every tick, but the pupil is the only thing that
// actually moves -- the white background and black ring never change
// while one of those moods is active. Re-doing a full fillScreen() plus
// two full-size fillCircle()s 25x/sec (for both eyes) pushes a lot more
// data over SPI than necessary, and was the source of the tearing/
// "angular" artifacting: erasing just the pupil's last position (a small
// white circle) and drawing the new one is a fraction of the data and
// finishes fast enough that the display doesn't visibly tear mid-update.
// Tracked as one shared position since both eyes always show the same
// pupil frame.
int prevPupilRadius = -1;
int prevPupilOffsetX = 0;
int prevPupilOffsetY = 0;

void updatePupil(Adafruit_GC9A01A &display, int pupilRadius, int offsetX, int offsetY) {
  int centerX = display.width() / 2;
  int centerY = display.height() / 2;

  // NOTE: wrapping these two calls in startWrite()/endWrite() to close the
  // tearing gap made the two moods that use this (CONFUSED/EXCITED) hang
  // the board outright, so this is back to two separate transactions --
  // the minor tearing artifact is far better than an unresponsive robot.
  // Worth revisiting later with a narrower test if we want to chase the
  // tearing further.
  if (prevPupilRadius >= 0) {
    display.fillCircle(centerX + prevPupilOffsetX, centerY + prevPupilOffsetY,
                        prevPupilRadius, display.color565(255, 255, 255));
  }
  display.fillCircle(centerX + offsetX, centerY + offsetY, pupilRadius, GC9A01A_BLACK);
}

// Advances whichever mood's animation is running and redraws both eyes
// with the new frame. Called from loop() only while currentMood is
// CONFUSED, EXCITED, or SAD.
void updateEyeAnimationFrame() {
  if (currentMood == CONFUSED) {
    confusedPhase += CONFUSED_PHASE_STEP;
    int mid = (CONFUSED_PUPIL_MIN_RADIUS + CONFUSED_PUPIL_MAX_RADIUS) / 2;
    int amplitude = (CONFUSED_PUPIL_MAX_RADIUS - CONFUSED_PUPIL_MIN_RADIUS) / 2;
    int radius = mid + (int)(amplitude * sin(confusedPhase));
    updatePupil(leftEyeTft, radius, 0, 0);
    updatePupil(rightEyeTft, radius, 0, 0);
    prevPupilRadius = radius;
    prevPupilOffsetX = 0;
    prevPupilOffsetY = 0;

    int wiggle = (int)(EAR_WIGGLE_DEG * sin(confusedPhase));
    int leftCommand = constrain(leftVisualToServo(EAR_UPRIGHT + wiggle), LEFT_SERVO_COMMAND_MIN, LEFT_SERVO_COMMAND_MAX);
    int rightCommand = constrain(rightVisualToServo(CONFUSED_RIGHT_EAR_VISUAL_ANGLE + wiggle), RIGHT_SERVO_COMMAND_MIN, RIGHT_SERVO_COMMAND_MAX);
    writeEarServos(leftCommand, rightCommand);
  } else if (currentMood == EXCITED) {
    excitedPhase += EXCITED_PHASE_STEP;
    int offsetY = (int)(EXCITED_BOUNCE_RANGE * sin(excitedPhase));
    updatePupil(leftEyeTft, EXCITED_PUPIL_RADIUS, 0, offsetY);
    updatePupil(rightEyeTft, EXCITED_PUPIL_RADIUS, 0, offsetY);
    prevPupilRadius = EXCITED_PUPIL_RADIUS;
    prevPupilOffsetX = 0;
    prevPupilOffsetY = offsetY;

    int wiggle = (int)(EAR_WIGGLE_DEG * sin(excitedPhase));
    int leftCommand = constrain(leftVisualToServo(EXCITED_EAR_VISUAL_ANGLE + wiggle), LEFT_SERVO_COMMAND_MIN, LEFT_SERVO_COMMAND_MAX);
    int rightCommand = constrain(rightVisualToServo(EXCITED_EAR_VISUAL_ANGLE + wiggle), RIGHT_SERVO_COMMAND_MIN, RIGHT_SERVO_COMMAND_MAX);
    writeEarServos(leftCommand, rightCommand);
  } else if (currentMood == SAD) {
    sadPhase += SAD_PHASE_STEP;
    updateSadHighlight(leftEyeTft);
    updateSadHighlight(rightEyeTft);
  }
}

// Secondary display mode: green "Matrix" digital rain, columns of random
// characters scrolling downward with a fading tail. Toggled on/off over
// serial with 'm'; while active it owns the screen instead of the face,
// but ears and sound still respond to mood keys as normal. Both eyes
// mirror the same rain, like the face does.
bool matrixModeActive = false;

const int MATRIX_CHAR_W = 12; // default font at textSize 2: 6px * 2
const int MATRIX_CHAR_H = 16; // default font at textSize 2: 8px * 2
const int MATRIX_COLS = 240 / MATRIX_CHAR_W;
const int MATRIX_ROWS = 240 / MATRIX_CHAR_H;
const int MATRIX_TRAIL_LEN = 6;
const unsigned long MATRIX_TICK_MS = 100;

int matrixDropRow[MATRIX_COLS];
unsigned long lastMatrixTickMs = 0;

char randomMatrixChar() {
  return (char)random(33, 127); // printable ASCII, '!' through '~'
}

// Brightness falls off from the head (level 0) to the end of the tail.
uint16_t matrixShade(int level) {
  static const uint8_t brightness[MATRIX_TRAIL_LEN] = {255, 210, 170, 130, 90, 55};
  if (level >= MATRIX_TRAIL_LEN) level = MATRIX_TRAIL_LEN - 1;
  return leftEyeTft.color565(0, brightness[level], 0);
}

// Clears one character cell to black, then draws a single character in it
// -- clearing first avoids leftover glyph strokes from whatever was drawn
// there before.
void drawMatrixCell(Adafruit_GC9A01A &display, int col, int row, char c, uint16_t color) {
  int x = col * MATRIX_CHAR_W;
  int y = row * MATRIX_CHAR_H;

  display.fillRect(x, y, MATRIX_CHAR_W, MATRIX_CHAR_H, GC9A01A_BLACK);
  display.setCursor(x, y);
  display.setTextSize(2);
  display.setTextColor(color);
  display.print(c);
}

void initMatrixMode() {
  for (int col = 0; col < MATRIX_COLS; col++) {
    matrixDropRow[col] = -random(0, MATRIX_ROWS);
  }

  leftEyeTft.fillScreen(GC9A01A_BLACK);
  rightEyeTft.fillScreen(GC9A01A_BLACK);
}

// Advances every column's drop by one row: redraws the head and its
// fading trail with fresh random characters (real matrix rain flickers
// even mid-trail), then blanks the cell the trail just moved past.
void updateMatrixRain() {
  for (int col = 0; col < MATRIX_COLS; col++) {
    int headRow = matrixDropRow[col];

    for (int i = 0; i < MATRIX_TRAIL_LEN; i++) {
      int row = headRow - i;
      if (row < 0 || row >= MATRIX_ROWS) continue;

      char ch = randomMatrixChar();
      uint16_t color = matrixShade(i);
      drawMatrixCell(leftEyeTft, col, row, ch, color);
      drawMatrixCell(rightEyeTft, col, row, ch, color);
    }

    int eraseRow = headRow - MATRIX_TRAIL_LEN;
    if (eraseRow >= 0 && eraseRow < MATRIX_ROWS) {
      int x = col * MATRIX_CHAR_W;
      int y = eraseRow * MATRIX_CHAR_H;
      leftEyeTft.fillRect(x, y, MATRIX_CHAR_W, MATRIX_CHAR_H, GC9A01A_BLACK);
      rightEyeTft.fillRect(x, y, MATRIX_CHAR_W, MATRIX_CHAR_H, GC9A01A_BLACK);
    }

    headRow++;
    if (headRow - MATRIX_TRAIL_LEN > MATRIX_ROWS) {
      headRow = -random(0, MATRIX_ROWS); // off the bottom -- restart above the top
    }
    matrixDropRow[col] = headRow;
  }
}

// Redraws both eyes for the given mood: a fixed pupil size for ALERT/
// CURIOUS (asymmetric for CURIOUS), or the first frame of an animation
// for CONFUSED/EXCITED/SAD -- loop() takes over animating those from here
// via updateEyeAnimationFrame(). No-op while Matrix mode owns the screen.
void updateEyesForMood(Mood mood) {
  if (matrixModeActive) return;

  switch (mood) {
    case ALERT:
      drawEyeFrame(leftEyeTft, ALERT_PUPIL_RADIUS, 0, 0);
      drawEyeFrame(rightEyeTft, ALERT_PUPIL_RADIUS, 0, 0);
      break;
    case SAD:
      sadPhase = 0.0f;
      drawSadEyeFrame(leftEyeTft);
      drawSadEyeFrame(rightEyeTft);
      break;
    case CURIOUS:
      drawEyeFrame(leftEyeTft, CURIOUS_SMALL_PUPIL_RADIUS, 0, 0);
      drawEyeFrame(rightEyeTft, CURIOUS_LARGE_PUPIL_RADIUS, 0, 0);
      break;
    case CONFUSED:
      confusedPhase = 0.0f;
      drawEyeFrame(leftEyeTft, CONFUSED_PUPIL_MIN_RADIUS, 0, 0);
      drawEyeFrame(rightEyeTft, CONFUSED_PUPIL_MIN_RADIUS, 0, 0);
      prevPupilRadius = CONFUSED_PUPIL_MIN_RADIUS;
      prevPupilOffsetX = 0;
      prevPupilOffsetY = 0;
      break;
    case EXCITED:
      excitedPhase = 0.0f;
      drawEyeFrame(leftEyeTft, EXCITED_PUPIL_RADIUS, 0, 0);
      drawEyeFrame(rightEyeTft, EXCITED_PUPIL_RADIUS, 0, 0);
      prevPupilRadius = EXCITED_PUPIL_RADIUS;
      prevPupilOffsetX = 0;
      prevPupilOffsetY = 0;
      break;
  }
}

// Brief grayscale "no signal" static burst, played on both eyes whenever
// the ears move to a new position -- like the fuzz when changing the
// station on an old analog TV. Deliberately blocky/grayscale rather than
// per-pixel, both to stay fast and because chunky noise reads more like
// a broken transmission than the smooth pupil/iris shapes would.
const int STATIC_BLOCK_SIZE = 6;
const int STATIC_GRID_SIZE = 240 / STATIC_BLOCK_SIZE;
const int STATIC_FRAME_COUNT = 4;
const unsigned long STATIC_FRAME_MS = 22;

void drawStaticFrame(Adafruit_GC9A01A &display) {
  for (int row = 0; row < STATIC_GRID_SIZE; row++) {
    for (int col = 0; col < STATIC_GRID_SIZE; col++) {
      uint8_t gray = random(0, 256);
      uint16_t color = display.color565(gray, gray, gray);
      display.fillRect(col * STATIC_BLOCK_SIZE, row * STATIC_BLOCK_SIZE,
                        STATIC_BLOCK_SIZE, STATIC_BLOCK_SIZE, color);
    }
  }
}

void playEarMoveStatic() {
  if (matrixModeActive) return; // Matrix mode owns the screen; don't fight it

  for (int frame = 0; frame < STATIC_FRAME_COUNT; frame++) {
    drawStaticFrame(leftEyeTft);
    drawStaticFrame(rightEyeTft);
    delay(STATIC_FRAME_MS);
  }
}

void toggleMatrixMode() {
  matrixModeActive = !matrixModeActive;

  if (matrixModeActive) {
    initMatrixMode();
  } else {
    updateEyesForMood(currentMood);
  }
}

void setupDisplay() {
  // Shared bus; CS/DC aren't passed here since each Adafruit_GC9A01A instance
  // above drives its own CS and DC pins directly.
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);

  leftEyeTft.begin();
  rightEyeTft.begin();

  // Both panels are physically mounted upside down, so flip the drawing
  // coordinate system 180deg here rather than adjusting every drawing
  // routine (eyelid, Matrix rain, etc.) to account for it individually.
  leftEyeTft.setRotation(2);
  rightEyeTft.setRotation(2);

  drawEyeFrame(leftEyeTft, ALERT_PUPIL_RADIUS, 0, 0);
  drawEyeFrame(rightEyeTft, ALERT_PUPIL_RADIUS, 0, 0);
}

void setupRangeSensor() {
  if (!rangeSensorEnabled)
    return;
  rangeSensor.setTimeout(500);
  if (!rangeSensor.init()) {
    Serial.println("Failed to detect VL53L1X range sensor");
  } else {
    rangeSensor.setDistanceMode(VL53L1X::Long);
    rangeSensor.setMeasurementTimingBudget(50000);
    rangeSensor.startContinuous(50);
  }
}

void checkRangeForAlert() {
  if (!rangeSensorEnabled)
    return;
  uint16_t distanceMm = rangeSensor.read();
  if (rangeSensor.timeoutOccurred()) return;

  if (distanceMm < RANGE_ALERT_THRESHOLD_MM && currentMood != ALERT) {
    setMood(ALERT);
  }
}


void setup() {
  Serial.begin(115200);
  delay(500);
  
  // Disabling unused radios to avoid current spikes on the shared power rail
  WiFi.mode(WIFI_OFF);
  btStop();
  esp_bt_controller_disable();

  Serial.println("Radios disabled");

  Wire.begin(RANGE_SENSOR_SDA, RANGE_SENSOR_SCL);

  adc2_config_channel_atten(MIC_ADC_CHANNEL, ADC_ATTEN_DB_12);

  setupRangeSensor();
  setupDisplay();

  leftEar.setPeriodHertz(50);
  rightEar.setPeriodHertz(50);

  leftEar.attach(LEFT_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  rightEar.attach(RIGHT_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  Sound::begin();

  setMood(ALERT);

  Serial.println("Ear servo test ready.");
  Serial.println("Type one angle, e.g. 90");
  Serial.println("Or two angles, e.g. 45 120");
  Serial.println("Type 'm' to toggle Matrix rain mode");
}

// Converts desired left visual angle to actual left servo command.
int leftVisualToServo(int visualAngle) {
  return visualAngle + EAR_OFFSET_DEG + leftTrim;
}

// Converts desired right visual angle to actual right servo command.
// This assumes the right servo is mirrored relative to the left.
int rightVisualToServo(int visualAngle) {
  return 180 - (visualAngle + EAR_OFFSET_DEG) + rightTrim;
}

// Tracks the last raw servo commands written, so a slow ease back to
// ALERT (see startRevertToAlert()) knows where to interpolate from.
int lastLeftServoCommand = 0;
int lastRightServoCommand = 0;

void writeEarServos(int leftCommand, int rightCommand) {
  leftEar.write(leftCommand);
  rightEar.write(rightCommand);
  lastLeftServoCommand = leftCommand;
  lastRightServoCommand = rightCommand;
}

void setEars(int leftVisualAngle, int rightVisualAngle) {
  int leftCommand = leftVisualToServo(leftVisualAngle);
  int rightCommand = rightVisualToServo(rightVisualAngle);

  leftCommand = constrain(leftCommand, LEFT_SERVO_COMMAND_MIN, LEFT_SERVO_COMMAND_MAX);
  rightCommand = constrain(rightCommand, RIGHT_SERVO_COMMAND_MIN, RIGHT_SERVO_COMMAND_MAX);

  writeEarServos(leftCommand, rightCommand);

  // Static transition disabled for now -- see playEarMoveStatic() above.
  // playEarMoveStatic();
  updateEyesForMood(currentMood);

  Serial.print("Left visual: ");
  Serial.print(leftVisualAngle);
  Serial.print(" -> servo command: ");
  Serial.print(leftCommand);

  Serial.print(" | Right visual: ");
  Serial.print(rightVisualAngle);
  Serial.print(" -> servo command: ");
  Serial.println(rightCommand);
}

void setEarsSame(int visualAngle) {
  setEars(visualAngle, visualAngle);
}

void setEarShape(Mood mood) {
  switch(mood) {
    case ALERT:
      setEars(EAR_UPRIGHT, EAR_UPRIGHT);
      break;
    case CURIOUS:
      setEarsSame(150);
      break;
    case CONFUSED:
      setEars(EAR_UPRIGHT, CONFUSED_RIGHT_EAR_VISUAL_ANGLE);
      break;
    case SAD:
      setEarsSame(30);
      break;
    case EXCITED:
      setEarsSame(EXCITED_EAR_VISUAL_ANGLE);
      break;
  }
}

Sound::Event soundForMood(Mood mood) {
  switch (mood) {
    case ALERT:
      return Sound::ALERT;
    case CURIOUS:
      return Sound::CURIOUS;
    case CONFUSED:
      return Sound::CONFUSED;
    case SAD:
      return Sound::SAD;
    case EXCITED:
      return Sound::EXCITED;
  }
  return Sound::TICK;
}

// Stops any in-progress or pending SAD purr sequence.
void cancelSadPurr() {
  sadPurrDelayPending = false;

  if (sadPurrHeldActive) {
    Sound::setHeldMode(false);
    sadPurrHeldActive = false;
  }
}

// Advances the SAD purr state machine. Non-blocking, called every loop().
void updateSadPurr() {
  if (sadPurrDelayPending && millis() - sadPurrTimerStart >= SAD_PURR_DELAY_MS) {
    sadPurrDelayPending = false;
    sadPurrHeldActive = true;
    sadPurrTimerStart = millis();
    Sound::setHeldMode(true);
    return;
  }

  if (sadPurrHeldActive && millis() - sadPurrTimerStart >= SAD_PURR_DURATION_MS) {
    sadPurrHeldActive = false;
    Sound::setHeldMode(false);
  }
}

// Sets the ear shape, updates the eyes, and plays the matching sound so
// all three stay in sync. SAD is special-cased: the purr starts after a
// short delay and then loops seamlessly for a few seconds instead of
// firing as an instant one-shot.
void setMood(Mood mood) {
  currentMood = mood;
  setEarShape(mood);
  updateEyesForMood(mood);
  cancelSadPurr();
  revertingToAlert = false; // an explicit mood change overrides any pending auto-revert

  if (moodShouldAutoRevert(mood)) {
    moodTimeoutStartMs = millis();
  }

  if (mood == CURIOUS) {
    micLastTriggerMs = millis();
  }

  Sound::play(soundForMood(mood));
  /*
  if (mood == SAD) {
    sadPurrDelayPending = true;
    sadPurrTimerStart = millis();
  } else {
  }
  */
}

// Starts a slow return to ALERT (no sound) when CONFUSED/EXCITED time out
// on their own, rather than the user explicitly picking a mood. The eyes
// switch to the ALERT face right away, but the ears ease from wherever
// they were left (mid-wiggle) to the ALERT position over
// EAR_REVERT_DURATION_MS instead of snapping there instantly --
// updateRevertToAlert(), ticked from loop(), does the easing.
void startRevertToAlert() {
  currentMood = ALERT;
  updateEyesForMood(ALERT);
  cancelSadPurr();

  revertingToAlert = true;
  revertStartMs = millis();
  revertStartLeftCommand = lastLeftServoCommand;
  revertStartRightCommand = lastRightServoCommand;
  revertTargetLeftCommand = constrain(leftVisualToServo(EAR_UPRIGHT), LEFT_SERVO_COMMAND_MIN, LEFT_SERVO_COMMAND_MAX);
  revertTargetRightCommand = constrain(rightVisualToServo(EAR_UPRIGHT), RIGHT_SERVO_COMMAND_MIN, RIGHT_SERVO_COMMAND_MAX);
}

void updateRevertToAlert() {
  if (!revertingToAlert) return;
  if (millis() - lastEarRevertTickMs < EAR_REVERT_TICK_MS) return;
  lastEarRevertTickMs = millis();

  float t = (float)(millis() - revertStartMs) / EAR_REVERT_DURATION_MS;
  if (t >= 1.0f) {
    t = 1.0f;
    revertingToAlert = false;
  }

  int leftCommand = revertStartLeftCommand + (int)((revertTargetLeftCommand - revertStartLeftCommand) * t);
  int rightCommand = revertStartRightCommand + (int)((revertTargetRightCommand - revertStartRightCommand) * t);
  writeEarServos(leftCommand, rightCommand);
}

void handleMoodKey(char key) {
  switch (key) {
    case 'a':
      Serial.println("Mood: ALERT");
      setMood(ALERT);
      break;
    case 'o':
      Serial.println("Mood: CURIOUS");
      setMood(CURIOUS);
      break;
    case 'e':
      Serial.println("Mood: CONFUSED");
      setMood(CONFUSED);
      break;
    case 'u':
      Serial.println("Mood: SAD");
      setMood(SAD);
      break;
    case 'i':
      Serial.println("Mood: EXCITED");
      setMood(EXCITED);
      break;
  }
}

void handleAngleLine(const String& input) {
  int spaceIndex = input.indexOf(' ');

  if (spaceIndex == -1) {
    int angle = input.toInt();

    Serial.print("Setting both ears to ");
    Serial.println(angle);

    setEarsSame(angle);
  } else {
    int leftAngle = input.substring(0, spaceIndex).toInt();
    int rightAngle = input.substring(spaceIndex + 1).toInt();

    Serial.print("Setting left ear to ");
    Serial.print(leftAngle);
    Serial.print(" and right ear to ");
    Serial.println(rightAngle);

    setEars(leftAngle, rightAngle);
  }
}

String lineBuffer;

// Non-blocking: takes one sample per loop() call and only evaluates the
// peak-to-peak swing once a full MIC_SAMPLE_WINDOW_MS window has accumulated.
void updateMicListening() {
  int sample = 0;
  if (adc2_get_raw(MIC_ADC_CHANNEL, ADC_WIDTH_BIT_12, &sample) == ESP_OK) {
    if (sample < micWindowMin) micWindowMin = sample;
    if (sample > micWindowMax) micWindowMax = sample;
  }

  if (millis() - micWindowStartMs < MIC_SAMPLE_WINDOW_MS) return;

  int peakToPeak = micWindowMax - micWindowMin;
  micWindowStartMs = millis();
  micWindowMin = 4095;
  micWindowMax = 0;

  // TEMP debug: remove once mic behavior is sorted out.
  Serial.println(peakToPeak);

  if (peakToPeak > MIC_CURIOUS_THRESHOLD) {
    micLastTriggerMs = millis();
    micConsecutiveOverThreshold++;

    if (currentMood != CURIOUS && micConsecutiveOverThreshold >= MIC_CURIOUS_DEBOUNCE_WINDOWS) {
      Serial.print("CURIOUS mode triggered by microphone at ");
      Serial.println(peakToPeak);
      setMood(CURIOUS);
    }
  } else {
    micConsecutiveOverThreshold = 0;
  }
}

void loop() {
  updateSadPurr();

  if (millis() - lastRangeCheckMs >= RANGE_CHECK_INTERVAL_MS) {
    lastRangeCheckMs = millis();
    checkRangeForAlert();
  }

  updateMicListening();

  if (matrixModeActive && millis() - lastMatrixTickMs >= MATRIX_TICK_MS) {
    lastMatrixTickMs = millis();
    updateMatrixRain();
  }

  if (moodHasEyeAnimation(currentMood) && !matrixModeActive && millis() - lastEyeAnimTickMs >= EYE_ANIM_TICK_MS) {
    lastEyeAnimTickMs = millis();
    updateEyeAnimationFrame();
  }

  if (moodShouldAutoRevert(currentMood) && millis() - moodTimeoutStartMs >= MOOD_TIMEOUT_MS) {
    startRevertToAlert();
  }

  if (currentMood == CURIOUS && millis() - micLastTriggerMs >= CURIOUS_SILENCE_MS) {
    startRevertToAlert();
  }

  updateRevertToAlert();

  while (Serial.available()) {
    char c = Serial.read();

    // Mood keys fire immediately, without waiting for Enter.
    if (c == 'a' || c == 'o' || c == 'e' || c == 'u' || c == 'i') {
      handleMoodKey(c);
      continue;
    }

    // 'm' toggles Matrix rain mode on/off, also without waiting for Enter.
    if (c == 'm') {
      toggleMatrixMode();
      Serial.println(matrixModeActive ? "Matrix mode: ON" : "Matrix mode: OFF");
      continue;
    }

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      lineBuffer.trim();
      if (lineBuffer.length() > 0) {
        handleAngleLine(lineBuffer);
      }
      lineBuffer = "";
      continue;
    }

    lineBuffer += c;
  }
}
