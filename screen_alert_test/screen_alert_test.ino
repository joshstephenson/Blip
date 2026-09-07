// Isolated screen-drawing test: left eye display only, ALERT mood only.
// No servos, no sound, no mic/range sensors, no serial mood switching,
// no Matrix mode -- just the SPI/display wiring and the ALERT eye frame,
// lifted straight from CatBot.ino's drawEyeFrame()/ALERT_PUPIL_RADIUS.

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

const int TFT_SCLK = 18; // SCL
const int TFT_MOSI = 23; // SDA
const int TFT_DC   = 22;
const int LEFT_TFT_CS = 5;
const int LEFT_TFT_DC = 13;
const int LEFT_TFT_RST  = 19;
const int RIGHT_TFT_CS = 4;
const int RIGHT_TFT_RST  = 21;

Adafruit_GC9A01A leftEyeTft(&SPI, TFT_DC, LEFT_TFT_CS, LEFT_TFT_RST);
Adafruit_GC9A01A rightEyeTft(&SPI, TFT_DC, RIGHT_TFT_CS, RIGHT_TFT_RST);

const int ALERT_PUPIL_RADIUS = 35;

// No-iris face: black ring around the edge, white background inside it,
// a single black pupil -- identical to CatBot.ino's drawEyeFrame().
void drawEyeFrame(Adafruit_GC9A01A &display, int pupilRadius, int offsetX, int offsetY) {
  int centerX = display.width() / 2;
  int centerY = display.height() / 2;
  int whiteRadius = min(display.width(), display.height()) / 2 - 20;

  display.fillScreen(GC9A01A_BLACK);
  display.fillCircle(centerX, centerY, whiteRadius, display.color565(255, 255, 255));
  display.fillCircle(centerX + offsetX, centerY + offsetY, pupilRadius, GC9A01A_BLACK);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  SPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);

  leftEyeTft.begin();
  rightEyeTft.begin();

  // Panel is physically mounted upside down -- flip the drawing coordinate
  // system 180deg here, same as CatBot.ino.
  leftEyeTft.setRotation(2);
  rightEyeTft.setRotation(2);

  drawEyeFrame(leftEyeTft, ALERT_PUPIL_RADIUS, 0, 0);
  drawEyeFrame(rightEyeTft, ALERT_PUPIL_RADIUS, 0, 0);

  Serial.println("Left eye: ALERT mood drawn. Nothing else is running.");
}

void loop() {
  // Static face -- nothing to do here.
}
