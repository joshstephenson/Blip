/*
 * blip_sound_localization.ino
 *
 * Estimates the horizontal angle of a sound source using two synchronized
 * I2S microphones and turns a servo toward it.
 *
 * ALGORITHM: GCC-PHAT (Generalized Cross-Correlation with Phase Transform)
 *   1. Capture a short synchronized stereo window from both mics.
 *   2. FFT both channels.
 *   3. Compute the cross-power spectrum, normalize by magnitude (the
 *      "phase transform" -- this is what makes GCC-PHAT robust to
 *      differences in the two mics' volume/frequency response).
 *   4. Inverse FFT to get a correlation function; its peak location tells
 *      you the sample delay (lag) between the two channels.
 *   5. Convert that lag to a time delay, then to an angle using the mic
 *      spacing and the speed of sound.
 *
 * LIMITATIONS TO KNOW GOING IN:
 *   - Two mics can't distinguish front from back -- a source directly
 *     ahead and one directly behind produce the same delay. Fine for a
 *     robot that mostly cares about "which way to turn," not for full
 *     360 degree localization (that needs 3+ mics).
 *   - Close mic spacing (10cm) means the maximum possible delay is small
 *     (a handful of samples at 16kHz) -- angle resolution is coarser
 *     near the sides than you might expect. Wider spacing = better
 *     resolution but smaller unambiguous range and a bulkier head.
 *   - Sign convention (does a positive angle mean "turn left" or "turn
 *     right") depends on which physical mic you wired as channel L vs R.
 *     Calibrate this empirically -- see the test procedure in the
 *     comment near turnToward().
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <ESP32Servo.h>

// ---- I2S pins (shared by both mics) ----
#define I2S_WS_PIN   25   // word select / LRCLK
#define I2S_SD_PIN   32   // data in (both mics tied to this pin)
#define I2S_SCK_PIN  33   // bit clock

// ---- Servo ----
#define SERVO_PIN    27
#define SERVO_CENTER 90   // pulse angle when facing forward -- calibrate on your mount

// ---- Audio / DSP config ----
#define SAMPLE_RATE      16000
#define FFT_SIZE         256        // ~16ms window at 16kHz
#define MIC_SPACING_M    0.10       // meters between the two mics -- measure yours
#define SPEED_OF_SOUND   343.0      // m/s

double vRealL[FFT_SIZE], vImagL[FFT_SIZE];
double vRealR[FFT_SIZE], vImagR[FFT_SIZE];
double crossReal[FFT_SIZE], crossImag[FFT_SIZE];

ArduinoFFT<double> fftL(vRealL, vImagL, FFT_SIZE, (double)SAMPLE_RATE);
ArduinoFFT<double> fftR(vRealR, vImagR, FFT_SIZE, (double)SAMPLE_RATE);
ArduinoFFT<double> fftCross(crossReal, crossImag, FFT_SIZE, (double)SAMPLE_RATE);

Servo panServo;

void setupI2S() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // INMP441 outputs 24-bit data in a 32-bit slot
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // stereo: both mics' channels
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = FFT_SIZE,
    .use_apll = false
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };
  i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
}

// Captures FFT_SIZE stereo samples and splits them into the left/right
// working buffers, removing DC offset.
void captureStereo() {
  int32_t rawBuf[FFT_SIZE * 2]; // interleaved L,R,L,R...
  size_t bytesRead = 0;
  i2s_read(I2S_NUM_0, rawBuf, sizeof(rawBuf), &bytesRead, portMAX_DELAY);

  double meanL = 0, meanR = 0;
  for (int i = 0; i < FFT_SIZE; i++) {
    vRealL[i] = (double)(rawBuf[2 * i] >> 8);      // shift down from 32-bit slot
    vRealR[i] = (double)(rawBuf[2 * i + 1] >> 8);
    vImagL[i] = 0; vImagR[i] = 0;
    meanL += vRealL[i]; meanR += vRealR[i];
  }
  meanL /= FFT_SIZE; meanR /= FFT_SIZE;
  for (int i = 0; i < FFT_SIZE; i++) {
    vRealL[i] -= meanL;
    vRealR[i] -= meanR;
  }
}

// Runs GCC-PHAT on the captured buffers and returns the estimated angle
// in degrees. 0 = straight ahead, negative = one side, positive = other
// (calibrate the sign against your physical mic wiring).
float estimateAngle() {
  fftL.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fftR.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fftL.compute(FFTDirection::Forward);
  fftR.compute(FFTDirection::Forward);

  // Cross-power spectrum: X_L * conj(X_R), then normalize magnitude to 1
  // (the "phase transform" step -- keeps only timing info, discards
  // relative loudness so the estimate isn't skewed by one mic being
  // louder than the other).
  for (int k = 0; k < FFT_SIZE; k++) {
    double cr = vRealL[k] * vRealR[k] + vImagL[k] * vImagR[k];
    double ci = vImagL[k] * vRealR[k] - vRealL[k] * vImagR[k];
    double mag = sqrt(cr * cr + ci * ci) + 1e-12; // epsilon avoids divide-by-zero
    crossReal[k] = cr / mag;
    crossImag[k] = ci / mag;
  }

  fftCross.compute(FFTDirection::Reverse);

  // Find the peak of the correlation function -> that index is the lag.
  int peakIdx = 0;
  double peakVal = -1e18;
  for (int i = 0; i < FFT_SIZE; i++) {
    if (crossReal[i] > peakVal) { peakVal = crossReal[i]; peakIdx = i; }
  }

  int lag = (peakIdx > FFT_SIZE / 2) ? (peakIdx - FFT_SIZE) : peakIdx;
  double tau = (double)lag / SAMPLE_RATE;             // seconds

  double tauMax = MIC_SPACING_M / SPEED_OF_SOUND;      // max physically possible delay
  double ratio = constrain(tau / tauMax, -1.0, 1.0);
  double angleRad = asin(ratio);
  return angleRad * 180.0 / PI;
}

/*
 * CALIBRATION PROCEDURE (do this before trusting the sign/scale):
 *   1. Play a short sound directly in front of blip. Angle should read
 *      close to 0. If it doesn't, check mic mounting symmetry.
 *   2. Play a sound clearly to blip's left. Note the sign of the angle
 *      returned. If "left" gives a positive number but you want turning
 *      left to be negative (or vice versa), just flip the sign in
 *      turnToward() below -- simpler than re-deriving the math.
 *   3. Repeat on the right to confirm it's consistent and opposite sign.
 */
void turnToward(float angleDeg) {
  angleDeg = constrain(angleDeg, -80.0, 80.0); // stay within servo's safe range
  int servoPos = SERVO_CENTER - (int)angleDeg;  // flip sign here if step 2 above disagrees
  servoPos = constrain(servoPos, 0, 180);
  panServo.write(servoPos);
}

void setup() {
  Serial.begin(115200);
  setupI2S();
  panServo.attach(SERVO_PIN);
  panServo.write(SERVO_CENTER);
}

void loop() {
  // In the full build, this only runs after the comparator/wake-on-sound
  // tier has already woken the ESP32 -- localization is too power-hungry
  // to run continuously. Here it's left as a free-running loop for testing.
  captureStereo();
  float angle = estimateAngle();
  Serial.printf("Estimated angle: %.1f deg\n", angle);
  turnToward(angle);
  delay(500);
}
