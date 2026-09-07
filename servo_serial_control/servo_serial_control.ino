// Basic ESP32 sketch: type a degree value (0-180) into Serial Monitor
// and press Enter to move the servo to that angle.

#include <ESP32Servo.h>

const int SERVO_PIN = 26;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

Servo myServo;

void setup() {
  Serial.begin(115200);
  myServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  Serial.println("Enter a servo angle (0-180) and press Enter:");
}

void loop() {
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();

    // Discard the trailing newline/whitespace left in the buffer.
    while (Serial.available() > 0 && !isDigit(Serial.peek())) {
      Serial.read();
    }

    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
      Serial.print("Moved to ");
      Serial.print(angle);
      Serial.println(" degrees.");
    } else {
      Serial.println("Please enter a value between 0 and 180.");
    }
  }
}
