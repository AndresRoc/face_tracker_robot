/* *************************************************************************************
 * The code controls the speed and direction of four motors using serial messages       *
 * to the Arduino platform.                                                            *
 *                                                                                     *
 * Written by: Andres Campo <andres.rodriguez.campo@gmail.com>                         *
 * Date: 14-Apr-23                                                                     *
 * *************************************************************************************
 */
// define constants for the motor speed and direction
const int MOTOR_1_SPEED_PIN = 2;
const int MOTOR_1_DIRECTION_PIN = 3;
const int MOTOR_2_SPEED_PIN = 4;
const int MOTOR_2_DIRECTION_PIN = 5;
const int MOTOR_3_SPEED_PIN = 6;
const int MOTOR_3_DIRECTION_PIN = 7;
const int MOTOR_4_SPEED_PIN = 8;
const int MOTOR_4_DIRECTION_PIN = 9;
const int image_width = 640;

unsigned long previousMillis = 0;        // will store the last time data was sent
const long interval = 500;              // interval at which to send data (in milliseconds)

int targetSpeed = 1270;  // The desired MAX speed for the motor in RPM
const int tolerance = 10;  // Tolerance level for speed matching
volatile int encoderPulses = 0;  // Variable to store the number of pulses

unsigned long prevTime = 0;
int currentPWM = 0;  // The current PWM value

void setup() {
  pinMode(MOTOR_1_SPEED_PIN, OUTPUT);
  pinMode(MOTOR_1_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_2_SPEED_PIN, OUTPUT);
  pinMode(MOTOR_2_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_3_SPEED_PIN, OUTPUT);
  pinMode(MOTOR_3_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_4_SPEED_PIN, OUTPUT);
  pinMode(MOTOR_4_DIRECTION_PIN, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input.startsWith("PWM")) {
        int newPWM = input.substring(4).toInt();
        setPWM(newPWM);
      } else if (input.startsWith("DIR")) {
        int separatorIndex = input.indexOf(' ', 3);
        if (separatorIndex > 0) {
          int m1_dir = input.substring(3, separatorIndex).toInt();
          int m2_dir = input.substring(separatorIndex + 1).toInt();
          int m3_dir = input.substring(separatorIndex + 1).toInt();
          int m4_dir = input.substring(separatorIndex + 1).toInt();
          setDirection(m1_dir, m2_dir, m3_dir, m4_dir);
        }

      } else if (input.startsWith("STP")) {
        stop_motors();
      } else if (input.startsWith("FACE")) {
        int separatorIndex = input.indexOf(' ', 4);  // Start looking for a space after "FACE"
        if (separatorIndex > 0) {
          int x_coordinate = input.substring(5, separatorIndex).toInt();  // Get the x-coordinate
          int face_distance = input.substring(separatorIndex + 1).toInt();  // Get the face_distance
          setSpeeds(x_coordinate, face_distance);
        }
      
      } else if (input.startsWith("MOVE")) {
        int separatorIndex = input.indexOf(' ', 5);
        if (separatorIndex > 0) {
          int speedSeparatorIndex = input.indexOf(' ', 5);
          int speed = (255 * (input.substring(5, speedSeparatorIndex).toInt()))/100;
          
          int m1_dirSeparatorIndex = input.indexOf(' ', speedSeparatorIndex+1);
          int m1_dir = input.substring(speedSeparatorIndex+1, m1_dirSeparatorIndex).toInt();
          
          int m2_dirSeparatorIndex = input.indexOf(' ', m1_dirSeparatorIndex+1);
          int m2_dir = input.substring(m1_dirSeparatorIndex+1, m2_dirSeparatorIndex).toInt();
          
          int m3_dirSeparatorIndex = input.indexOf(' ', m2_dirSeparatorIndex+1);
          int m3_dir = input.substring(m2_dirSeparatorIndex+1, m3_dirSeparatorIndex).toInt();
          
          int m4_dir = input.substring(m3_dirSeparatorIndex+1).toInt();
          setDirection(m1_dir, m2_dir, m3_dir, m4_dir);
          setPWM(speed);
        }
      }
    }
  }
    // delay(500);
}

void setDirection(bool m1_dir, bool m2_dir, bool m3_dir, bool m4_dir) {
  digitalWrite(MOTOR_1_DIRECTION_PIN, m1_dir ? HIGH : LOW);
  digitalWrite(MOTOR_2_DIRECTION_PIN, m2_dir ? HIGH : LOW);
  digitalWrite(MOTOR_3_DIRECTION_PIN, m3_dir ? HIGH : LOW);
  digitalWrite(MOTOR_4_DIRECTION_PIN, m4_dir ? HIGH : LOW);
}

void setPWM(int pwmValue) {
  analogWrite(MOTOR_1_SPEED_PIN, pwmValue);
  analogWrite(MOTOR_2_SPEED_PIN, pwmValue);
  analogWrite(MOTOR_3_SPEED_PIN, pwmValue);
  analogWrite(MOTOR_4_SPEED_PIN, pwmValue);
  currentPWM = pwmValue;
}

void stop_motors(){
  int stop_pwm = 0;
  analogWrite(MOTOR_1_SPEED_PIN, stop_pwm);
  analogWrite(MOTOR_2_SPEED_PIN, stop_pwm);
  analogWrite(MOTOR_3_SPEED_PIN, stop_pwm);
  analogWrite(MOTOR_4_SPEED_PIN, stop_pwm);
}

void setSpeeds(int x_coordinate, int face_distance) {
  // Parameters
  float kp = 0.1;  // Proportional gain for x-coordinate control
  float kd = 0.01;  // Proportional gain for distance control
  int target_speed = 127;  // Initialize target speed to half of the maximum speed

  // Calculate the error in x-coordinate
  int x_error = 0;
  if (x_coordinate < 220) {
    x_error = 300 - x_coordinate;  // 300 is the middle of your desired range (280-380)
  } else if (x_coordinate > 420) {
    x_error = 300 - x_coordinate;  // 300 is the middle of your desired range (280-380)
  }

  // Calculate the distance error
  int distance_error = 0;
  if (face_distance < 100) {
    distance_error = 100 - face_distance;
  } else if (face_distance > 200) {
    distance_error = 150 - face_distance;  // 150 is the middle of your desired range (100-200)
  }

  // Calculate speed adjustment based on x-coordinate error
  int speed_adjustment = kp * x_error;
  
  // Adjust the target speed based on distance error
  target_speed += kd * distance_error;

  // Calculate the speed of each motor
  int left_motor_speed = constrain(target_speed - speed_adjustment, 0, 255);
  int right_motor_speed = constrain(target_speed + speed_adjustment, 0, 255);

  // Set the speed of each motor
  analogWrite(MOTOR_1_SPEED_PIN, left_motor_speed);
  analogWrite(MOTOR_2_SPEED_PIN, left_motor_speed);
  analogWrite(MOTOR_3_SPEED_PIN, right_motor_speed);
  analogWrite(MOTOR_4_SPEED_PIN, right_motor_speed);
}