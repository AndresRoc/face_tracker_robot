
/*  *************************************************************************************
 * The code controls the speed and direction of four motors                     *
 * using serial messages to the Arduino platform.                                       *
 *                                                                                      *
 * Written by: Andres Campo <andres.rodriguez.campo@gmail.com>                          *
 * Date: 04-Apr-23                                                                      *
 * **************************************************************************************
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

const unsigned long TIMEOUT = 1000; // Timeout in milliseconds

unsigned long last_command_time = 0;
// This function will map the correct motor and direction value for the motor
void pwm_direction_map(int motor_pin, float motor_speed, int direction_pin) {
    int pwm_value;
    bool dir_bool;
    
    if (motor_speed == 0) {
        pwm_value = 0;
        dir_bool = true;  // Choose based on your setup. It could be false.
    } else {
        if (motor_speed < 0) {
            dir_bool = false; // If the speed is negative, set direction to reverse (can be HIGH or LOW based on your setup)
            motor_speed = -motor_speed; // Take the absolute value of the speed
        } else {
            dir_bool = true; // If the speed is positive, set direction to forward (can be HIGH or LOW based on your setup)
        }
        pwm_value = map(constrain(motor_speed, 0.0, 10.0), 0.0, 1.0, 0, 255);
    }
    
    analogWrite(motor_pin, pwm_value);
    digitalWrite(direction_pin, dir_bool ? HIGH : LOW);
}


void stop_motor() {
  // Serial.println("Stopping motors");
  int stop_pwm = 0;
  analogWrite(MOTOR_1_SPEED_PIN, stop_pwm);
  analogWrite(MOTOR_2_SPEED_PIN, stop_pwm);
  analogWrite(MOTOR_3_SPEED_PIN, stop_pwm);
  analogWrite(MOTOR_4_SPEED_PIN, stop_pwm);
}

// define a function to process incoming motor commands
void process_motor_command(float linear_velocity_x, float linear_velocity_y, float angular_velocity_z) {
  // calculate motor speeds for mecanum wheels
  float motor_1_speed = linear_velocity_x - linear_velocity_y - angular_velocity_z;
  float motor_2_speed = linear_velocity_x + linear_velocity_y + angular_velocity_z;
  float motor_3_speed = linear_velocity_x + linear_velocity_y - angular_velocity_z;
  float motor_4_speed = linear_velocity_x - linear_velocity_y + angular_velocity_z;

  // normalize motor speeds between -1 and 1
  float max_speed = max(abs(motor_1_speed), max(abs(motor_2_speed), max(abs(motor_3_speed), abs(motor_4_speed))));
  
  if(max_speed > 1) {
      motor_1_speed /= max_speed;
      motor_2_speed /= max_speed;
      motor_3_speed /= max_speed;
      motor_4_speed /= max_speed;
  }
  // Print out the calculated motor speeds
  Serial.print("Motor 1 speed: "); Serial.println(motor_1_speed);
  Serial.print("Motor 2 speed: "); Serial.println(motor_2_speed);
  Serial.print("Motor 3 speed: "); Serial.println(motor_3_speed);
  Serial.print("Motor 4 speed: "); Serial.println(motor_4_speed);
  // set the motor speeds and directions
  pwm_direction_map(MOTOR_1_SPEED_PIN, motor_1_speed, MOTOR_1_DIRECTION_PIN);
  pwm_direction_map(MOTOR_2_SPEED_PIN, motor_2_speed, MOTOR_2_DIRECTION_PIN);
  pwm_direction_map(MOTOR_3_SPEED_PIN, motor_3_speed, MOTOR_3_DIRECTION_PIN);
  pwm_direction_map(MOTOR_4_SPEED_PIN, motor_4_speed, MOTOR_4_DIRECTION_PIN);
}



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
  if (Serial.available() > 0) {
    char incomingByte[30];
    Serial.readBytesUntil('\n', incomingByte, sizeof(incomingByte));

    // Parse the data to get the linear and angular velocities
    char *token = strtok(incomingByte, ",");
    float linear_velocity_x = atof(token);    
    token = strtok(NULL, ",");
    float linear_velocity_y = atof(token);
    token = strtok(NULL, ",");
    float angular_velocity_z = atof(token);

    // Executes motor command in velocity commands
    process_motor_command(linear_velocity_x, linear_velocity_y, angular_velocity_z);

    // Update the time of the last command received
    last_command_time = millis();
  }

  // If no commands have been received for the timeout period, stop the motor
  if (millis() - last_command_time > TIMEOUT) {
    stop_motor();
  }
    
}
