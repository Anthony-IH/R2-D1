/*
   7MRI0060 - Applied Medical Robotics Module
   October 2024
   Author: Alejandro Granados and Harry Robertshaw

   Purpose: Run a PID controller and read encoder
*/

// Define the pins connected to encoder channels A and B
#define outputA_m1 3   // Channel A of the encoder for motor 1
#define outputB_m1 11  // Channel B of the encoder for motor 1
#define outputA_m2 2   // Channel A of the encoder for motor 2
#define outputB_m2 10  // Channel B of the encoder for motor 2

#define current_sensor A2  // Current sensor for motor 2

// Pins for setting the direction of motor 1 and 2
const int motorPin1_m1 = 4;
const int motorPin2_m1 = 5;
const int motorPin1_m2 = 7;
const int motorPin2_m2 = 8;

// Pins for setting the speed of rotation (Enable pin) of motors 1 and 2
const int enablePin_m1 = 6;
const int enablePin_m2 = 9;

// Declare variables
volatile long counter_m1 = 0;  // Position of motor 1
volatile long counter_m2 = 0;  // Position of motor 2

int minspeed = 54; // 54 min
int maxspeed = 255;

// Variables for encoder positions and desired positions
float demandPositionInDegrees_m1 = 0;
float demandPositionInDegrees_m2 = 0;
int aLastState_m1;
int aLastState_m2;
String spin_m1 = " ";
String spin_m2 = " ";

float currentPositionInDegrees_m1;
float currentPositionInDegrees_m2;

// Time parameters
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long deltaT;

// PID gains - Can be updated with GUI dynamically
float Kp_m1 = 2, Kd_m1 = 0.1, Ki_m1 = 0.1; // Adjusted PID gains for smoother ramp-up
float Kp_m2 = 1.0, Kd_m2 = 0.5, Ki_m2 = 0.05;

// Error values
float errorPositionInDegrees_prev_m1 = 0, errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0, errorPositionInDegrees_sum_m2 = 0;

// Integral windup limits
const float integralLimit_m1 = 1; // Adjust this value as needed
const float integralLimit_m2 = 1; // Adjust this value as needed

// Encoder constants for PPR and GR
const float PPR = 3575.0855;
const float GR = 297.924;

// Calculate degrees per count because encoder is on output shaft
const float degreesPerCount_m1 = 360.0 / (PPR / 2);
const float degreesPerCount_m2 = 360.0 / (PPR * 1.1667);



// Array to store the last 10 current readings
int currentReadings[10] = {0};
int currentIndex = 0;
float currentAverage = 0;

// Define PWM ramping parameters
const int PWM_STEP = 20; // PWM change per control loop (adjust as needed)
int prev_pwm_m1 = 0;
int prev_pwm_m2 = 0;

// Helper function to ramp PWM values
int rampPWM(int currentPWM, int targetPWM) {
  if (currentPWM < targetPWM) {
    currentPWM += PWM_STEP;
    if (currentPWM > targetPWM) currentPWM = targetPWM;
  }
  else if (currentPWM > targetPWM) {
    // Allow immediate ramp-down
    currentPWM = targetPWM;
  }
  return currentPWM;
}

void setup() {
  pinMode(outputA_m1, INPUT_PULLUP);
  pinMode(outputB_m1, INPUT_PULLUP);
  pinMode(outputA_m2, INPUT_PULLUP);
  pinMode(outputB_m2, INPUT_PULLUP);

  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);
  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);

  Serial.begin(115200);

  aLastState_m1 = digitalRead(outputA_m1);
  aLastState_m2 = digitalRead(outputA_m2);

  attachInterrupt(digitalPinToInterrupt(outputA_m1), updateEncoder_m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputA_m2), updateEncoder_m2, CHANGE);

  previousTime = micros();
}

void loop() {
  // Handle incoming serial data
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any leading/trailing whitespace

    // Check if the input is a PID command
    if (input.startsWith("PID:")) {
      // Remove "PID:" prefix
      String pid_values = input.substring(4);

      // Split the string by commas
      int first_comma = pid_values.indexOf(',');
      int second_comma = pid_values.indexOf(',', first_comma + 1);
      int third_comma = pid_values.indexOf(',', second_comma + 1);
      int fourth_comma = pid_values.indexOf(',', third_comma + 1);
      int fifth_comma = pid_values.indexOf(',', fourth_comma + 1);

      // Extract the values
      if (first_comma > 0 && second_comma > first_comma &&
          third_comma > second_comma && fourth_comma > third_comma &&
          fifth_comma > fourth_comma) {
        float m1_Kp_new = pid_values.substring(0, first_comma).toFloat();
        float m1_Ki_new = pid_values.substring(first_comma + 1, second_comma).toFloat();
        float m1_Kd_new = pid_values.substring(second_comma + 1, third_comma).toFloat();
        float m2_Kp_new = pid_values.substring(third_comma + 1, fourth_comma).toFloat();
        float m2_Ki_new = pid_values.substring(fourth_comma + 1, fifth_comma).toFloat();
        float m2_Kd_new = pid_values.substring(fifth_comma + 1).toFloat();

        // Update the PID gains
        Kp_m1 = m1_Kp_new;
        Ki_m1 = m1_Ki_new;
        Kd_m1 = m1_Kd_new;
        Kp_m2 = m2_Kp_new;
        Ki_m2 = m2_Ki_new;
        Kd_m2 = m2_Kd_new;

        Serial.println("PID gains updated");
      } else {
        Serial.println("Invalid PID command format");
      }
    }
    else {
      // Existing position command handling
      int commaIndex = input.indexOf(',');
      if (commaIndex > 0) {
        demandPositionInDegrees_m1 = input.substring(0, commaIndex).toFloat();
        demandPositionInDegrees_m2 = input.substring(commaIndex + 1).toFloat();
      }
    }
  }

  // Calculate current positions in degrees
  currentPositionInDegrees_m1 = counter_m1 * degreesPerCount_m1;
  currentPositionInDegrees_m2 = counter_m2 * degreesPerCount_m2;

  currentTime = micros();
  deltaT = currentTime - previousTime;

  // PID control loop at intervals greater than 40,000 microseconds (40 ms)
  if (deltaT > 40000) {
    // Calculate errors
    float errorPositionInDegrees_m1 = demandPositionInDegrees_m1 - currentPositionInDegrees_m1;
    float errorPositionInDegrees_m2 = demandPositionInDegrees_m2 - currentPositionInDegrees_m2;

    // Integral term with windup prevention
    errorPositionInDegrees_sum_m1 += errorPositionInDegrees_m1 * (deltaT / 1e6);
    errorPositionInDegrees_sum_m2 += errorPositionInDegrees_m2 * (deltaT / 1e6);

    // Added lines for integral windup prevention
    errorPositionInDegrees_sum_m1 = constrain(errorPositionInDegrees_sum_m1, -integralLimit_m1, integralLimit_m1);
    errorPositionInDegrees_sum_m2 = constrain(errorPositionInDegrees_sum_m2, -integralLimit_m2, integralLimit_m2);

    // Derivative term
    float errorPositionInDegrees_diff_m1 = (errorPositionInDegrees_m1 - errorPositionInDegrees_prev_m1) / (deltaT / 1e6);
    float errorPositionInDegrees_diff_m2 = (errorPositionInDegrees_m2 - errorPositionInDegrees_prev_m2) / (deltaT / 1e6);

    // Update previous errors
    errorPositionInDegrees_prev_m1 = errorPositionInDegrees_m1;
    errorPositionInDegrees_prev_m2 = errorPositionInDegrees_m2;

    // PID control outputs
    float controlOutput_m1 = (Kp_m1 * errorPositionInDegrees_m1) + (Ki_m1 * errorPositionInDegrees_sum_m1) + (Kd_m1 * errorPositionInDegrees_diff_m1);
    float controlOutput_m2 = (Kp_m2 * errorPositionInDegrees_m2) + (Ki_m2 * errorPositionInDegrees_sum_m2) + (Kd_m2 * errorPositionInDegrees_diff_m2);

    // Apply minimum speed threshold for Motor 1
    if (abs(errorPositionInDegrees_m1) <= 1) {
      controlOutput_m1 = 0;
    }
    else {
      if (errorPositionInDegrees_m1 > 0 && controlOutput_m1 < minspeed) {
        controlOutput_m1 = minspeed;
      }
      else if (errorPositionInDegrees_m1 < 0 && controlOutput_m1 > -minspeed) {
        controlOutput_m1 = -minspeed;
      }
    }

    // Apply minimum speed threshold for Motor 2
    if (abs(errorPositionInDegrees_m2) <= 1) {
      controlOutput_m2 = 0;
    }
    else {
      if (errorPositionInDegrees_m2 > 0 && controlOutput_m2 < minspeed) {
        controlOutput_m2 = minspeed;
      }
      else if (errorPositionInDegrees_m2 < 0 && controlOutput_m2 > -minspeed) {
        controlOutput_m2 = -minspeed;
      }
    }

    // Set direction and determine target PWM for Motor 1
    int target_pwm_m1 = min(abs(controlOutput_m1), maxspeed);
    if (controlOutput_m1 < 0) {
      digitalWrite(motorPin1_m1, HIGH);
      digitalWrite(motorPin2_m1, LOW);
    }
    else {
      digitalWrite(motorPin1_m1, LOW);
      digitalWrite(motorPin2_m1, HIGH);
    }

    // Ramping PWM for Motor 1 (Only ramp up; immediate ramp-down)
    prev_pwm_m1 = rampPWM(prev_pwm_m1, target_pwm_m1);
    analogWrite(enablePin_m1, prev_pwm_m1);

    // Set direction and determine target PWM for Motor 2
    int target_pwm_m2 = min(abs(controlOutput_m2), maxspeed);
    if (controlOutput_m2 < 0) {
      digitalWrite(motorPin1_m2, HIGH);
      digitalWrite(motorPin2_m2, LOW);
    }
    else {
      digitalWrite(motorPin1_m2, LOW);
      digitalWrite(motorPin2_m2, HIGH);
    }

    // Ramping PWM for Motor 2 (Only ramp up; immediate ramp-down)
    prev_pwm_m2 = rampPWM(prev_pwm_m2, target_pwm_m2);
    analogWrite(enablePin_m2, prev_pwm_m2);

    // Read current and store in the array
    int currentValue = analogRead(current_sensor);
    currentReadings[currentIndex] = currentValue;
    currentIndex = (currentIndex + 1) % 10; // Wrap around after 10

    // Calculate average current value
    int currentSum = 0;
    for (int i = 0; i < 10; i++) {
      currentSum += currentReadings[i];
    }
    currentAverage = currentSum / 10.0;

    // Print the current position, demand, and average current value
    Serial.print("Motor 1 - Current: ");
    Serial.print(currentPositionInDegrees_m1);
    Serial.print("°, Demand: ");
    Serial.print(demandPositionInDegrees_m1);
    Serial.print(", motor control: ");
    Serial.print(controlOutput_m1);
    Serial.print("°, Motor 2 - Current: ");
    Serial.print(currentPositionInDegrees_m2);
    Serial.print("°, Demand: ");
    Serial.print(demandPositionInDegrees_m2);
    Serial.print(", motor control: ");
    Serial.print(controlOutput_m2);
    Serial.print(", Average Current Value: ");
    Serial.println(currentAverage);

    previousTime = currentTime;
  }
}

void updateEncoder_m1() {
  int aState_m1 = digitalRead(outputA_m1);
  if (aState_m1 != aLastState_m1) {
    if (digitalRead(outputB_m1) != aState_m1) {
      counter_m1++;
      spin_m1 = "Motor 1 is rotating clockwise";
    }
    else {
      counter_m1--;
      spin_m1 = "Motor 1 is rotating anticlockwise";
    }
  }
  aLastState_m1 = aState_m1;
}

void updateEncoder_m2() {
  int aState_m2 = digitalRead(outputA_m2);
  if (aState_m2 != aLastState_m2) {
    if (digitalRead(outputB_m2) != aState_m2) {
      counter_m2--;
      spin_m2 = "Motor 2 is rotating clockwise";
    }
    else {
      counter_m2++;
      spin_m2 = "Motor 2 is rotating anticlockwise";
    }
  }
  aLastState_m2 = aState_m2;
}
