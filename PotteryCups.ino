#include <Arduino.h>
#include <VarSpeedServo.h>  // Include the VarSpeedServo library
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

int servoAngle = 0;

// Keypad setup
const byte ROWS = 4; // Number of rows
const byte COLS = 4; // Number of columns
char keys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {37, 35, 33, 31};    // Connect to the row pins
byte colPins[COLS] = {36, 34, 32, 30};    // Connect to the column pins

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2); // Address 0x27, 16 columns, 2 rows

/************************************/

const int motorEnablePin = 44; // PWM pin to control motor speed
const int motorIn1Pin = 43;    // Motor control pin 1 for direction
const int motorIn2Pin = 42;    // Motor control pin 2 for direction
const int limitSwitchPin = 16; // Limit switch pin (NC configuration)

// Variables for motor control
bool motorDirection = true;      // true = forward, false = backward
unsigned long lastToggleTime = 0; // Tracks time after direction toggle
bool motorStopped = false;       // Indicates if the motor is stopped
bool moveAgain = false;          // Flag to trigger additional motor movement
unsigned long moveAgainStart = 0; // Tracks start time of additional movement

// Variables for servo control
VarSpeedServo myServo1;          // Create a VarSpeedServo object for the servo
bool servoActive = false;        // Indicates if the servo operation has completed
unsigned long motorStopTime = 0; // T

VarSpeedServo myServo2;  // Create a VarSpeedServo object for the servo drowing

const int TrigPin = 17;   // Trigger pin
const int EchoPin = 19;   // Echo pin (interrupt capable)

// LDR and laser setup
const int ldrPin = A5;    // Analog pin connected to the LDR
const int ldrThreshold = 200; // Threshold for detecting laser interruption

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

// Stepper motor pins (Motor1, Motor2, Motor3)
const int Motor1_EN = 2;
const int Motor1_DIR = 3;
const int Motor1_PUL = 4;

const int Motor2_EN = 5;
const int Motor2_DIR = 6;
const int Motor2_PUL = 7;

const int Motor3_EN = 8;  // Motor3 pins
const int Motor3_DIR = 9;
const int Motor3_PUL = 10;

// Motor Specifications
const int stepDelay1 = 200;  // Delay between steps for the leadscrew motor (in microseconds)
const int stepDelay2 = 200;  // Delay between steps for the second motor (in microseconds)

// Timing variables
long lastStepTime1 = 0; // Tracks time for the first motor's steps
long lastStepTime2 = 0; // Tracks time for the second motor's steps
unsigned long startTime2 = 0;  // Start time for the time limit

// Time limit in milliseconds
const unsigned long motorRunTime = 11000;  // 9 seconds (in milliseconds)

// Push button pins
const int Button1Pin = 11; // Button to toggle direction 1
const int Button2Pin = 12; // Button to toggle direction 2

// Relay pin 
const int RelayPin = 13;   // Relay control pin DRILL
const int relay2=46; //Led strip


volatile bool stopMotorsFlag = false; // Flag to indicate motor stop
volatile unsigned long startTime = 0;  // Store time for echo pulse
volatile long duration = 0;            // Duration of the echo pulse
volatile long distance = 0;            // Calculated distance

// bool motor1Direction = HIGH;  // Motor1 direction state
// bool motor2Direction = HIGH; // Motor2 direction state
// bool motor3Direction = HIGH; // Motor3 direction state

bool processRunning = false; 
bool laserDetected = false; 
bool motorDirection1 = HIGH;  // Motor direction state (LOW = one direction, HIGH = other direction)

// const unsigned long stopDelay = 4000; // 4 seconds delay
// unsigned long button1PressTime = 0;   // Variable to store the time Button1 is pressed
bool button1Pressed = false; 

void setup() {
  Serial.begin(9600);

    // Initialize the LCD
    lcd.begin();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Starting System");
    lcd.setCursor(0, 1);
    lcd.print("Press a Key...");


  myServo2.attach(A7);  // Attach the servo to pin A7 drowing

  // Ultrasonic setup
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  // LDR setup
  pinMode(ldrPin, INPUT);

  // Stepper motor setup (Motor1, Motor2, Motor3)
  pinMode(Motor1_EN, OUTPUT);
  pinMode(Motor1_DIR, OUTPUT);
  pinMode(Motor1_PUL, OUTPUT);

  pinMode(Motor2_EN, OUTPUT);
  pinMode(Motor2_DIR, OUTPUT);
  pinMode(Motor2_PUL, OUTPUT);

  pinMode(Motor3_EN, OUTPUT);
  pinMode(Motor3_DIR, OUTPUT);
  pinMode(Motor3_PUL, OUTPUT);

  // Push button setup
  pinMode(Button1Pin, INPUT_PULLUP);  // Button 1 (activated on LOW)
  pinMode(Button2Pin, INPUT_PULLUP);  // Button 2 (activated on LOW)

  // Relay setup
  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, LOW); // Ensure relay is off initially

  // Initially disable all motors (set EN to HIGH)
  digitalWrite(Motor1_EN, HIGH);  // Motor1 initially OFF
  digitalWrite(Motor2_EN, HIGH);  // Motor2 initially OFF
  digitalWrite(Motor3_EN, HIGH);  // Motor3 initially OFF

  // Attach interrupt for EchoPin (on both rising and falling edge)
  attachInterrupt(digitalPinToInterrupt(EchoPin), handleEcho, CHANGE);

  // Trigger ultrasonic sensor
  digitalWrite(TrigPin, LOW);  // Ensure trigger is LOW initially


  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorIn1Pin, OUTPUT);
  pinMode(motorIn2Pin, OUTPUT);
  pinMode(limitSwitchPin, INPUT_PULLUP);


  pinMode(relay2, OUTPUT); // Set the relay pin as an output
digitalWrite(relay2, HIGH); 


  // Initialize servo
  myServo1.attach(A6);  // Attach the servo to pin A6
  Serial.begin(9600);   // Start serial communication for debugging
  myServo1.write(85);  // Set the servo to 115° position on startup
  Serial.println("DC Motor and Servo Control Initialized");

  // Set initial direction for both motors
  digitalWrite(Motor2_DIR, HIGH); // Adjust direction as needed (HIGH = up, LOW = down)
  digitalWrite(Motor3_DIR, HIGH); // Adjust direction as needed

  // Move servo to initial position
  myServo2.write(120, 80, true);
}

void controlMotor(bool direction, int speed) {
  if (direction) {
    // Rotate motor forward
    digitalWrite(motorIn1Pin, HIGH);
    digitalWrite(motorIn2Pin, LOW);
  } else {
    // Rotate motor backward
    digitalWrite(motorIn1Pin, LOW);
    digitalWrite(motorIn2Pin, HIGH);
  }
  analogWrite(motorEnablePin, speed); // Set motor speed (0 to 255)
}

// Function to check the limit switch and toggle motor direction
void checkLimitSwitch() {
  static bool lastSwitchState = LOW; // Default state for NC switch is LOW
  bool currentSwitchState = digitalRead(limitSwitchPin);

  // Detect rising edge (LOW to HIGH transition for NC switch)
  if (lastSwitchState == LOW && currentSwitchState == HIGH) {
    motorDirection = !motorDirection; // Toggle motor direction
    lastToggleTime = millis();        // Record the time of toggle
    motorStopped = false;             // Reset motor stop flag
    servoActive = false;              // Reset servo operation flag
    delay(200);                       // Debounce delay
  }

  lastSwitchState = currentSwitchState; // Update last state
}

// Function to control the servo movement
void controlServo() {
  static unsigned long servoStartTime = 0;
  static int servoState = 0; // 0 = idle, 1 = moving to 0°, 2 = waiting, 3 = moving to 115°

  if (servoActive) return;

  switch (servoState) {
    case 0: // Idle, start moving to 0°
      myServo1.write(0, 80, true);
      servoStartTime = millis();
      servoState = 1;
      break;

    case 1: // Waiting for 5 seconds after moving to 0°
      if (millis() - servoStartTime >= 5000) {
        myServo1.write(85, 80, true);
        servoStartTime = millis();
        servoState = 3;
      }
      break;

    case 3: // Waiting for 3 seconds after moving to 115°
      if (millis() - servoStartTime >= 3000) {
        servoActive = true;
        moveAgain = true;
        moveAgainStart = millis();
        servoState = 0; // Reset to idle
      }
      break;
  }
}
void controlServo1() {
      myServo1.write(0, 80, true);
}


void loop() {
    static int piecesCount = 0;      // Number of pieces to process
    static bool processEnabled = false; // Flag to indicate if the process can run

    // Poll the keypad
    char key = keypad.getKey();

    if (key) {
        Serial.println(key);  // Debug: Print key to Serial Monitor
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Key Pressed:");
        lcd.setCursor(0, 1);
        lcd.print(key);

        // Handle key press
        handleKeyPress(key, piecesCount, processEnabled, servoAngle);
    }

    // Process logic if enabled
    if (processEnabled && piecesCount && servoAngle> 0) {
        int ldrValue = analogRead(ldrPin); // Read LDR value
        Serial.print("LDR Value: ");
        Serial.println(ldrValue);

        if (ldrValue >= ldrThreshold && !processRunning) {
            // Laser beam detected
            processRunning = true;
            Serial.println("Laser detected! Starting process...");
            startProcess(piecesCount,servoAngle);
            processRunning = false; // Mark process as completed
        } else {
            Serial.println("Waiting for laser beam...");
            // Perform other background operations
            checkLimitSwitch();
            handleMotorControl();
        }
    }
}

void startProcess(int piecesCount ,int servoAngle) {

    Serial.print("Starting process for ");
    Serial.print(piecesCount);
    Serial.println(" repetitions...");

    for (int i = 0; i < piecesCount; i++) {
        Serial.print("Process iteration: ");
        Serial.println(i + 1);

        bool processActive = true; // Control the loop
        bool button1Pressed = false; // Track Button1 press

        while (processActive) {
            // Send ultrasonic pulse
            digitalWrite(TrigPin, HIGH);
            delayMicroseconds(40);  // Send 10ms pulse to trigger ultrasonic sensor
            digitalWrite(TrigPin, LOW);

            // Check if Button1 is pressed
            if (digitalRead(Button1Pin) == LOW) {
                toggleMotorDirectionForOneSecond(servoAngle);
                delay(200); // Debounce delay for the button
                button1Pressed = true; // Mark Button1 as pressed
                Serial.println("Button1 pressed, exiting loop...");
                processActive = false; // Stop the loop
            }

            // Check if Button2 is pressed
            if (digitalRead(Button2Pin) == LOW) {
                toggleMotorDirection();
                delay(200); // Debounce delay for the button
                Serial.println("Button2 pressed, toggling motor direction.");
            }

            // Motor behavior based on `stopMotorsFlag`
            if (!stopMotorsFlag) {
                Serial.println("Motors running forward...");
                moveForward();
            } else {
                Serial.println("Stop flag is true. Executing else block...");
                // Servo movement
                myServo1.write(85, 80, true);
               // myServo1.detach();
                delay(1000); // Allow servo to reach position

                // Motor control
                controlMotor(false, 255); // Run motor forward at full speed
                delay(200);
                analogWrite(motorEnablePin, 0); // Stop motor
                myServo1.detach();
                Serial.println("Servo and motor returned to initial states.");
                processActive = false; // Stop the loop after completing actions
            }
        }

        // Print completion message for this iteration
        Serial.print("Iteration ");
        Serial.print(i + 1);
        Serial.println(" completed.");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(i+" cup completed.");

        // Add a small delay between iterations (optional)
        delay(1000); // 1 second delay
    }

    Serial.println("All process repetitions completed.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("All cups completed.");
    toggleLED(5000);

}

void toggleLED(int duration1) {
  unsigned long startTime2 = millis(); // Get the start time
  while (millis() - startTime2 < duration1) { // Run for the given duration
    digitalWrite(relay2, LOW); // Turn on the LED by activating the relay
    delay(500); // Wait for 0.5 second
    digitalWrite(relay2, HIGH); // Turn off the LED by deactivating the relay
    delay(500); // Wait for 0.5 second
  }
}

void sendUltrasonicPulse() {
  // Periodically send trigger pulse to ultrasonic sensor
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(40);  // Send 10ms pulse to trigger ultrasonic sensor
  digitalWrite(TrigPin, LOW);
}

void checkButtons() {
  // Check if Button1 is pressed (toggle for 1 second)
  if (digitalRead(Button1Pin) == LOW && (millis() - lastDebounceTime) > debounceDelay) {
    toggleMotorDirectionForOneSecond(servoAngle);
    lastDebounceTime = millis();  // Debounce delay for the button
  }

  // Check if Button2 is pressed (toggle direction normally)
  if (digitalRead(Button2Pin) == LOW && (millis() - lastDebounceTime) > debounceDelay ) {
    toggleMotorDirection();
    lastDebounceTime = millis();  // Debounce delay for the button
  }
}

void moveForward() {
  // Enable Motor1 before moving
  digitalWrite(Motor1_EN, LOW);

  // Stepper motor forward with current direction
  digitalWrite(Motor1_DIR, motorDirection1);

  // Stepper motor pulses
  for (int i = 0; i < 1000; i++) {
    if (stopMotorsFlag) return;
    digitalWrite(Motor1_PUL, HIGH);
    delayMicroseconds(400);
    digitalWrite(Motor1_PUL, LOW);
    delayMicroseconds(400);
  }

  // Disable Motor1 after moving
  digitalWrite(Motor1_EN, HIGH);
}

void stopMotor1() {
  Serial.println("Stopping motor...");

  // Disable stepper motor
  digitalWrite(Motor1_EN, HIGH);
}

void toggleMotorDirection() {
  // Toggle motor direction on each button press
  delay(500);
  motorDirection1 = LOW;
  Serial.print("Motor direction: ");
  Serial.println(motorDirection1 == LOW ? "Forward" : "Backward");
  digitalWrite(RelayPin, LOW); 
  
}

void toggleMotorDirectionForOneSecond(int servoAngle) {
  // Toggle motor direction for 1 second
  motorDirection1 = !motorDirection1;
  Serial.print("Motor direction changed to: ");
  Serial.println(motorDirection1 == LOW ? "Forward" : "Backward");

  // Run motor for 1 second
  unsigned long toggleStartTime = millis();
  while (millis() - toggleStartTime < 1000) {
    moveForward(); // Run motor for 1 second
  }

  // Stop motor after 1 second
  stopMotor1();

  myServo2.write(servoAngle, 80, true);  // Move servo to final position
  Serial.print("Servo moved to angle: ");
  Serial.println(servoAngle);

  // Start the timer
  runMotors();
  stopMotorsFlag=true;

   // Move servo to initial position
  myServo2.write(120, 80, true);

delay(10000);
toggle2Motors();

}

void handleEcho() {
  // Capture time of echo pulse change (either rising or falling edge)
  if (digitalRead(EchoPin) == HIGH) {
    startTime = micros();  // Start timing on rising edge
  } else {
    // Calculate duration and distance on falling edge
    duration = micros() - startTime;
    distance = duration * 0.034 / 2;  // Convert duration to distance in cm

    // Print the distance for debugging
    Serial.print("Distance: ");
    Serial.println(distance);

    // If distance is greater than 10 cm, turn on relay
    if (distance > 10 && motorDirection1 == HIGH) {
      digitalWrite(RelayPin, HIGH);  // Turn relay on
      Serial.println("Relay ON");
    } else {
      digitalWrite(RelayPin, LOW);   // Turn relay off
      Serial.println("Relay OFF");
    }
  }
}

// Function to control motors continuously for a specified duration
void runMotors() {
  // Enable motors
  digitalWrite(Motor2_EN, LOW);
  digitalWrite(Motor3_EN, LOW);

  startTime2 = millis(); 
  // Run motors until the time limit
  while (millis() - startTime2 <= motorRunTime) {
    moveMotorContinuously(Motor2_PUL, stepDelay1, lastStepTime1);
    moveMotorContinuously(Motor3_PUL, stepDelay2, lastStepTime2);
  }
  // Stop motors after time limit
  stopMotors();
 // stopMotorsFlag=true;
  Serial.println("Motors stopped after running.");
}

// Function to control motors continuously for a specified duration
void toggle2Motors() {
  // Enable motors
  digitalWrite(Motor2_EN, LOW);
  digitalWrite(Motor2_DIR,LOW) ;

  startTime2 = millis(); 
  // Run motors until the time limit
  while (millis() - startTime2 <= motorRunTime) {
    moveMotorContinuously(Motor2_PUL, stepDelay1, lastStepTime1);
    //moveMotorContinuously(Motor3_PUL, stepDelay2, lastStepTime2);
  }
  // Stop motors after time limit
  stopMotors();
 // stopMotorsFlag=true;
  Serial.println("Motors stopped after running.");
}

// Function to stop both motors
void stopMotors() {
  // Disable both motors (set EN pin HIGH)
  digitalWrite(Motor2_EN, HIGH);
  digitalWrite(Motor3_EN, HIGH);

  // Stop motor movement by setting STEP pins LOW
  digitalWrite(Motor2_PUL, LOW);
  digitalWrite(Motor3_PUL, LOW);
}

// Function to move a motor continuously at a specified step delay
void moveMotorContinuously(int stepPin, int stepDelay, long &lastStepTime) {
  // Check if it's time for the next step
  if (micros() - lastStepTime >= stepDelay) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1); // Pulse width
    digitalWrite(stepPin, LOW);

    // Update the last step time
    lastStepTime = micros();
  }
}

void handleKeyPress(char key, int &piecesCount, bool &processEnabled , int &servoAngle) {
    static bool systemOn = true; // System status flag

    if (!systemOn && key != 'C') {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("System OFF");
        return; // Only 'C' can turn the system back on
    }

    switch (key) {
        case 'A': // Set number of repetitions
             lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Enter 1-9:");
            while (true) {
                char numKey = keypad.getKey();
                if (numKey >= '1' && numKey <= '9') {
                    piecesCount = numKey - '0'; // Convert char to int
                    processEnabled = true;     // Enable the process
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Pieces: ");
                    lcd.print(piecesCount);
                    Serial.print("Pieces set to: ");
                    Serial.println(piecesCount);
                    delay(2000); // Allow user to read the message

                    // Automatically move to case 'B' (enter servo shape)
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Servo: 1=SH1 2=SH2");
                    while (true) {
                        char servoKey = keypad.getKey();
                        if (servoKey == '1') {
                            servoAngle = 88;
                          //  myServo2.write(85, 80, true); // Move servo2 to 85 degrees
                            lcd.clear();
                            lcd.setCursor(0, 0);
                            lcd.print("Servo to SH1");
                            Serial.println("Servo2 set to 88 degrees.");
                            delay(2000); // Allow user to read the message
                            break;
                        } else if (servoKey == '2') {
                            servoAngle = 158;
                           // myServo2.write(160, 80, true); // Move servo2 to 160 degrees
                            lcd.clear();
                            lcd.setCursor(0, 0);
                            lcd.print("Servo to SH2");
                            Serial.println("Servo2 set to 158 degrees.");
                            delay(2000); // Allow user to read the message
                            break;
                        }
                    }
                    break;
                }
            }
            break;
        
        case 'C': // Turn off the system
            processEnabled = false;
            systemOn = false;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("System OFF");
            Serial.println("System turned OFF.");
            break;

        default: // Invalid keys
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Invalid Key");
            break;
    }
}
void handleMotorControl() { ////DC
    if (!motorDirection && !motorStopped && millis() - lastToggleTime >= 300) {
        analogWrite(motorEnablePin, 0); // Stop the motor
        motorStopped = true;           // Mark motor as stopped
        motorStopTime = millis();      // Record stop time
    } else if (!motorStopped) {
        controlMotor(motorDirection, 255); // Continue motor in current direction
    }

    // Trigger servo 1 second after motor stops
    if (motorStopped && millis() - motorStopTime >= 1000) {
        controlServo1();
    }

    if (moveAgain) {
        if (millis() - moveAgainStart <= 700) {
            controlMotor(false, 255); // Run motor forward
        } else {
            analogWrite(motorEnablePin, 0); // Stop motor after 40 ms
            moveAgain = false;             // Reset movement flag
        }
    }
}

