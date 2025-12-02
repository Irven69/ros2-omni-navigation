#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

// ==================== TUNABLE PARAMETERS ====================
double Kp = 25.0;
double Ki = 5.0;
double Kd = 5.0;
int min_pwm[4] = {30, 30, 30, 30}; // [FL, FR, BL, BR]
// ============================================================

// Communication variables (FROM motorControllerClient)
const int byteSize = 5;
byte receivedBytes[byteSize];
uint8_t motorSpeed[4] = {0};
bool motorDirection[4] = {0};
unsigned long previousMillisReceivedData = 0;

// Scaling factor (FROM motorControllerClient)
const double SF = 255.0/30.0;

// Constants (FROM motorControllerClient)
const double pi = 3.14159265359;
const int pulsesPerRevolution = 330 * 4;
const double radPerPulse = (2 * pi) / pulsesPerRevolution;

// Motor pins (FROM motorControllerClient - EXACT COPY)
const int PWM_Front_Left    = 5;
const int DIR1_Front_Left   = 52;
const int DIR2_Front_Left   = 50;

const int PWM_Front_Right   = 4;
const int DIR1_Front_Right  = 48;
const int DIR2_Front_Right  = 46;

const int PWM_Back_Left     = 3;
const int DIR1_Back_Left    = 42;
const int DIR2_Back_Left    = 44;

const int PWM_Back_Right    = 2;
const int DIR1_Back_Right   = 40;
const int DIR2_Back_Right   = 38;

const int PWM_Pins[4] = {PWM_Front_Left, PWM_Front_Right, PWM_Back_Left, PWM_Back_Right};
const int DIR1_Pins[4] = {DIR1_Front_Left, DIR1_Front_Right, DIR1_Back_Left, DIR1_Back_Right};
const int DIR2_Pins[4] = {DIR2_Front_Left, DIR2_Front_Right, DIR2_Back_Left, DIR2_Back_Right};

// Encoder pins (FROM motorControllerClient - EXACT COPY)
const int Encoder_Front_Left_A1 = 18;
const int Encoder_Front_Left_A2 = 36;
const int Encoder_Front_Right_A1 = 19;
const int Encoder_Front_Right_A2 = 34;
const int Encoder_Back_Left_A1 = 20;
const int Encoder_Back_Left_A2 = 32;
const int Encoder_Back_Right_A1 = 21;
const int Encoder_Back_Right_A2 = 30;

// Encoder objects (FROM motorControllerClient - EXACT COPY)
Encoder encoders[4] = {
    Encoder(Encoder_Front_Left_A1, Encoder_Front_Left_A2),
    Encoder(Encoder_Front_Right_A1, Encoder_Front_Right_A2),
    Encoder(Encoder_Back_Left_A1, Encoder_Back_Left_A2),
    Encoder(Encoder_Back_Right_A1, Encoder_Back_Right_A2)
};

// Encoder count storage (FROM motorControllerClient)
volatile long lastEncoderCounts[4] = {0};
volatile long encoderCounts[4] = {0};

// PID Variables (NEW - Added for PID)
double Setpoint[4] = {0.0, 0.0, 0.0, 0.0};
double Input[4] = {0.0, 0.0, 0.0, 0.0};
double Output[4] = {0.0, 0.0, 0.0, 0.0};
double filteredInput[4] = {0.0, 0.0, 0.0, 0.0};
double currentSpeed[4] = {0.0, 0.0, 0.0, 0.0};

// PID Controllers (NEW)
PID pidControllers[4] = {
    PID(&Input[0], &Output[0], &Setpoint[0], Kp, Ki, Kd, DIRECT),
    PID(&Input[1], &Output[1], &Setpoint[1], Kp, Ki, Kd, DIRECT),
    PID(&Input[2], &Output[2], &Setpoint[2], Kp, Ki, Kd, DIRECT),
    PID(&Input[3], &Output[3], &Setpoint[3], Kp, Ki, Kd, DIRECT)
};

// Timing variables (FROM motorControllerClient)
unsigned long lastTime = 0;
const int updateInterval = 5;

// Interrupt Service Routines (FROM motorControllerClient - EXACT COPY)
void encoderISR0() { encoderCounts[0] = encoders[0].read(); }
void encoderISR1() { encoderCounts[1] = encoders[1].read(); }
void encoderISR2() { encoderCounts[2] = encoders[2].read(); }
void encoderISR3() { encoderCounts[3] = encoders[3].read(); }

void setup() {
    Serial.begin(115200);

    // Pin setup (FROM motorControllerClient - EXACT COPY)
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PWM_Front_Left, OUTPUT);
    pinMode(PWM_Front_Right, OUTPUT);
    pinMode(PWM_Back_Left, OUTPUT);
    pinMode(PWM_Back_Right, OUTPUT);
    
    pinMode(DIR1_Front_Left, OUTPUT);
    pinMode(DIR2_Front_Left, OUTPUT);
    pinMode(DIR1_Front_Right, OUTPUT);
    pinMode(DIR2_Front_Right, OUTPUT);
    pinMode(DIR1_Back_Left, OUTPUT);
    pinMode(DIR2_Back_Left, OUTPUT);
    pinMode(DIR1_Back_Right, OUTPUT);
    pinMode(DIR2_Back_Right, OUTPUT);
    
    // PID setup (NEW)
    for (int i = 0; i < 4; i++) {
        pidControllers[i].SetMode(AUTOMATIC);
        pidControllers[i].SetControllerDirection(DIRECT);
        pidControllers[i].SetOutputLimits(0, 255 - min_pwm[i]);
    }
    
    // Interrupts (FROM motorControllerClient - EXACT COPY)
    attachInterrupt(digitalPinToInterrupt(Encoder_Front_Left_A1), encoderISR0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_Front_Right_A1), encoderISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_Back_Left_A1), encoderISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_Back_Right_A1), encoderISR3, CHANGE);

    Serial.println("=== PID Motor Controller ===");
    Serial.print("Kp="); Serial.print(Kp);
    Serial.print(" Ki="); Serial.print(Ki);
    Serial.print(" Kd="); Serial.println(Kd);
}

void stopMotor(int motor) {
    digitalWrite(DIR1_Pins[motor], LOW);
    digitalWrite(DIR2_Pins[motor], LOW);
    analogWrite(PWM_Pins[motor], 0);
}

void setMotorSpeed(int motor) {
    if (abs(Setpoint[motor]) < 0.1) {
        stopMotor(motor);
        pidControllers[motor].SetMode(MANUAL);
        Output[motor] = 0;
        pidControllers[motor].SetMode(AUTOMATIC);
        return;
    }

    pidControllers[motor].Compute();
    
    int pwmValue = abs(Output[motor]) + min_pwm[motor];
    pwmValue = constrain(pwmValue, 0, 255);

    digitalWrite(DIR1_Pins[motor], motorDirection[motor]);
    digitalWrite(DIR2_Pins[motor], !motorDirection[motor]);
    analogWrite(PWM_Pins[motor], pwmValue);
}

uint8_t* generateSerialMsg(double* speeds) {
    static uint8_t msg[5] = {};
    double feedbackSpeeds[4] = {};
    bool feedbackDirections[4] = {0, 0, 0, 0};

    for (int i = 0; i < 4; i++) {
        feedbackSpeeds[i] = round(abs(speeds[i] * SF));
        if (feedbackSpeeds[i] < 256) {
            msg[i + 1] = uint8_t(feedbackSpeeds[i]);
        } else {
            msg[i + 1] = 255;
        }
        if (speeds[i] < 0) {
            feedbackDirections[i] = 0;
        } else {
            feedbackDirections[i] = 1;
        }
    }

    msg[0] = 0;
    if (feedbackDirections[0]) msg[0] |= (1 << 0);
    if (feedbackDirections[1]) msg[0] |= (1 << 1);
    if (feedbackDirections[2]) msg[0] |= (1 << 2);
    if (feedbackDirections[3]) msg[0] |= (1 << 3);

    return msg;
}

void decodeSerialMsg() {
    Serial.readBytes(receivedBytes, byteSize);

    byte direction_byte = receivedBytes[0];
    motorDirection[0] = (direction_byte & 0b00000001) != 0;
    motorDirection[1] = (direction_byte & 0b00000010) != 0;
    motorDirection[2] = (direction_byte & 0b00000100) != 0;
    motorDirection[3] = (direction_byte & 0b00001000) != 0;

    for (int i = 1; i < 5; i++) {
        Setpoint[i - 1] = ((uint8_t)receivedBytes[i]) / SF;
    }

    while (Serial.available() > 0) {
        Serial.read();
    }
}

void loop() {
    // Serial communication (FROM motorControllerClient - EXACT COPY)
    if (Serial.available() >= byteSize) {
        previousMillisReceivedData = millis();
        digitalWrite(LED_BUILTIN, LOW);
        decodeSerialMsg();
        uint8_t* serialMsg = generateSerialMsg(currentSpeed);
        Serial.write(serialMsg, byteSize);
    } else {
        digitalWrite(LED_BUILTIN, HIGH);
    }

    // Safety timeout (FROM motorControllerClient)
    if (millis() - previousMillisReceivedData >= 200) {
        for (int i = 0; i < 4; i++) {
            Setpoint[i] = 0;
        }
    }

    // Speed calculation and PID update (FROM motorControllerClient + PID)
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= updateInterval) {
        double deltaTime = (currentTime - lastTime) / 1000.0;
        
        for (int i = 0; i < 4; i++) {
            // Calculate speed from encoder
            long deltaCounts = -(encoderCounts[i] - lastEncoderCounts[i]);
            Input[i] = (deltaCounts / deltaTime) * radPerPulse;
            lastEncoderCounts[i] = encoderCounts[i];
            
            // Filter the input
            filteredInput[i] = filteredInput[i] + ((Input[i] - filteredInput[i]) / 2.0);
            currentSpeed[i] = filteredInput[i];

            // PID uses absolute speed
            Input[i] = abs(filteredInput[i]);
            
            // Apply motor control with PID
            setMotorSpeed(i);
        }
        
        lastTime = currentTime;
    }
}