#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

// Communication variables
const int byteSize = 5;
byte receivedBytes[byteSize];
byte sendBytes[4];
uint8_t motorSpeed[4] = {0};
bool motorDirection[4] = {0};
bool previousMotorDirection[4] = {0};
unsigned long previousMillisReceivedData = 0;

// SF = scaling factor
const double SF = 255.0/30.0;

// Constants
const double pi = 3.14159265359;
const int pulsesPerRevolution = 330 * 4;
const double radPerPulse = (2 * pi) / pulsesPerRevolution; 

// --- MOTOR TUNING CONSTANTS ---
// Minimum PWM required to get the wheels turning (overcome friction)
// If your robot hums but doesn't move, increase this slightly (e.g., to 45 or 50)
const int MIN_PWM = 40; 

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

// Define encoder pins
const int Encoder_Front_Left_A1 = 18;
const int Encoder_Front_Left_A2 = 36;
const int Encoder_Front_Right_A1 = 19;
const int Encoder_Front_Right_A2 = 34;
const int Encoder_Back_Left_A1 = 20;
const int Encoder_Back_Left_A2 = 32;
const int Encoder_Back_Right_A1 = 21;
const int Encoder_Back_Right_A2 = 30;

// Encoder objects
Encoder encoders[4] = {
    Encoder(Encoder_Front_Left_A1, Encoder_Front_Left_A2),
    Encoder(Encoder_Front_Right_A1, Encoder_Front_Right_A2),
    Encoder(Encoder_Back_Left_A1, Encoder_Back_Left_A2),
    Encoder(Encoder_Back_Right_A1, Encoder_Back_Right_A2)
};

// Encoder count storage
volatile long lastEncoderCounts[4] = {0};
volatile long encoderCounts[4] = {0};

// PID Constants 
// Lowered Kp slightly because Feedforward (MIN_PWM) now does the heavy lifting
double Kp = 25.0;    
double Ki = 15.0;     
double Kd = 5.0;    

// PID Variables
double Setpoint[4] = {0.0, 0.0, 0.0, 0.0}; 
double Input[4], Output[4], filteredInput[4] = {0.0};
double currentSpeed[4] = {0.0, 0.0, 0.0, 0.0};

PID pidControllers[4] = {
    PID(&Input[0], &Output[0], &Setpoint[0], Kp, Ki, Kd, DIRECT),
    PID(&Input[1], &Output[1], &Setpoint[1], Kp, Ki, Kd, DIRECT),
    PID(&Input[2], &Output[2], &Setpoint[2], Kp, Ki, Kd, DIRECT),
    PID(&Input[3], &Output[3], &Setpoint[3], Kp, Ki, Kd, DIRECT)
};

// Timing variables
unsigned long lastTime = 0;
const int updateInterval = 8; // Increased to 10ms to give encoders time to tick

// Interrupt Service Routine
void encoderISR0() { encoderCounts[0] = encoders[0].read(); }
void encoderISR1() { encoderCounts[1] = encoders[1].read(); }
void encoderISR2() { encoderCounts[2] = encoders[2].read(); }
void encoderISR3() { encoderCounts[3] = encoders[3].read(); }

void setup() {
    Serial.begin(115200);

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
    
    for (int i = 0; i < 4; i++) {
        pidControllers[i].SetMode(AUTOMATIC);
        pidControllers[i].SetControllerDirection(DIRECT);
        // Important: Set output limits to allow room for feedforward math
        pidControllers[i].SetOutputLimits(0, 255 - MIN_PWM); 
    }
    
    attachInterrupt(digitalPinToInterrupt(Encoder_Front_Left_A1), encoderISR0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_Front_Right_A1), encoderISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_Back_Left_A1), encoderISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_Back_Right_A1), encoderISR3, CHANGE);
}

void stopMotor(int motor){
    digitalWrite(DIR1_Pins[motor], LOW);
    digitalWrite(DIR2_Pins[motor], LOW);
    analogWrite(PWM_Pins[motor], 0);
}

void setMotorSpeed(int motor) {        
    // Deadzone: If target is practically zero, hard stop
    if (abs(Setpoint[motor]) < 0.1) { 
        stopMotor(motor);
        // Reset PID integral build up so it doesn't jump when we start again
        pidControllers[motor].SetMode(MANUAL);
        Output[motor] = 0;
        pidControllers[motor].SetMode(AUTOMATIC);
        return; 
    }

    pidControllers[motor].Compute();
    
    // FEEDFORWARD: Add the minimum power required to move the wheel
    // This overcomes the "Stiction" (Static Friction) immediately
    int pwmValue = abs(Output[motor]) + MIN_PWM; 
    
    // Safety clamp
    pwmValue = constrain(pwmValue, 0, 255);

    digitalWrite(DIR1_Pins[motor], motorDirection[motor]);
    digitalWrite(DIR2_Pins[motor], !motorDirection[motor]);
    analogWrite(PWM_Pins[motor], pwmValue);
}

uint8_t* generateSerialMsg(double* speeds){
    static uint8_t msg[5] = {};
    double feedbackSpeeds[4] = {};
    bool feedbackDirections[4] = {0,0,0,0};

    for(int i=0; i < 4; i++){
        feedbackSpeeds[i] = round(abs(speeds[i] * SF)); 
        if (feedbackSpeeds[i] < 256){
            msg[i+1] = uint8_t(feedbackSpeeds[i]);
        } 
        else{
            msg[i+1]=255;
        }
        if(speeds[i]<0){
            feedbackDirections[i]=0;
        }
        else{
            feedbackDirections[i]=1;
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

    for(int i=1; i<5; i++){
        Setpoint[i-1] = ((uint8_t)receivedBytes[i])/SF;
    }

    while (Serial.available() > 0){
        Serial.read();
    }
}

void loop() {
    if (Serial.available() >= byteSize) {
        previousMillisReceivedData = millis();
        digitalWrite(LED_BUILTIN,LOW);
        decodeSerialMsg();
        uint8_t* serialMsg = generateSerialMsg(currentSpeed);
        Serial.write(serialMsg,byteSize);
    }
    else{
        digitalWrite(LED_BUILTIN,HIGH);
    }

    // Safety timeout: Stop robot if no data for 200ms
    if(millis()-previousMillisReceivedData>=200){
        for(int i=0;i<4;i++){
            Setpoint[i] = 0;
        }
    }
    
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= updateInterval) {
        double deltaTime = (currentTime - lastTime) / 1000.0;
        for (int i = 0; i < 4; i++) {
            long deltaCounts = -(encoderCounts[i] - lastEncoderCounts[i]);
            Input[i] = (deltaCounts / deltaTime) * radPerPulse;

            lastEncoderCounts[i] = encoderCounts[i];
            
            // Filter the input slightly to remove noise
            filteredInput[i] = filteredInput[i] + ((Input[i] - filteredInput[i]) / 2.0); // Reduced filter strength
            currentSpeed[i]=filteredInput[i];

            // PID uses absolute speed, direction handled by Serial logic
            Input[i] = abs(filteredInput[i]);
            setMotorSpeed(i);
        }
        lastTime = currentTime;
    }
}