#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>

#define MPU6050_REG_GYRO_XOUT_H 0x43

Servo myServo;

MPU6050 mpu(Wire);
unsigned long timer = 0;

const int thrustFanPositivePin = 18;  // P3 - Positive terminal for fan 1
const int thrustFanNegativePin = 7;   // PD5 - Negative terminal for fan 1

const int liftFanPositivePin = 11;  // P11 - Positive terminal for fan 2
const int liftFanNegativePin = 9;   // PB1 - Negative terminal for fan 2

void trigger_HC_SR04() {

    //message for debugging purposes
    //uart_tx_str("Triggering HC-SR04...\n");

    //set TRIG_PIN (PB3) to high
    PORTB |= (1 << PB3);

    // Wait for a short duration (10 microseconds minimum)
    //the datasheet confirms 10us
    _delay_us(12);

    // Set TRIG_PIN (PB3) low
    PORTB &= ~(1 << PB3);

    _delay_us(10);
}




volatile struct {
    unsigned long hello = millis();
    unsigned long world = millis();
    unsigned long print = millis();
    // ...              <------- add more lines as needed
} PREVIOUS_TIMES;

int secondsstuck = 0;
int leftdistance = 0;
int rightdistance = 0;
int saveangle = 0;
int onlyonce = 0;
int state2 = 0;
int relativedeg = 0;
int notturnedyet = 0;
int atleastonce = 0;
int state3 = 0;

int world(int distance, int state, double gyroZ_deg) {
    // Points/Refers to the previous time assigned to "world" (PREVIOUS_TIMES.world)
    volatile unsigned long *previous_time = &PREVIOUS_TIMES.world;

    // Set your delay
    unsigned long delay = 1000;

    // Check if the delay has been reached
    if (millis() >= *previous_time + delay) {
        if(state3 == 1){
            state3 = 0;
        }
        
        Serial.println(distance);  //distances readings can be weird when the battery isn't used... turn the switch ON
        Serial.println(state);
        Serial.println(gyroZ_deg);
        // Place your code here (Critical section)
        if (atleastonce == 1) {
            secondsstuck++;
        }

        if (secondsstuck > 15) {
            analogWrite(liftFanPositivePin, 0);       // Set PWM duty cycle (0-255) for speed control
            digitalWrite(liftFanNegativePin, LOW);    // Turn on the negative terminal for fan 1
            analogWrite(thrustFanPositivePin, 0);     // Set PWM duty cycle (0-255) for speed control (default 250)
            digitalWrite(thrustFanNegativePin, LOW);  // Turn on the negative terminal for fan 1
            if (onlyonce == 0) {
                saveangle = 90 - gyroZ_deg + relativedeg;
                onlyonce = 1;
            }
            if (state2 == 0) {
                myServo.write(180);
                state2 == 1;
            } else if (state2 == 1) {
                rightdistance = distance;
                state2 = 2;
            } else if (state2 == 2) {
                myServo.write(0);
                state = 3;
            } else if (state2 == 3) {
                leftdistance = distance;
                state2 = 4;
            } else if (state2 == 4) {
                if (leftdistance > rightdistance) {
                    myServo.write(30);
                    state2 = 5;
                } else {
                    myServo.write(150);
                    state2 = 5;
                }
            } else if (state2 == 5) {
                myServo.write(saveangle);
                onlyonce = 0;
                secondsstuck = 0;
                state2 = 0;
            }
            return state;
        }

        if (distance < 40 && notturnedyet == 0) {

            // stop lift fan and thrust fan
            if (state == 0) {  //state 0: hovercraft not in the middle of turning
                state = 1;     //state 1: turn the hovercraft 180

                myServo.write(180);
            } else if (state == 1) {  //state 1: hovercraft turned once
                state = 2;            //state 2: turn the hovercraft 0
                myServo.write(0);
            } else if (state == 2) {  //state 1: hovercraft turned twice
                state = 0;            //state 2: turn the hovercraft 90
                myServo.write(90);
            }
        } else {
            analogWrite(liftFanPositivePin, 250);  // Set PWM duty cycle (0-255) for speed control
            digitalWrite(liftFanNegativePin, HIGH);
            analogWrite(thrustFanPositivePin, 250);    // Set PWM duty cycle (0-255) for speed control (default 250)
            digitalWrite(thrustFanNegativePin, HIGH);  // Turn on the negative terminal for fan 1


            if (state == 1) {
                notturnedyet = 1;
                //analogWrite(thrustFanPositivePin, 250);    // Set PWM duty cycle (0-255) for speed control (default 250)
                //digitalWrite(thrustFanNegativePin, HIGH);  // Turn on the negative terminal for fan 1

                //power thrust fan until gyroZ_deg == 90 (+- 5 degrees~) (or -90 I forgot if IMU reads clockwise or counter)
                //when that happens, signal that on the next cycle (or next 2 cycles depending on how long it would take to make the 90 degree turn), turn the servo back to 90 and state = 0

            } else if (state == 2) {
                notturnedyet = 1;
                //analogWrite(thrustFanPositivePin, 250);    // Set PWM duty cycle (0-255) for speed control (default 250)
                //digitalWrite(thrustFanNegativePin, HIGH);  // Turn on the negative terminal for fan 1

                //power thrust fan until gyroZ_deg == 90 (+- 5 degrees~) (or -90 I forgot if IMU reads clockwise or counter)
                //when that happens, signal that on the next cycle (or next 2 cycles depending on how long it would take to make the 90 degree turn), turn the servo back to 90 and state = 0

            } else if (state == 0) {



                //power lift fan and thrust fan
            }
        }

        // Update PREVIOUS_TIMES.world to current_time
        *previous_time = millis();
    }

    // Return something if needed
    return state;
}

int state = 0;

double print(double gyroZ_deg) {
    // Points/Refers to the previous time assigned to "print" (PREVIOUS_TIMES.print)
    volatile unsigned long *previous_time = &PREVIOUS_TIMES.print;

    // Set your period
    unsigned long period = 100;  // ms

    // Initiallize the next time period this task should be executed
    unsigned long next_time = *previous_time + period;

    // Check if the next period has begun
    if (millis() >= next_time) {

        // Place your code here (Critical section)

        if (gyroZ_deg > 80 + relativedeg || gyroZ_deg < -80 + relativedeg) {  // might want to adjust the numbers on 90 degrees
            myServo.write(90);
            notturnedyet = 0;
            state = 0;
            relativedeg = gyroZ_deg;
            secondsstuck = 0;
            atleastonce = 1;
            state3 = 1;
        }





        // Update PREVIOUS_TIMES.print by adding the period
        *previous_time = next_time;
    }

    // Return something if needed
    return gyroZ_deg;
}


void setup() {

    myServo.attach(6);  // attaches the servo on pin 9
    myServo.write(90);
    Serial.begin(9600);
    Wire.begin();
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) {}  // stop everything if could not connect to MPU6050
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets();  // gyro and accelero

    pinMode(thrustFanPositivePin, OUTPUT);  // Set fan 1 positive pin as an output
    pinMode(thrustFanNegativePin, OUTPUT);  // Set fan 1 negative pin as an output

    //Initialize the lift fan
    pinMode(liftFanPositivePin, OUTPUT);  // Set fan 2 positive pin as an output
    pinMode(liftFanNegativePin, OUTPUT);  // Set fan 2 positive pin as an output

    // Set TRIG_PIN (PB3) as output
    DDRB |= (1 << PB3);
    pinMode(2, INPUT);
}

void loop() {



    mpu.update();
    double gyroZ_deg = mpu.getAngleZ();
    timer = millis();

    //analogWrite(liftFanPositivePin, 200);    // Set PWM duty cycle (0-255) for speed control
    //digitalWrite(liftFanNegativePin, HIGH);  // Turn on the negative terminal for fan 1

    //analogWrite(thrustFanPositivePin, 250);    // Set PWM duty cycle (0-255) for speed control
    //digitalWrite(thrustFanNegativePin, HIGH);  // Turn on the negative terminal for fan 1

    trigger_HC_SR04();
    uint16_t pulse_width = pulseIn(2, HIGH);
    int distance = (pulse_width * 0.0343) / 2;

    if (distance < 35 && notturnedyet == 0) {
        if(state3 == 0){
            analogWrite(liftFanPositivePin, 0);       // Set PWM duty cycle (0-255) for speed control
            digitalWrite(liftFanNegativePin, LOW);    // Turn on the negative terminal for fan 1
            analogWrite(thrustFanPositivePin, 0);     // Set PWM duty cycle (0-255) for speed control (default 250)
            digitalWrite(thrustFanNegativePin, LOW);  // Turn on the negative terminal for fan 1
        }
        
    }

    state = world(distance, state, gyroZ_deg);
    print(gyroZ_deg);

    if (state == 0) {
        myServo.write(90 - gyroZ_deg + relativedeg);
    }
}
