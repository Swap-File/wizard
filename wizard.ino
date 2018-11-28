#include <Wire.h>
#include <ZX_Sensor.h>
#include <SPI.h>
#include <FastLED.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define NRF_PACKETSIZE 9
#define IMU_INTERRUPT_PIN 2
#define FASTLED_PIN 3
#define NUM_LEDS 4

RF24 radio(9, 10);

CRGB leds[NUM_LEDS];
CRGB leds_pulse;
CRGB leds_effect;
CHSV leds_spin;

boolean flashlight = false;
MPU6050 mpu;
ZX_Sensor zx_sensor = ZX_Sensor(0x10); // ZX Sensor I2C address
GestureType gesture;
uint8_t gesture_speed;

bool main_ran = false;


uint8_t  wireless_remote_mode = 0;
uint32_t wireless_remote_update_time = 0;
uint32_t wireless_remote_time = 0;
CRGB     wireless_remote_pulse = CRGB::Black;

uint16_t imu_fps = 0;
uint16_t idle_fps = 0;
uint8_t  packets_in = 0;
uint8_t  packets_out = 0;

uint8_t broadcasted_mode = 0;
uint8_t actual_mode = 0;

uint32_t tap_time = 0;
uint8_t tap_count = 0;
bool tap_counted = false;


uint32_t gesture_time = 0;
uint8_t gesture_count = 0;


uint32_t left_swipe_time = 0;
uint32_t right_swipe_time = 1000;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q2;          // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float z_angle = 0;


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  mpuInterrupt = true;
}

uint8_t side = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  printf_begin();
  Serial.begin(115200);
  zx_setup();
  nr_setup();
  FastLED.addLeds<WS2811, FASTLED_PIN, GRB>(leds, NUM_LEDS);
  imu_setup();  //init imu last so fifo doesnt fill up while init-ing other stuff
}


void update_mode(bool pulse) {

  static uint8_t actual_mode_last = 0;
  if (actual_mode_last != actual_mode) {
    if (actual_mode == 1) {
      if (pulse) leds_pulse  = CRGB::White;
    } else if (actual_mode == 2) {
      if (pulse) leds_pulse  = CRGB::Red;
      leds_effect = CRGB::Red;
    } else if (actual_mode == 3) {
      if (pulse)  leds_pulse  = CRGB::Green;
      leds_effect = CRGB::Green;
    } else if (actual_mode == 4) {
      if (pulse) leds_pulse  = CRGB::Blue;
      leds_effect = CRGB::Blue;
    } else if (actual_mode == 5) {
      if (pulse) leds_pulse  = CRGB::Lime;
      leds_effect = CRGB::Black;
    } else if (actual_mode >= 6) {
      if (pulse) leds_pulse  = CRGB::Black;
      leds_effect = CRGB::Black;
    }
    actual_mode_last = actual_mode;
  }

}

void loop() {

  if (!dmpReady) return;

  while ((!mpuInterrupt && fifoCount < packetSize) || !main_ran) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }

    static uint32_t nrf24_timer = 0;
    if (millis() - nrf24_timer > 15) {
      nrf24_timer = millis();
      nr_update();
    }

    static uint8_t wireless_remote_mode_last = 0;
    if (wireless_remote_mode_last != wireless_remote_mode && wireless_remote_mode < 127 ) {
      Serial.print("Remote Change Init! ");
      broadcasted_mode = 255;
      actual_mode = wireless_remote_mode;
      Serial.println(actual_mode);
      wireless_remote_mode_last = wireless_remote_mode;
      update_mode(0);
    }

    static uint32_t fps_time = 0;
    if (millis() - fps_time > 1000) {
      fps_time = millis();

      Serial.print(F(" packets_in : "));
      Serial.print(packets_in);
      Serial.print(F(" packets_out : "));
      Serial.print(packets_out);
      Serial.print(F(" idle_fps : "));
      Serial.print(idle_fps);
      Serial.print(F(" imu_fps : "));
      Serial.println(imu_fps);

      idle_fps = 0;
      imu_fps = 0;
      packets_in = 0;
      packets_out = 0;
    }

    static uint32_t fade_timer = 0;
    if (millis() - fade_timer > 200 ) {
      fadeToBlackBy( &leds_pulse, 1, 10);
      fade_timer = millis();
    }

    uint32_t temp = millis();
    uint32_t combined_time = (temp >> 1) +  ((temp - wireless_remote_update_time) + wireless_remote_time >> 1);

    float val = (exp(sin(combined_time / 2000.0 * PI)) - 0.36787944) * 108.0;

    CRGB temp2 = leds_effect;

    if (actual_mode == 5) {
      temp2 = CHSV((( combined_time >> 6)  % 256), 255, 255);
    } else {
      temp2.nscale8_video(val);
    }

    leds[0] = temp2 | leds_pulse | wireless_remote_pulse;
    leds[1] = leds[0];

    if (abs(z_angle) > 10) {
      uint8_t temp3 = ((uint8_t )abs(z_angle) - 10) << 1;
      uint8_t temp4  = qadd8(temp3, temp3);
      leds[2] = leds[0].nscale8_video(temp4);
    } else {
      leds[2] = CRGB::Black;
    }

    if (flashlight) {
      leds[2] = CRGB::White;
    }
    
    leds[3] = leds[2];
    FastLED.show();
    idle_fps++;

    main_ran = true;
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;


    mpu.dmpGetQuaternion(&q2, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q2);
    mpu.dmpGetYawPitchRoll(ypr, &q2, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    // Serial.print("ypr\t");
    //Serial.print(ypr[0] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180 / M_PI);  // Z angle
    z_angle = ypr[1] * 180 / M_PI;

    //Serial.print("\t");
    // Serial.println(ypr[2] * 180 / M_PI);

    if ( zx_sensor.gestureAvailable() ) {
      gesture = zx_sensor.readGesture();
      gesture_count++;
      gesture_time = millis();
      /* gesture = zx_sensor.readGesture();
        gesture_speed = zx_sensor.readGestureSpeed();
        switch ( gesture ) {
         case NO_GESTURE:
           Serial.println("No Gesture");
           break;
         case RIGHT_SWIPE:
           Serial.print(F("Right Swipe. Speed: "));
           Serial.println(gesture_speed, DEC);
          // right_swipe_time = millis();
           break;
         case LEFT_SWIPE:
           Serial.print(F("Left Swipe. Speed: "));
           Serial.println(gesture_speed, DEC);
           //left_swipe_time = millis();
           break;
         case UP_SWIPE:
           Serial.print(F("Up Swipe. Speed: "));
           Serial.println(gesture_speed, DEC);
           break;
         default:
           break;

        }
      */
    }

    if (gesture_count > 0) {
      if (millis() - gesture_time > 1000) {
        Serial.print("Gesture! ");
        Serial.println(gesture_count);

        if (gesture_count >= 4) {
          flashlight = !flashlight;
        }

        gesture_count = 0;
      }
    }

    uint8_t x_pos;
    uint8_t z_pos;

    x_pos = zx_sensor.readX();
    if ( x_pos != ZX_ERROR ) {
      //Serial.print("X: ");
      //Serial.print(x_pos);
    }
    z_pos = zx_sensor.readZ();
    if ( z_pos != ZX_ERROR ) {
      //Serial.print(" Z: ");
      //Serial.print(z_pos);
    }
    //Serial.println("");



    //some kind of slow dim thing, figure it later
    uint32_t current_acceleration = abs(aaReal.z) + abs( aaReal.y ) + abs(aaReal.x );

    //Serial.println(current_acceleration);

    if (current_acceleration > 10000 && tap_counted == false && millis() - tap_time > 300) {
      tap_count++;
      tap_time = millis();
      tap_counted = true;
      leds_pulse = CRGB::White; //white pop for hit detect
    }
    if (current_acceleration < 4000) {
      tap_counted = false;
    }
    if (tap_count > 0) {
      if (millis() - tap_time > 1000) {
        Serial.print("TAP! ");
        if (tap_count > 1){
          flashlight = false;
        }
        Serial.println(tap_count);
        actual_mode = tap_count;
        broadcasted_mode = tap_count;
        update_mode(1);
        tap_count = 0;
      }
    }

    main_ran = false;  //force main loop to run once
    imu_fps++;
  }
}
