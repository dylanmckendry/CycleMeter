#include <Arduino.h>
#include "rotation_calculator.h"

float loop_time;
float last_write_time;

int wheel_sensor_pin = 2;
int wheel_sensor_value;

rotation_calculator wheel_rotation_calculator = rotation_calculator();

void setup() {
    Serial.begin(9600);
    pinMode(wheel_sensor_pin, INPUT);
}

void loop() {
    loop_time = millis();
    wheel_sensor_value = digitalRead(wheel_sensor_pin);
    wheel_rotation_calculator.on_reading(wheel_sensor_value == HIGH, loop_time);

    if (loop_time - last_write_time > 5000) {
        last_write_time = loop_time;
        Serial.println(wheel_rotation_calculator.rotations_per_unit);
    }
}