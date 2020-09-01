#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <FaBo9Axis_MPU9250.h>
#include <FaBoLCD_PCF8574.h>
#include <MadgwickAHRS.h>
#include "average_calculator.h"
#include "rotation_calculator.h"


// need to look at humidity with BME280 for better air pressure
// wind angle use mics
// ultrasonics for position

float pi = 3.14;
float gravity = 9.8067;
float celsius_to_kelvin = 273.15;
float dry_air_constant = 287.05;

float tire_resistance_crr = 0.0055;
float frontal_area = .5 * 0.44704;
float coef_friction = 1;

float rider_mass = 70;
float bike_mass = 10;
float total_mass = rider_mass + bike_mass;
float total_weight = total_mass * gravity;
float wheel_radius = 0.622 / 2.0;
//float drivetrain_efficiency = 0.98;
//float rider_position = 0;
//float tire_  = 0;
//float tire_pressue = 0;
//float road_condition = 0;


// raw readings
float static_pressure = 103068;
float stagnation_pressure = 103074;
float sensor_pressure_bias = 0;
float air_temperature = 20;

float slope_degrees = 5; // also get from altitude
float acceleration = 0; // also get from changes in speed? what if this is -ve?

float wheel_rotations_second = 4.6;
float crank_rotations_per_second = 1; // if not pedalling, then no power

float cadence;
float slope_ratio;
float dynamic_pressure;
float air_density;
float ground_velocity;
float air_velocity;
float relative_velocity;
float vertical_velocity;
float air_drag;


// power
float gravity_power;
float inertia_power;
float air_drag_power;
float tire_resistance_power;
float total_power;

float degrees_to_radians(float degrees) {
    return degrees * (pi / 180);
}

float calculate_slope_ratio(float slope_degrees) {
    return tan(degrees_to_radians(slope_degrees));
}

float calculate_cadence(float crank_rotations_second) {
    return crank_rotations_second * 60;
}

float calculate_ground_velocity(float wheel_rotations_second, float wheel_radius) {
    return wheel_rotations_second * wheel_radius * 2 * pi;
}

float calculate_vertical_velocity(float slope_degrees, float ground_velocity) {
    return sin(degrees_to_radians(slope_degrees)) * ground_velocity;
}

// TODO: should this be abs or just go to 0
float calculate_dynamic_pressure(float stagnation_pressure, float static_pressure, float sensor_pressure_bias) {
    return abs(stagnation_pressure - static_pressure - sensor_pressure_bias);
}

float calculate_air_density(float static_pressure, float air_temperature) {
    return static_pressure / (dry_air_constant * (air_temperature + celsius_to_kelvin));
}

float calculate_air_drag(float air_density, float relative_velocity) {
    return .5 * coef_friction * air_density * pow(relative_velocity, 2) * frontal_area;
}

// power
float calculate_air_velocity(float dynamic_pressure, float air_density) {
    return sqrt((2 * dynamic_pressure) / air_density);
}

float calculate_gravity_power(float total_weight, float vertical_velocity) {
    return total_weight * vertical_velocity;
}

float calculate_inertia_power(float total_weight, float acceleration, float ground_velocity) {
    return total_weight * acceleration * ground_velocity;
}

float calculate_air_drag_power(float air_drag, float ground_velocity) {
    return air_drag * ground_velocity;
}

float calculate_tire_resistance_power(float total_weight, float slope_ratio, float ground_velocity) {
    return total_weight * sqrt((1 - pow(slope_ratio, 2))) * tire_resistance_crr * ground_velocity;
}

long micros_per_mpu_reading;

int crank_sensor_pin = 2;
int wheel_sensor_pin = 2;

FaBoLCD_PCF8574 lcd(0x27);

char cadence_display[5] = "";
char ground_velocity_display[5] = "";
char air_velocity_display[5] = "";
char total_power_display[5] = "";

long loop_time;
long last_mpu_read_time;
long last_lcd_write_time;

int wheel_sensor_value;


// average_calculator test_calc = average_calculator();
Madgwick filter = Madgwick();
rotation_calculator wheel_rotation_calculator = rotation_calculator(2, 5);
rotation_calculator cadence_rotation_calculator = rotation_calculator(2, 3);


void setup() {
    long start_time = micros();

    Serial.begin(9600);
    pinMode(crank_sensor_pin, INPUT);
    pinMode(wheel_sensor_pin, INPUT);

    filter.begin(1000);
    micros_per_mpu_reading = 1000000 / 25;
    last_mpu_read_time = start_time;

    // LCD:
    lcd.begin(16, 2);
    lcd.print("S:");
    lcd.setCursor(8, 0);
    lcd.print("C:");
    lcd.setCursor(0, 1);
    lcd.print("W:");
    lcd.setCursor(8, 1);
    lcd.print("P:");

    // slope_ratio = calculate_slope_ratio(slope_degrees);
    // ground_velocity = calculate_ground_velocity(wheel_rotations_second, wheel_radius);
    // dynamic_pressure = calculate_dynamic_pressure(stagnation_pressure, static_pressure, sensor_pressure_bias);
    // air_density = calculate_air_density(static_pressure, air_temperature);
    // air_velocity = calculate_air_velocity(dynamic_pressure, air_density);
    // vertical_velocity = calculate_vertical_velocity(slope_degrees, ground_velocity);

    // cout << "slope_ratio " << slope_ratio << endl;
    // cout << "ground_velocity " << ground_velocity << endl;
    // cout << "air_velocity " << air_velocity << endl;
    // cout << "vertical_velocity " << vertical_velocity << endl;

    // relative_velocity = ground_velocity + air_velocity;
    
    // gravity_power = calculate_gravity_power(total_weight, vertical_velocity);

    // inertia_power = calculate_inertia_power(total_weight, acceleration, ground_velocity);

    // air_drag = calculate_air_drag(air_density, relative_velocity);
    // air_drag_power = calculate_air_drag_power(air_drag, ground_velocity);

    // tire_resistance_power = calculate_tire_resistance_power(total_weight, slope_ratio, ground_velocity);
 
    // total_power = gravity_power + inertia_power + air_drag_power + tire_resistance_power;

    // cout << "gravity_power " << gravity_power << endl;
    // cout << "inertia_power " << inertia_power << endl;
    // cout << "air_drag_power " << air_drag_power << endl;
    // cout << "tire_resistance_power " << tire_resistance_power << endl;
    // cout << "total_power " << total_power << endl;
}

void loop() {
    loop_time = micros();

    if (loop_time - last_mpu_read_time >= micros_per_mpu_reading) {
        //filter.updateIMU(gx, gy, gz, ax, ay, az);
        slope_degrees = filter.getPitch();
        last_mpu_read_time += micros_per_mpu_reading;
    }

    wheel_sensor_value = digitalRead(wheel_sensor_pin);
    if (wheel_rotation_calculator.on_reading(wheel_sensor_value == LOW, loop_time)) {
        ground_velocity = calculate_ground_velocity(wheel_rotation_calculator.rotations_per_second, wheel_radius);
        cadence = calculate_cadence(wheel_rotation_calculator.rotations_per_second);

        relative_velocity = ground_velocity + air_velocity;
        
        gravity_power = calculate_gravity_power(total_weight, vertical_velocity);

        inertia_power = calculate_inertia_power(total_weight, acceleration, ground_velocity);

        air_drag = calculate_air_drag(air_density, relative_velocity);
        air_drag_power = calculate_air_drag_power(air_drag, ground_velocity);

        tire_resistance_power = calculate_tire_resistance_power(total_weight, slope_ratio, ground_velocity);
    
        total_power = gravity_power + inertia_power + air_drag_power + tire_resistance_power;

        // lcd
        lcd.setCursor(2, 0);
        dtostrf(ground_velocity, 5, 1, ground_velocity_display);
        lcd.print(ground_velocity_display);

        lcd.setCursor(10, 0);
        dtostrf(cadence, 5, 1, cadence_display);
        lcd.print(cadence_display);

        lcd.setCursor(2, 1);
        dtostrf(air_velocity, 5, 1, air_velocity_display);
        lcd.print(air_velocity_display);

        lcd.setCursor(10, 1);
        dtostrf(total_power, 5, 1, total_power_display);
        lcd.print(total_power_display);

        // serial
        // Serial.print("velocity: ");
        // Serial.print(ground_velocity);
        // Serial.print(", cadence: ");
        // Serial.println(cadence);
    }

    // if (loop_time - last_write_time > 5000) {
    //     last_write_time = loop_time;
    //     Serial.println(wheel_rotation_calculator.rotations_per_unit);
    // }
}