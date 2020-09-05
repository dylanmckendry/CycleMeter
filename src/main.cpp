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

int sensor_pressure_correction_count = 100;


// raw readings
float static_pressure;
float stagnation_pressure;
float sensor_pressure_correction;
float air_temperature;

float slope_degrees; // also get from altitude
float slope_degrees_correction = -20;
float acceleration = 0; // also get from changes in speed? what if this is -ve?

float wheel_rotations_second;
float crank_rotations_per_second; // if not pedalling, then no power

float cadence;
float slope_ratio;
float dynamic_pressure;
float air_density;
float ground_velocity;
float air_velocity;
float wind_velocity;
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

float calculate_slope_slope_degrees(float slope_degrees) {
    return tan(degrees_to_radians(slope_degrees));
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
float calculate_dynamic_pressure(float stagnation_pressure, float static_pressure, float sensor_pressure_correction) {
    return abs((stagnation_pressure - static_pressure) + sensor_pressure_correction);
}

float calculate_air_density(float static_pressure, float air_temperature) {
    return static_pressure / (dry_air_constant * (air_temperature + celsius_to_kelvin));
}

float calculate_air_drag(float air_density, float air_velocity) {
    return .5 * coef_friction * air_density * pow(air_velocity, 2) * frontal_area;
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
long measured_micros_per_mpu_reading;


FaBo9Axis fabo_9axis(0x68);

int crank_sensor_pin = 6;
int wheel_sensor_pin = 9;

Adafruit_BMP280 static_sensor;
Adafruit_BMP280 stagnation_sensor;

FaBoLCD_PCF8574 lcd(0x27);

char cadence_display[5] = "";
char ground_velocity_display[5] = "";
char air_velocity_display[5] = "";
char total_power_display[5] = "";

long start_loop_time;
long end_loop_time;
long last_mpu_read_time;
long next_mpu_read_time;
long last_power_calculation_time;
long last_lcd_write_time;
long last_serial_write_time;

float mpu_ax, mpu_ay, mpu_az;
float mpu_gx, mpu_gy, mpu_gz;

int crank_sensor_value;
int wheel_sensor_value;

Madgwick filter;
average_calculator slope_degrees_calculator(2, 50);
rotation_calculator wheel_rotation_calculator(2, 10);
rotation_calculator cadence_rotation_calculator(2, 10);
average_calculator air_velocity_calculator(2, 10);

#define LCD true;
#define DEBUG true;

void setup() {
    long start_time = micros();

#ifdef DEBUG
    Serial.begin(9600);
#endif

    if (!fabo_9axis.begin()) {
#ifdef DEBUG
        Serial.println("Could not find the MPU at address 0x68");
#endif
        while(1);
    }

    pinMode(crank_sensor_pin, INPUT);
    pinMode(wheel_sensor_pin, INPUT);

    if (!static_sensor.begin(0x77)) {
#ifdef DEBUG
        Serial.println("Could not find the static sensor at address 0x77");
#endif
        while (1);
    }

    if (!stagnation_sensor.begin(0x76)) {
#ifdef DEBUG
            Serial.println("Could not find the static sensor at address 0x76");
#endif
        while (1);
    }

    static_sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X2,
                    Adafruit_BMP280::STANDBY_MS_1);

    stagnation_sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X2,
                    Adafruit_BMP280::STANDBY_MS_1);
#ifdef LCD
    lcd.begin(16, 2);
#endif

    filter.begin(50);
    micros_per_mpu_reading = 1000000 / 50;
    last_mpu_read_time = start_time;
    next_mpu_read_time = start_time + micros_per_mpu_reading;

#ifdef DEBUG
    Serial.println("micros_per_mpu_reading " + String(micros_per_mpu_reading));
#endif

    for (int i = 0; i < 5; i++)
    {
        static_pressure = static_sensor.readPressure();
        stagnation_pressure = stagnation_sensor.readPressure();
    }

    sensor_pressure_correction = 0;
    for (int i = 0; i < sensor_pressure_correction_count; i++)
    {
        static_pressure = static_sensor.readPressure();
        stagnation_pressure = stagnation_sensor.readPressure();
        sensor_pressure_correction += stagnation_pressure - static_pressure;
    }

    sensor_pressure_correction /= -sensor_pressure_correction_count;

#ifdef DEBUG
    Serial.println("sensor_pressure_correction " + String(sensor_pressure_correction, 2));
#endif

#ifdef LCD
    lcd.print("s_p_c: " + String(sensor_pressure_correction, 2));
    delay(5000);
    lcd.clear();

    lcd.print("Place bike flat");
    delay(10000);
    lcd.clear();

    int imu_update_count = 0;
    int slope_degrees_correction_count = 0;

    while (slope_degrees_correction_count <= 1000) {
        start_loop_time = micros();

        if (start_loop_time >= next_mpu_read_time) {
            fabo_9axis.readAccelXYZ(&mpu_ax,&mpu_ay,&mpu_az);
            fabo_9axis.readGyroXYZ(&mpu_gx,&mpu_gy,&mpu_gz);
            
            filter.updateIMU(mpu_gx, mpu_gy, mpu_gz, mpu_ax, mpu_ay, mpu_az);
            imu_update_count++;
            if (imu_update_count < 100) {
                continue;
            }

            if (slope_degrees_calculator.on_reading(-filter.getPitch(), start_loop_time)) {
                slope_degrees_correction_count++;

                if (slope_degrees_correction_count == 500) {
                    slope_degrees_correction = slope_degrees_calculator.average;
                    slope_degrees_calculator.reset();
                    imu_update_count = 0;

                    lcd.print("Rotate bike");
                    delay(15000);
                    lcd.clear();
                } else if (slope_degrees_correction_count == 1000) {
                    slope_degrees_correction += slope_degrees_calculator.average;
                    slope_degrees_correction /= -2;
                    slope_degrees_calculator.reset();
                    imu_update_count = 0;

#ifdef DEBUG
                    Serial.println("slope_degrees_correction " + String(slope_degrees_correction, 2));
#endif
                    lcd.print("s_d_c: " + String(slope_degrees_correction, 2));
                    delay(5000);
                    lcd.clear();
                }
            }

            next_mpu_read_time = start_loop_time + micros_per_mpu_reading;
        }
    }


    lcd.print("S:");
    lcd.setCursor(8, 0);
    lcd.print("C:");
    lcd.setCursor(0, 1);
    lcd.print("W:");
    lcd.setCursor(8, 1);
    lcd.print("P:");
#endif
}


void loop() {
    start_loop_time = micros();

    if (start_loop_time >= next_mpu_read_time) {
        fabo_9axis.readAccelXYZ(&mpu_ax,&mpu_ay,&mpu_az);
        fabo_9axis.readGyroXYZ(&mpu_gx,&mpu_gy,&mpu_gz);
        
        filter.updateIMU(mpu_gx, mpu_gy, mpu_gz, mpu_ax, mpu_ay, mpu_az);

        if (slope_degrees_calculator.on_reading(-filter.getPitch(), start_loop_time)) {
            slope_degrees = slope_degrees_calculator.average + slope_degrees_correction;
            slope_ratio = calculate_slope_ratio(slope_degrees);
            vertical_velocity = calculate_vertical_velocity(slope_degrees, ground_velocity);
        }

        // slope_degrees = (atan2((mpu_ax) , sqrt(mpu_ay * mpu_ay + mpu_az * mpu_az)) * 57.3) + slope_degrees_correction;
        // if (slope_degrees > 30) {
        //     slope_degrees = 30;
        // } else if (slope_degrees < -30) {
        //     slope_degrees = -30;
        // }
#ifdef DEBUG
        measured_micros_per_mpu_reading = start_loop_time - last_mpu_read_time;
        last_mpu_read_time = start_loop_time;
#endif
        next_mpu_read_time = start_loop_time + micros_per_mpu_reading;
    }

    wheel_sensor_value = digitalRead(wheel_sensor_pin);
    if (wheel_rotation_calculator.on_reading(wheel_sensor_value == LOW, start_loop_time)) {
        ground_velocity = calculate_ground_velocity(wheel_rotation_calculator.rotations_per_second, wheel_radius);
    }

    // crank_sensor_value = digitalRead(crank_sensor_pin);
    // if (cadence_rotation_calculator.on_reading(crank_sensor_value == LOW, start_loop_time)) {
    //     cadence = calculate_cadence(cadence_rotation_calculator.rotations_per_second);
    // }

    static_pressure = static_sensor.readPressure();
    stagnation_pressure = stagnation_sensor.readPressure();
    air_temperature = static_sensor.readTemperature();
    dynamic_pressure = calculate_dynamic_pressure(stagnation_pressure, static_pressure, sensor_pressure_correction);
    air_density = calculate_air_density(static_pressure, air_temperature);
    if (air_velocity_calculator.on_reading(calculate_air_velocity(dynamic_pressure, air_density), start_loop_time)) {
        air_velocity = air_velocity_calculator.average;
    }

    wind_velocity = air_velocity - ground_velocity;

    if (start_loop_time - last_power_calculation_time > 1 * 1000000) {
        last_power_calculation_time = start_loop_time;

        gravity_power = calculate_gravity_power(total_weight, vertical_velocity);

        inertia_power = calculate_inertia_power(total_weight, acceleration, ground_velocity);

        air_drag = calculate_air_drag(air_density, air_velocity);
        air_drag_power = calculate_air_drag_power(air_drag, ground_velocity);

        tire_resistance_power = calculate_tire_resistance_power(total_weight, slope_ratio, ground_velocity);
    
        total_power = gravity_power + inertia_power + air_drag_power + tire_resistance_power;

        if (total_power < 0) {
            total_power = 0;
        }
    }

#ifdef DEBUG
    end_loop_time = micros();
#endif

#ifdef LCD
    if (start_loop_time - last_lcd_write_time > 5 * 1000000) {
        last_lcd_write_time = start_loop_time;

        lcd.setCursor(2, 0);
        dtostrf(ground_velocity * 3.6, 5, 1, ground_velocity_display);
        lcd.print(ground_velocity_display);

        lcd.setCursor(2, 1);
        dtostrf(air_velocity * 3.6, 5, 1, air_velocity_display);
        lcd.print(air_velocity_display);

        lcd.setCursor(10, 0);
        dtostrf(slope_degrees, 5, 1, cadence_display);
        lcd.print(cadence_display);

        lcd.setCursor(10, 1);
        dtostrf(total_power, 5, 1, total_power_display);
        lcd.print(total_power_display);
    }
#endif

#ifdef DEBUG
    if (start_loop_time - last_serial_write_time > 5 * 1000000) {
        last_serial_write_time = start_loop_time;

        Serial.print("loop_time: ");
        Serial.println(end_loop_time - start_loop_time);

        Serial.print("slope_degrees: ");
        Serial.println(slope_degrees);

        Serial.print("ground_velocity: ");
        Serial.println(ground_velocity);

        Serial.print("air_velocity: ");
        Serial.println(air_velocity);

        Serial.print("wind_velocity: ");
        Serial.println(wind_velocity);

        Serial.print("vertical_velocity: ");
        Serial.println(vertical_velocity);

        Serial.print("gravity_power: ");
        Serial.println(gravity_power);

        Serial.print("inertia_power: ");
        Serial.println(inertia_power);

        Serial.print("air_drag_power: ");
        Serial.println(air_drag_power);

        Serial.print("tire_resistance_power: ");
        Serial.println(tire_resistance_power);

        Serial.print("total_power: ");
        Serial.println(total_power);

        Serial.print("measured_micros_per_mpu_reading: ");
        Serial.println(measured_micros_per_mpu_reading);

        Serial.println();
    }
#endif
}