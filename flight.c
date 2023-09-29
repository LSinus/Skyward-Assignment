//
//  flight.c
//  Skyward Assignment 2023-09
//
//  Created by Leonardo Sinibaldi.
//

#include <stdio.h>
#include <math.h>

#include "grader.h"

#define DELTA_T 0.02f // Rate of data in seconds.

#include "include/madgwickFilter.h"

#define G 9.81f
#define P0 101325 // Pressure in pascal at sea level.
#define NOISE 0.5f // Assuming a fixed noise value for data without knowing the real sensitivity of the instruments, in order to make control around a certain value possible.

typedef enum
{
    STATE_INIT,
    STATE_FLIGHT,
    STATE_DESCENT,
    STATE_LANDED
} flight_state_t;

typedef struct{
    float x;
    float y;
    float z;
} vec3_t;

float get_altitude_from_pressure(float pressure);
float get_vel(float altitude, float prev_heigh);
float filter_data(float data, float prev_data, float weight);
float vec_magn(float x, float y, float z);
vec3_t update_struct(float x, float y, float z);

int counter;

float roll;
float pitch;
float yaw;

vec3_t prev_gyro;
vec3_t prev_acc;

float altitude;
float prev_altitude;

float vel_x;
float prev_vel_x;

FILE *fp;

flight_state_t flight_state = STATE_INIT;

void init() { 
    
    printf("Init!\n"); 

    counter = 0;
    prev_vel_x = 0; // The previous value of vertical velocity is set to 0.

    fp = fopen("out.csv", "w");
    fprintf(fp, "time,altitude,acc_x,vel_x,roll,pitch,yaw\n");
}

void update(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y,float gyro_z, float baro){   

    // During the first iteration initial values are read from the file and all variables are initialized correctly.
    if (counter == 0){
        prev_altitude = get_altitude_from_pressure(baro);

        prev_acc.x = acc_x;
        prev_acc.y = acc_y;
        prev_acc.z = acc_z;

        prev_gyro.x = gyro_x;
        prev_gyro.y = gyro_y;
        prev_gyro.z = gyro_z;

        // Madgwick filter.
        imu_filter(acc_y, acc_x, acc_z, gyro_x, gyro_y, gyro_z);
        eulerAngles(q_est, &roll, &pitch, &yaw);

        counter++;

        return;
    }

    // During other iterations data is processed to make estimations about flight state.
    
    float filter_weight = 0.06;    // Filter weight for EWMA.
    float acc_magn;

    // Altitude estimation.
    altitude = filter_data(get_altitude_from_pressure(baro), prev_altitude, filter_weight);

    // Vertical veloctiy estimation.
    vel_x = get_vel(altitude, prev_altitude);

    filter_weight = 0.08;    // The weight is changed in order to reduce the impact of the filter because accelerometer and gyroscope data are directly read from a device.

    // Accelerometer data filtering.
    acc_x = filter_data(acc_x, prev_acc.x, filter_weight);
    acc_y = filter_data(acc_y, prev_acc.y, filter_weight);
    acc_z = filter_data(acc_z, prev_acc.z, filter_weight);

    // Gyroscope data filtering.
    gyro_x = filter_data(gyro_x, prev_gyro.x, filter_weight);
    gyro_y = filter_data(gyro_y, prev_gyro.y, filter_weight);
    gyro_z = filter_data(gyro_z, prev_gyro.z, filter_weight);

    // Acceleration vector magnitude.
    acc_magn = vec_magn(acc_x, acc_y, acc_z);

    // Madgwick filter.
    imu_filter(acc_y, acc_x, acc_z, gyro_x, gyro_y, gyro_z);
    eulerAngles(q_est, &roll, &pitch, &yaw);

    // Flight state datection

    // -  At the moment of liftoff a large acceleration given by the thrust of the engines is detected by the accelerometer on the x axis.
    if(acc_x > (G + NOISE) && flight_state == STATE_INIT) {     
        flight_state = STATE_FLIGHT;
        liftoff();
    } 

    // -  At the moment of the apogee there will be a point of inversion of the velocity which will therefore be around 0, 
    //    furthermore, any linear combination of pitch and yaw values ​​is checked by checking that at least one of the values ​​is greater than 45 degrees or less than -45 degrees with respect to the starting position, 
    //    i.e. the orientation of the rocket is no longer that of the ascent.
    else if(vel_x > 0 - NOISE && vel_x < 0 + NOISE && (yaw > 45 || yaw < -45 || pitch > 45 || pitch < -45) && flight_state == STATE_FLIGHT) {   
        flight_state = STATE_DESCENT;
        apogee();
    }

    // -  At the moment of landing the velocity will again be around 0, 
    //    the magnitude of the acceleration vector will be around G and, since we assume that the rocket lands horizontally braked by a parachute, 
    //    the vertical component of the acceleration detected by the accelerometer will be almost zero.
    else if(vel_x < 0 + NOISE && acc_magn < G + NOISE && acc_magn > G - NOISE && acc_x < 0 + NOISE && flight_state == STATE_DESCENT) {

        flight_state = STATE_LANDED;
        landed();

        fclose(fp);

        printf("\n+-------------------------------------------------------+\n");
        printf("|                                                       |\n");
        printf("|  Flight graphs generated, run graphs.py to show them  |\n");
        printf("|                                                       |\n");
        printf("+-------------------------------------------------------+\n\n");
    }

    // In this section all prev_values are updated for the next cycle.
    prev_altitude = altitude;
    prev_vel_x = vel_x;

    prev_acc = update_struct(acc_x, acc_y, acc_z);
    prev_gyro = update_struct(gyro_x, gyro_y, gyro_z);

    fprintf(fp, "%f,%f,%f,%f,%f,%f,%f\n", counter * DELTA_T, altitude, acc_x, vel_x, roll, pitch, yaw);
    
    counter++;
}

float get_altitude_from_pressure(float pressure){
    return 44330 * (1 - pow((pressure/P0), (1/5.255)));     // International Barometric Formula for altitude.
}

float get_vel(float position, float prev_position){
    return (position - prev_position) / DELTA_T;    // Simple numerical derivative.
}

float filter_data(float data, float prev_data, float weight){
    return (1-weight) * prev_data + weight * data;    // Implementation of Esponential weighted Moving Average (EWMA).
}

float vec_magn(float x, float y, float z){
    return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
}

vec3_t update_struct(float x, float y, float z){
    vec3_t updated_struct;

    updated_struct.x = x;
    updated_struct.y = y;
    updated_struct.z = z;
    
    return updated_struct;
}

