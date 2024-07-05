#ifndef PARAMETER_STATIC_H
#define PARAMETER_STATIC_H

// hardware
#define PARAMETER_STATIC_ENCODER_PPR 119.538f //! [Pulses per Revolution]
#define PARAMETER_STATIC_TIRE_DIAMETER 20.9f  //! [mm]
#define PARAMETER_STATIC_TREAD_WIDTH 83.7f    //! [mm]

// logger
#define PARAMETER_STATIC_LOGGER_ENABLED 1
#define PARAMETER_STATIC_LOGGER_BUFFER_SIZE 4000

// search course
#define PARAMETER_STATIC_COURSE_STACK_SIZE 256

// odometry
#define PARAMETER_STATIC_ODOMETRY_NUM_VELOCITY_SAMPLES 16

// line
#define PARAMETER_STATIC_LINE_EXISTS_THRESHOLD 0.7f

// servo
#define PARAMETER_STATIC_SERVO_LIMIT_VOTAGE 4.0f


#endif // PARAMETER_STATIC_H
