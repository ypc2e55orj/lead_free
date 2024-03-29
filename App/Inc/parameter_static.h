#ifndef PARAMETER_STATIC_H
#define PARAMETER_STATIC_H

// logger
#define PARAMETER_STATIC_LOGGER_ENABLED 1
#define PARAMETER_STATIC_LOGGER_BUFFER_SIZE 4000

// odometry
#define PARAMETER_STATIC_ODOMETRY_ENCODER_PPR 119.538f //! [Pulses per Revolution]
#define PARAMETER_STATIC_ODOMETRY_TIRE_DIAMETER 20.9f  //! [mm]
#define PARAMETER_STATIC_ODOMETRY_NUM_VELOCITY_SAMPLES 8

// line
#define PARAMETER_STATIC_LINE_EXISTS_THRESHOLD 0.7f

#endif // PARAMETER_STATIC_H
