#ifndef SEARCH_H
#define SEARCH_H

// libc
#include <stdint.h>

//! Course pattern
typedef enum {
    COURSE_PATTERN_STRAIGHT,
    COURSE_PATTERN_ARC,
} COURSE_PATTERN;

//! Course
typedef struct {
    COURSE_PATTERN pattern;
    union {
        struct {
            float length;
        } straight;
        struct {
            float radius;
            float angle;
        } arc;
    };
} COURSE;

//! Course stack
typedef struct {
    COURSE *stack;
    uint32_t length;
} COURSE_STACK;

#endif // SEARCH_H
