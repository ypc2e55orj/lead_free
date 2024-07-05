#ifndef SEARCH_H
#define SEARCH_H

// libc
#include <stdint.h>
// project
#include "parameter_static.h"

//! Course pattern
typedef enum
{
    COURSE_PATTERN_STRAIGHT,
    COURSE_PATTERN_ARC,
} COURSE_PATTERN;

//! Course
typedef struct
{
    COURSE_PATTERN pattern;
    float angle;
    float length;
} COURSE;

//! Course stack
typedef struct
{
    COURSE stack[PARAMETER_STATIC_COURSE_STACK_SIZE];
    uint16_t index;
} COURSE_STACK;

/**
 * Reset search stack
 */
void SEARCH_Reset();

/**
 * Update search state
 */
void SEARCH_Update();

/**
 * @brief Print search stack
 */
void SEARCH_Print();

/**
 * @berif start search
 */
void SEARCH_Start();

/**
 * @berif stop search
 */
void SEARCH_Stop();


/**
 * @brief Start fast
 */
void FAST_Start();

/**
 * @brief Stop fast
 */
void FAST_Stop();

/**
 * @brief Get search data
 */
const COURSE *FAST_Get();

#endif // SEARCH_H
