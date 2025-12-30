#ifndef FILLER_FSM_H
#define FILLER_FSM_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FILLER_IDLE = 0,
    FILLER_FIND_SLOT,
    FILLER_VERIFY_EMPTY,
    FILLER_FILL,
    FILLER_SLOT_SETTLE,
    FILLER_DRIP_WAIT,
    FILLER_VERIFY_TARGET,
    FILLER_DONE,
    FILLER_FAULT
} filler_state_t;

typedef enum {
    FLT_NONE = 0,
    FLT_NO_JAR,
    FLT_WEIGHT_RANGE,
    FLT_MOTOR_TIMEOUT,
    FLT_SERVO_TIMEOUT,
    FLT_SCALE_TIMEOUT,
    FLT_EMPTY_HONEY,
    FLT_USER_ABORT
} filler_fault_t;

void filler_start_task(UBaseType_t prio, BaseType_t core); // creates the FSM task
void filler_request_start(void);  // set an atomic flag
void filler_request_abort(void);  // set an atomic flag
filler_state_t filler_get_state(void);
uint8_t filler_get_slot_idx(void);
filler_fault_t filler_get_fault(void);
const char *filler_state_name(filler_state_t st);
const char *filler_fault_name(filler_fault_t flt);

#ifdef __cplusplus
}
#endif

#endif // FILLER_FSM_H
