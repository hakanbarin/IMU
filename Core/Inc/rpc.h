#ifndef RPC_H
#define RPC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define MAX(a, b) ((a) > (b) ? (a) : (b))

    typedef enum
    {
        CALIBRATE_PID,
        DEGREES_OF_YAW,
        SET_DEPTH_CM,
        NUM_OF_RPC_SERVICES
    } rpc_service_t;

    typedef struct
    {
        rpc_service_t service;
        uint8_t data;
    } rpc_header_t;

    typedef struct
    {
        float kp;
        float ki;
        float kd;
    } calibrate_pid_t;

    typedef struct
    {
        float degrees;
    } degrees_of_yaw_t;

    typedef struct
    {
        float cm;
    } set_depth_cm_t;

#ifdef __cplusplus
}
#endif

#endif // RPC_H