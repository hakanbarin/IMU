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

#ifdef __cplusplus
}
#endif

#endif // RPC_H