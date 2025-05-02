#ifndef MESSAGE_H
#define MESSAGE_H

#include <windows.h>
#include "Core/Inc/rpc.h"

#ifdef __cplusplus
extern "C" {
#endif

HANDLE open_serial(void);
/*
    * @brief  Open and configure the serial port.
    * @return A valid HANDLE, or INVALID_HANDLE_VALUE on error
*/

void close_serial(HANDLE h);
/*
    * @brief  Close a previously opened serial port.
    * @param  h  The HANDLE returned by open_serial()
*/

void send_message(const rpc_message_t *msg);
/*
    * @brief  Send an rpc_message_t packet over the serial port.
    * @param  msg  Pointer to the rpc_message_t to send
    * @note   Errors are silently ignored
*/

#ifdef __cplusplus
}
#endif

#endif // MESSAGE_H
