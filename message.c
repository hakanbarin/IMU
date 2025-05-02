// message.c

#include "message.h"
#include <stdio.h>
#include <windows.h>

HANDLE open_serial(void) {
    HANDLE h = CreateFileA(
        COM_PORT,
        GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );
    
    if (h == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Error: could not open port %s\n", COM_PORT);
        return INVALID_HANDLE_VALUE;
    }

    DCB dcb = { .DCBlength = sizeof(dcb) };
    if (!GetCommState(h, &dcb)) {
        fprintf(stderr, "Error: GetCommState failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }

    dcb.BaudRate = BAUD_RATE;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    if (!SetCommState(h, &dcb)) {
        fprintf(stderr, "Error: SetCommState failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }

    return h;
}

void close_serial(HANDLE h) {
    if (h != INVALID_HANDLE_VALUE) {
        CloseHandle(h);
    }
}

void send_message(const rpc_message_t *msg) {
    HANDLE h = open_serial();
    if (h == INVALID_HANDLE_VALUE) {
        return;
    }

    DWORD written = 0;
    WriteFile(h, msg, sizeof(*msg), &written, NULL);
    close_serial(h);
}
