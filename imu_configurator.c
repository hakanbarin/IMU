#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "IMU\IMU\Core\Inc\rpc.h"  // Use definitions from rpc.h

// RPC message size: 4 bytes for service type + 16 bytes for data = 20 bytes total
#define RPC_MSG_SIZE 20

// Sends a 20-byte message over the COM port.
void send_message(HANDLE hSerial, uint32_t service, const void *data, size_t data_len) {
    unsigned char msg[RPC_MSG_SIZE];
    // Copy the service code into the first 4 bytes (little-endian format)
    memcpy(msg, &service, sizeof(service));
    
    // Copy the data; if data_len is less than 16 bytes, pad the remainder with zeros.
    if (data && data_len > 0) {
        memcpy(msg + 4, data, data_len);
    }
    if (data_len < 16) {
        memset(msg + 4 + data_len, 0, 16 - data_len);
    }
    
    DWORD bytesWritten;
    if (!WriteFile(hSerial, msg, RPC_MSG_SIZE, &bytesWritten, NULL)) {
        fprintf(stderr, "Failed to send the message.\n");
    } else if (bytesWritten != RPC_MSG_SIZE) {
        fprintf(stderr, "Unexpected number of bytes written: %lu\n", bytesWritten);
    }
}

// CALIBRATE_PID command: 3 floats (12 bytes) + 4 bytes padding = 16 bytes of data
void send_calibrate_pid(HANDLE hSerial, float kp, float ki, float kd) {
    unsigned char data[16];
    memcpy(data, &kp, sizeof(float));
    memcpy(data + 4, &ki, sizeof(float));
    memcpy(data + 8, &kd, sizeof(float));
    memset(data + 12, 0, 4);  // Padding
    send_message(hSerial, CALIBRATE_PID, data, sizeof(data));
    printf("Calibrate PID sent: kp = %f, ki = %f, kd = %f\n", kp, ki, kd);
}

// SET_DEGREES_OF_YAW command: 1 float (4 bytes) + 12 bytes padding
void send_set_degrees_of_yaw(HANDLE hSerial, float degrees) {
    unsigned char data[16];
    memcpy(data, &degrees, sizeof(float));
    memset(data + 4, 0, 12);
    send_message(hSerial, SET_DEGREES_OF_YAW, data, sizeof(data));
    printf("Yaw set to: %f degrees\n", degrees);
}

// SET_DEPTH_CM command: 1 float (4 bytes) + 12 bytes padding
void send_set_depth_cm(HANDLE hSerial, float depth_cm) {
    unsigned char data[16];
    memcpy(data, &depth_cm, sizeof(float));
    memset(data + 4, 0, 12);
    send_message(hSerial, SET_DEPTH_CM, data, sizeof(data));
    printf("Depth set to: %f cm\n", depth_cm);
}

// COEFFICIENT_COMPLEMANTARY_FILTER command: 4 floats (16 bytes) to fully fill the data field
void send_coefficient_complemantary_filter(HANDLE hSerial, float alpha_pitch, float alpha_yaw, float alpha_roll, float alpha_stabilize) {
    unsigned char data[16];
    memcpy(data, &alpha_pitch, sizeof(float));
    memcpy(data + 4, &alpha_yaw, sizeof(float));
    memcpy(data + 8, &alpha_roll, sizeof(float));
    memcpy(data + 12, &alpha_stabilize, sizeof(float));
    send_message(hSerial, COEFFICIENT_COMPLEMANTARY_FILTER, data, sizeof(data));
    printf("Filter coefficients sent: alpha_pitch = %f, alpha_yaw = %f, alpha_roll = %f, alpha_stabilize = %f\n",
           alpha_pitch, alpha_yaw, alpha_roll, alpha_stabilize);
}

// PWM_MOTORS_FOR_STOP command: 8 uint16 values (16 bytes) to fully fill the data field
void send_pwm_motors_for_stop(HANDLE hSerial, unsigned short pwm_values[8]) {
    unsigned char data[16];
    memcpy(data, pwm_values, 8 * sizeof(unsigned short));
    send_message(hSerial, PWM_MOTORS_FOR_STOP, data, sizeof(data));
    printf("PWM motor command sent: ");
    for (int i = 0; i < 8; i++) {
        printf("%hu ", pwm_values[i]);
    }
    printf("\n");
}

// FOR_ARM command: 1 uint8 value (1 byte) + 15 bytes padding = 16 bytes of data
void send_for_arm(HANDLE hSerial, uint8_t arm_flag) {
    unsigned char data[16];
    data[0] = arm_flag;
    memset(data + 1, 0, 15);
    send_message(hSerial, FOR_ARM, data, sizeof(data));
    printf("Arm/Disarm command sent. Value: %u\n", arm_flag);
}

// Reads and prints a 20-byte response from the COM port.
void read_response(HANDLE hSerial) {
    uint8_t response[RPC_MSG_SIZE];
    DWORD bytesRead;
    // Attempt to read a response using the configured timeouts
    if (ReadFile(hSerial, response, RPC_MSG_SIZE, &bytesRead, NULL)) {
        if (bytesRead > 0) {
            printf("Received %lu bytes: ", bytesRead);
            for (DWORD i = 0; i < bytesRead; i++) {
                printf("%02X ", response[i]);
            }
            printf("\n");
        } else {
            printf("No response received.\n");
        }
    } else {
        fprintf(stderr, "Error reading from serial port.\n");
    }
}

// Parses terminal commands and calls the corresponding RPC function.
void process_command(char *command, HANDLE hSerial) {
    // Convert the command string to lowercase
    for (int i = 0; command[i]; i++) {
        command[i] = (char)tolower(command[i]);
    }

    if (strncmp(command, "pid", 3) == 0) {
        // Example: "pid 1.0,0.1,0.05"
        char *ptr = command + 3;
        while (*ptr == ' ') ptr++;
        float kp, ki, kd;
        if (sscanf(ptr, "%f,%f,%f", &kp, &ki, &kd) == 3) {
            send_calibrate_pid(hSerial, kp, ki, kd);
        } else {
            printf("PID command must be in the format 'pid kp,ki,kd'.\n");
        }
    }
    else if (strncmp(command, "yaw", 3) == 0) {
        // Example: "yaw 45"
        char *ptr = command + 3;
        while (*ptr == ' ') ptr++;
        float degrees;
        if (sscanf(ptr, "%f", &degrees) == 1) {
            send_set_degrees_of_yaw(hSerial, degrees);
        } else {
            printf("Yaw command must be in the format 'yaw degrees'.\n");
        }
    }
    else if (strncmp(command, "depth", 5) == 0) {
        // Example: "depth 120"
        char *ptr = command + 5;
        while (*ptr == ' ') ptr++;
        float depth_cm;
        if (sscanf(ptr, "%f", &depth_cm) == 1) {
            send_set_depth_cm(hSerial, depth_cm);
        } else {
            printf("Depth command must be in the format 'depth <value>'.\n");
        }
    }
    else if (strncmp(command, "complementary", 13) == 0) {
        // Example: "complementary 0.98,0.95,0.97,0.93"
        char *ptr = command + 13;
        while (*ptr == ' ') ptr++;
        float a_pitch, a_yaw, a_roll, a_stabilize;
        if (sscanf(ptr, "%f,%f,%f,%f", &a_pitch, &a_yaw, &a_roll, &a_stabilize) == 4) {
            send_coefficient_complemantary_filter(hSerial, a_pitch, a_yaw, a_roll, a_stabilize);
        } else {
            printf("Complementary command must be in the format 'complementary a_pitch,a_yaw,a_roll,a_stabilize'.\n");
        }
    }
    else if (strncmp(command, "motors", 6) == 0) {
        // Example: "motors 1000,1000,1000,1000,1000,1000,1000,1000"
        char *ptr = command + 6;
        while (*ptr == ' ') ptr++;
        unsigned short pwm_values[8];
        int count = sscanf(ptr, "%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu",
                           &pwm_values[0], &pwm_values[1], &pwm_values[2], &pwm_values[3],
                           &pwm_values[4], &pwm_values[5], &pwm_values[6], &pwm_values[7]);
        if (count == 8) {
            send_pwm_motors_for_stop(hSerial, pwm_values);
        } else {
            printf("Motors command must include 8 PWM values.\n");
        }
    }
    else if (strncmp(command, "arm", 3) == 0) {
        // Example: "arm 1", "arm 0", or "arm 2"
        char *ptr = command + 3;
        while (*ptr == ' ') ptr++;
        int flag;
        if (sscanf(ptr, "%d", &flag) == 1 && (flag == 0 || flag == 1 || flag == 2)) {
            send_for_arm(hSerial, (uint8_t)flag);
        } else {
            printf("Arm command must be in the format 'arm 0', 'arm 1', or 'arm 2'.\n");
        }
    }
    else {
        printf("Unknown command. Available commands: pid, yaw, depth, complementary, motors, arm\n");
    }
}

// Helper function to open and configure the COM port.
HANDLE open_serial_port(const char *port_name) {
    HANDLE hSerial = CreateFile(port_name,
                                GENERIC_READ | GENERIC_WRITE,
                                0,      // No sharing
                                NULL,   // Default security attributes
                                OPEN_EXISTING,
                                0,
                                NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Failed to open serial port: %s\n", port_name);
        return INVALID_HANDLE_VALUE;
    }
    
    // Configure the serial port parameters
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        fprintf(stderr, "Failed to get serial port parameters.\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }
    
    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;
    
    if (!SetCommState(hSerial, &dcbSerialParams)) {
        fprintf(stderr, "Failed to set serial port parameters.\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }
    
    // Optional timeout settings
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    
    if (!SetCommTimeouts(hSerial, &timeouts)) {
        fprintf(stderr, "Failed to set serial port timeouts.\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }
    
    return hSerial;
}

int main(void) {
    // Use COM5 port (change the port name if necessary)
    HANDLE hSerial = open_serial_port("\\\\.\\COM5");
    if (hSerial == INVALID_HANDLE_VALUE) {
        return 1;
    }
    
    printf("Serial port (COM5) opened successfully.\n");
    printf("Waiting for commands... (Type 'exit' or 'quit' to terminate)\n");
    
    char input_line[256];
    while (1) {
        printf("Command: ");
        if (!fgets(input_line, sizeof(input_line), stdin)) {
            break;
        }
        
        // Remove any newline characters
        input_line[strcspn(input_line, "\r\n")] = 0;
        
        // Check for exit commands
        if (strcmp(input_line, "exit") == 0 || strcmp(input_line, "quit") == 0) {
            break;
        }
        
        process_command(input_line, hSerial);
        // After sending the command, read and display the response from the port.
        read_response(hSerial);
    }
    
    CloseHandle(hSerial);
    printf("Serial port closed. Exiting.\n");
    return 0;
}
