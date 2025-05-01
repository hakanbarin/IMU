#include <stdio.h>
#include <stdint.h>
#include <windows.h>
#include <string.h>
#include "Core/Inc/rpc.h"

#define COM_PORT      "COM5"
#define BAUD_RATE     115200
#define RPC_LED_CONTROL_ID 1


// #pragma pack(push, 1)

// typedef struct {
//     uint8_t led_aktif; // 1 byte
// } led_kontrol;

// // Test için 4 byte’lık service ve 16 byte union
// typedef struct {
//     uint8_t service;  // STM32’de enum 1 byte ama padding ile 4 byte olabilir
//     union {
//         led_kontrol p7;
//         uint8_t raw[16];  // union: 16 byte → test amaçlı byte byte doldurulacak
//     } data;
// } rpc_message_t;

// #pragma pack(pop)

void print_packet_hex(const rpc_message_t *msg) {
    const uint8_t *bytes = (const uint8_t *)msg;
    printf("📦 Gönderilen veri (hex dump):\n");
    for (size_t i = 0; i < sizeof(rpc_message_t); i++) {
        printf("[%02zu] = 0x%02X\n", i, bytes[i]);
    }
}

int send_led_control(int led_on_raw)
{
    uint8_t led_on = (uint8_t)led_on_raw;
    HANDLE hSerial = CreateFileA(COM_PORT, GENERIC_WRITE, 0, NULL,
                                 OPEN_EXISTING, 0, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        printf("❌ Port açılamadı: %s\n", COM_PORT);
        return -1;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hSerial, &dcb);
    dcb.BaudRate = BAUD_RATE;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;  
    SetCommState(hSerial, &dcb);

    // uint8_t msg[sizeof(rpc_message_t)];

    // for(uint8_t i = 0; i < sizeof(rpc_message_t); i++){

    //     msg[i] = i;

    // }


    rpc_message_t msg = {0};
    // memset(&msg, 0, sizeof(rpc_message_t));
    // ——————————— Alanları DOLDURALIM

    msg.service = PWM_MOTORS_FOR_STOP;       // 0x06  → 4 byte (0x06, 0x00, 0x00, 0x00)

    // // // union alanı 16 byte  ilk byte ‘led_aktif’ olarak kullanılacak

    // msg.data.p7.led_aktif = led_on;
    msg.data.p6.pwm1 = (56);
    msg.data.p6.pwm2 = (15);
    msg.data.p6.pwm3 = (325);

    // for (int i = 1; i < 19; i++) {
    //     msg.data.raw[i] = 0xAA + i;         // diğer alanlara ayırt edici değerler (0xAB, 0xAC, ...)
    // }

    printf("📏 sizeof(rpc_message_t): %zu (beklenen: 20)\n", sizeof(msg));
    print_packet_hex(&msg);

    DWORD written = 0;
    WriteFile(hSerial, &msg, sizeof(msg), &written, NULL);
    // if (written != sizeof(msg)) {
    //     printf("❌ Yazılan byte sayısı hatalı! %lu yerine %zu byte yazıldı.\n", written, sizeof(msg));
    // }
    CloseHandle(hSerial);
    

    // printf("✅ Gönderildi: service = %d, led_aktif = %d, yazılan = %lu byte\n",
    //        msg.service, msg.data.p7.led_aktif, written);

    
    return 0;
}

int main()
{
    return send_led_control(35);  
}





/*
// #include <stdio.h>
// #include <stdint.h>
// #include <string.h>
// #include <windows.h>

// #define COM_PORT      "COM5"        // kendi portunu yaz
// #define BAUD_RATE     115200

// // —————————————————————— Servis ID tanımı
// #define RPC_LED_CONTROL_ID    6     // sırayla 0–6 gidiyor, led_control son servis

// // —————————————————————— Gönderilecek veri yapısı
// typedef struct
// {
    //     uint8_t service;
    //     union {
        //         struct {
            //             uint8_t led_aktif;
            //         } payload;
            //         uint8_t raw[31];  // toplam 32 byte olacak, padding için
            //     };
            // } rpc_message_t;


            // // —————————————————————— UART ile STM32’ye gönder
            // int send_rpc_led_control(uint8_t led_on)
            // {
                //     HANDLE hSerial = CreateFileA(COM_PORT, GENERIC_WRITE, 0, NULL,
                //                                  OPEN_EXISTING, 0, NULL);

                //     if (hSerial == INVALID_HANDLE_VALUE) {
                    //         fprintf(stderr, "Port açılamadı: %s\n", COM_PORT);
                    //         return -1;
                    //     }

                    //     DCB dcbSerialParams = {0};
                    //     dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

                    //     if (!GetCommState(hSerial, &dcbSerialParams)) {
                        //         fprintf(stderr, "Baudrate ayarı okunamadı.\n");
                        //         CloseHandle(hSerial);
                        //         return -1;
                        //     }

                        //     dcbSerialParams.BaudRate = BAUD_RATE;
                        //     dcbSerialParams.ByteSize = 8;
                        //     dcbSerialParams.StopBits = ONESTOPBIT;
                        //     dcbSerialParams.Parity   = NOPARITY;

                        //     SetCommState(hSerial, &dcbSerialParams);

                        //     // — Paket oluştur
                        //     rpc_message_t msg = {0};
                        //     msg.service = RPC_LED_CONTROL_ID;
                        //     msg.payload.led_aktif = led_on ? 1 : 0;

                        //     DWORD bytes_written;
                        //     BOOL success = WriteFile(hSerial, &msg, sizeof(msg), &bytes_written, NULL);

                        //     if (!success || bytes_written != sizeof(msg)) {
                            //         fprintf(stderr, "Yazma hatası.\n");
                            //         CloseHandle(hSerial);
                            //         return -1;
                            //     }

                            //     CloseHandle(hSerial);
                            //     return 0;
                            // }

                            // // —————————————————————— Main örneği
                            // int main()
                            // {
                                //     uint8_t led_aktif = 1;
                                //     send_rpc_led_control(led_aktif);
                                //     // while (1)
                                //     // {

                                //     //     if (result == 0)
                                //     //         printf("Gönderildi: LED = %d\n", led_aktif);
                                //     //     else
                                //     //         printf("Gönderim HATASI!\n");

                                //     //     Sleep(500);  // 500ms bekle, yoksa STM32 yetişemez
                                //     // }

                                //     return 0;
                                // }

*/
