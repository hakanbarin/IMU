
//
//uint16_t C[8];   // PROM doğrulama katsayıları
//uint32_t D1, D2;
//int32_t TEMP_Bar30;
//int32_t P;


//void BAR30_Read(float *depth)
//{
//    uint8_t buffer[3];
//
//    // Basınç (D1) ölçümünü başlat
//    HAL_I2C_Mem_Write(&hi2c1,
//                      BAR30_I2C_ADDR,
//                      MS5837_CMD_CONVERT_D1_8192,
//                      I2C_MEMADD_SIZE_8BIT,
//                      NULL, 0,
//                      HAL_MAX_DELAY);
//    osDelay(20);
//    // https://discuss.bluerobotics.com/t/ms5837-fast-sampling/478/3 FIRAT ABİYE SOR BUNU DA HAL DELAY KISMINI VE OKUMA HIZINI
//
//    // ADC verisini oku (basınç)
//    HAL_I2C_Mem_Read(&hi2c1,
//                     BAR30_I2C_ADDR,
//                     MS5837_CMD_ADC_READ,
//                     I2C_MEMADD_SIZE_8BIT,
//                     buffer, sizeof(buffer),
//                     HAL_MAX_DELAY);
//    D1 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2]; // SADECE BASINCI OKUMAK İSTİYORUM
//
//    // Sıcaklık (D2) ölçümünü başlat
//    HAL_I2C_Mem_Write(&hi2c1,
//                      BAR30_I2C_ADDR,
//                      MS5837_CMD_CONVERT_D2_8192,
//                      I2C_MEMADD_SIZE_8BIT,
//                      NULL, 0,
//                      HAL_MAX_DELAY);
//    osDelay(20);
//
//    // ADC verisini oku (sıcaklık)
//    HAL_I2C_Mem_Read(&hi2c1,
//                     BAR30_I2C_ADDR,
//                     MS5837_CMD_ADC_READ,
//                     I2C_MEMADD_SIZE_8BIT,
//                     buffer, sizeof(buffer),
//                     HAL_MAX_DELAY);
//    D2 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2]; // SADECE SICAKLIĞI OKUMAK İSTİYORUM
//
//    // 2048 OSR de MIN 3.72 MAX 4.54, ORTALAMA OLARAKTA 4.13 SANİYEDE DÖNÜŞÜM YAPIYOR.
//
//    *depth = calculate();
//}
//
//int BAR30_init(void)
//{ // GERİ DÖNÜŞ DEĞERİ BOOL OLARAK DEĞİŞTİRİLECEK
//    uint8_t data[2];
//
//    // RESET komutunu gönder
//    HAL_I2C_Mem_Write(&hi2c1,
//                      BAR30_I2C_ADDR,
//                      MS5837_CMD_RESET,
//                      I2C_MEMADD_SIZE_8BIT,
//                      NULL, 0,
//                      HAL_MAX_DELAY);
//    osDelay(10);
//
//    // PROM veri katsayılarını oku
//    for (uint8_t i = 0; i < 8; i++)
//    {
//        HAL_I2C_Mem_Read(&hi2c1,
//                         BAR30_I2C_ADDR,
//                         MS5837_CMD_PROM_READ + (i * 2),
//                         I2C_MEMADD_SIZE_8BIT,
//                         data, sizeof(data),
//                         HAL_MAX_DELAY);
//
//        C[i] = ((uint16_t)data[0] << 8) | data[1]; // 16 bitlik veriyi arraye depoluyoruz
//    }
//
//    // CRC kontrolü
//    uint8_t crcCalculated = crc4(C); // DATASHEETTE AYNI KOD VAR
//    uint8_t crcRead       = C[0] >> 12;
//
//    if (crcCalculated != crcRead)
//    { // CRC eşleşmiyor ise hata
//        return 1;
//    }
//
//    return 0;
//}
//
//uint8_t crc4(uint16_t n_prom[])
//{ // DATASHEETTE BÖYLE VERİLMİŞ
//    uint16_t n_rem = 0;
//    uint8_t crc = 0;
//
//    n_prom[0] &= 0x0FFF;  // PROM verisinin 12 bitlik kısmını al
//
//    for (uint8_t i = 0; i < 16; i++) {
//        if (i & 1) {
//            n_rem ^= (n_prom[i >> 1] & 0x00FF);
//        } else {
//            n_rem ^= (n_prom[i >> 1] >> 8);
//        }
//
//        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
//            if (n_rem & 0x8000) {
//                n_rem = (n_rem << 1) ^ 0x3000;
//            } else {
//                n_rem <<= 1;
//            }
//        }
//    }
//
//    crc = (n_rem >> 12) & 0x0F;  // CRC'nin son 4 biti
//    return crc ^ 0x00;
//}
//
//float calculate()
//{
//    // Değişken tanımlamaları
//    int32_t dT = D2 - ((uint32_t)C[5] * 256L);
//    int64_t SENS = ((int64_t)C[1] * 32768L) + (((int64_t)C[3] * dT) / 256L);
//    int64_t OFF  = ((int64_t)C[2] * 65536L) + (((int64_t)C[4] * dT) / 128L);
//    int32_t Ti = 0, OFFi = 0, SENSi = 0;
//    int64_t OFF2, SENS2;
//
//    P = (((D1 * SENS) / 2097152L - OFF) / 8192L) / 10;  // DİREKT OLARAK CM VERİYOR
//    TEMP_Bar30 = 2000L + ((int64_t)dT * C[6] / 8388608LL);
//
//    if (TEMP_Bar30 < 2000) {
//        Ti    = (3 * (int64_t)dT * dT) / 8589934592LL;
//        OFFi  = (3 * (TEMP_Bar30 - 2000) * (TEMP_Bar30 - 2000)) / 2;
//        SENSi = (5 * (TEMP_Bar30 - 2000) * (TEMP_Bar30 - 2000)) / 8;
//        if (TEMP_Bar30 < -1500) {
//            OFFi  += 7 * (TEMP_Bar30 + 1500) * (TEMP_Bar30 + 1500);
//            SENSi += 4 * (TEMP_Bar30 + 1500) * (TEMP_Bar30 + 1500);
//        }
//    } else {
//        Ti    = (2 * (int64_t)dT * dT) / 137438953472LL;
//        OFFi  = ((TEMP_Bar30 - 2000) * (TEMP_Bar30 - 2000)) / 16;
//        SENSi = 0;
//    }
//
//    OFF2  = OFF  - OFFi;
//    SENS2 = SENS - SENSi;
//    TEMP_Bar30 -= Ti;
//
//    P = (((D1 * SENS2) / 2097152L - OFF2) / 8192L) / 10.0;
//    return P;
//}

//void rpc_thread(void *argument)
//{
//    (void)argument;
//    rpc_message_t msg;
//
//    printf("rpc_thread başladı. rpc_message_t boyutu: %d\n", sizeof(rpc_message_t));
//    HAL_UART_Receive_DMA(&huart2,(uint8_t *)rpc_rx_buffer, sizeof(rpc_rx_buffer));
//
//
//    while (1)
//    {
//        // Kuyruktan veri gelene kadar bekle
//        xQueueReceive(rpc_queueHandle, &msg, portMAX_DELAY);
//        RPC_SERVICE_MAP[msg.service](&msg.data);
////        uint8_t *bytes = (uint8_t *)&msg;
////        led_kontrol* input = &msg.data;
////        printf("kp = %f, ki = %f, kd = %f \r\n", input->kp, input->ki, input->kd);
////        printf("kp = %d\r\n", input->led_aktif);
//    }
//}

//void rpc_thread(void *argument) {
//	osDelay(20000);
//    rpc_message_t msg;
//
//    printf("rpc_thread başladı (rpc_message_t size = %u)\r\n",
//           (unsigned)sizeof(rpc_message_t));
//
//    // İlk DMA alımını başlat
//    HAL_UART_Receive_DMA(&huart2,
//                         (uint8_t*)rpc_rx_buffer,
//                         sizeof(rpc_message_t));
//
//    for (;;) {
//        // Mesaj gelene kadar tamamen blokla (CPU başka thread’e gider)
//        if (osMessageQueueGet(rpc_queueHandle,
//                              &msg,
//                              NULL,
//                              osWaitForever) == osOK) {
//
//		RPC_SERVICE_MAP[msg.service](&msg.data);
//
//        }
//    }
//}

//void HMC5883L_Init(void)
//{
//    uint8_t data = 0xF0; // Ölçüm Sıklığı ve Mod (75Hz, Sürekli)
//    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
//
//    data = 0x00; // Sürekli ölçüm
//    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
//}
//
//void HMC5883L_Read(int16_t *mx, int16_t *my, int16_t *mz)
//{
//    uint8_t buffer[6];
//    HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x03, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);
//    *mx = (buffer[0] << 8) | buffer[1];  // X ekseni
//    *my = (buffer[4] << 8) | buffer[5];  // Y ekseni (sırası X,Z,Y)
//    *mz = (buffer[2] << 8) | buffer[3];  // Z ekseni
//}

