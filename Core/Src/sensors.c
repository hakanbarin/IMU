#include "i2c.h"
#include "sensors.h"

#define ITG3205_ADDR (0x68 << 1) // AD0 LOW ise 0x68, HIGH ise 0x69
#define ADXL345_ADDR (0x53 << 1) // SDO LOW ise 0x53
#define HMC5883L_ADDR (0x1E << 1)
#define BAR30_I2C_ADDR 0x76 << 1
const uint8_t MS5837_ADDR_Read = 0xEC;
const uint8_t MS5837_ADDR_Write = 0xED;
const uint8_t MS5837_RESET = 0x1E;
uint8_t MS5837_ADC_READ = 0x00;        // Her okumada bu adrese yazılması gerekiyor
const uint8_t MS5837_PROM_READ = 0xA0; // PROM okuma adresi
uint8_t MS5837_CONVERT_D1_8192 = 0x4A; // BURALARI DA SOR FIRAT ABİYE
uint8_t MS5837_CONVERT_D2_8192 = 0x5A;

uint16_t C[7]; // PROM(doğrulama) için array 2 bytelık verilerle doğrulama yapıyoruz ondan dolayı 16lık

uint32_t D1, D2;
int32_t TEMP_Bar30;
int32_t P;

// Manyetometre ofset ve ölçek faktörleri
float offset_x = 0, offset_y = 0, offset_z = 0;
float offset_x_mg = 0, offset_y_mg = 0, offset_z_mg = 0;
float scale_x = 1, scale_y = 1, scale_z = 1;

void BAR_30_Read(float *depth)
{
    uint8_t buffer[3];
    HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR_Write, &MS5837_CONVERT_D1_8192, 1, HAL_MAX_DELAY);
    HAL_Delay(20);
    // https://discuss.bluerobotics.com/t/ms5837-fast-sampling/478/3 FIRAT ABİYE SOR BUNU DA HAL DELAY KISMINI VE OKUMA HIZINI

    //	The conversion command is used to initiate uncompensated pressure (D1) or uncompensated temperature (D2)
    //	conversion. After the conversion, using ADC read command the result is clocked out with the MSB first. If the
    //	conversion is not executed before the ADC read command, or the ADC read command is repeated, it will give 0
    //	as the output result. If the ADC read command is sent during conversion the result will be 0, the conversion will
    //	not stop and the final result will be wrong.  DATASHEETTE YAZIYOR.
    HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR_Write, &MS5837_ADC_READ, 1, HAL_MAX_DELAY); // ADC READ İÇİN

    HAL_I2C_Master_Receive(&hi2c1, MS5837_ADDR_Read, buffer, 3, HAL_MAX_DELAY);
    D1 = (buffer[0] << 16 | buffer[1] << 8 | buffer[2]); // SADECE BASINCI OKUMAK İSTİYORUM

    HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR_Write, &MS5837_CONVERT_D2_8192, 1, HAL_MAX_DELAY);
    HAL_Delay(20);

    HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR_Write, &MS5837_ADC_READ, 1, HAL_MAX_DELAY); // ADC READ İÇİN

    HAL_I2C_Master_Receive(&hi2c1, MS5837_ADDR_Read, buffer, 3, HAL_MAX_DELAY);
    D2 = (buffer[0] << 16 | buffer[1] << 8 | buffer[2]); // SADECE BASINCI OKUMAK İSTİYORUM

    // 2048 OSR de MIN 3.72 MAX 4.54, ORTALAMA OLARAKTA 4.13 SANİYEDE DÖNÜŞÜM YAPIYOR.

    *depth = calculate();
}

int BAR30_init(void)
{ // GERİ DÖNÜŞ DEĞERİ BOOL OLARAK DEĞİŞTİRİLECEK
    uint8_t cmd = MS5837_PROM_READ;
    uint8_t data[2];

    HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR_Write, &cmd, 1, 100); // RESET YOLLUYORUZ
    HAL_Delay(10);

    for (uint8_t i = 0; i < 7; i++)
    {

        cmd = cmd + (i * 2);
        HAL_I2C_Master_Transmit(&hi2c1, 0xEC, &cmd, 1, HAL_MAX_DELAY); // BURALARI FIRAT ABİYE SOR EC Mİ KULALNMAK GEREKİYOR 0x76 MI?
        HAL_I2C_Master_Receive(&hi2c1, 0xED, data, 2, HAL_MAX_DELAY);

        C[i] = (data[0] << 8) | data[1]; // 16 bitlik veriyi arraye depoluyoruz
    }

    uint8_t crcCalculated = crc4(C); // BURDA CRC KARŞILAŞTIRMASI YAPIYORUZ
    uint8_t crcRead = C[0] >> 12;    // BURDA CRC KARŞILAŞTIRMASI YAPIYORUZ

    if (crcCalculated != crcRead)
    { // BURDA CRC KARŞILAŞTIRMASI YAPIYORUZ
        return 1;
    }

    return 0;
}

uint8_t crc4(uint16_t n_prom[]) { //BU KODA BAK DİREKT OLARAK DATASHEETTE AYNI KOD VAR
    uint16_t n_rem = 0;
    uint8_t crc = 0;

    n_prom[0] = (n_prom[0]) & 0x0FFF;  // PROM verisinin 12 bitlik kısmını al

    for (uint8_t i = 0; i < 16; i++) {
        if (i % 2 == 1) {
            n_rem ^= (n_prom[i >> 1] & 0x00FF);
        } else {
            n_rem ^= (n_prom[i >> 1] >> 8);
        }

        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    crc = (n_rem >> 12) & 0x000F;  // CRC'nin son 4 bitini al
    return crc ^ 0x00;
}


float calculate()
{
    // Değişken tanımlamaları
    int32_t dT = 0;
    int64_t SENS = 0, OFF = 0;
    int32_t SENSi = 0, OFFi = 0, Ti = 0;
    int64_t OFF2 = 0, SENS2 = 0;

    // dT hesaplama (D2 sıcaklık ölçümü ile düzeltme katsayısı arasındaki fark)
    dT = D2 - ((uint32_t)C[5] * 256L);

    SENS = ((int64_t)C[1] * 32768L) + (((int64_t)C[3] * dT) / 256L);
    OFF = ((int64_t)C[2] * 65536L) + (((int64_t)C[4] * dT) / 128L);
    P = ((D1 * SENS / 2097152L) - OFF) / 8192L;

    // Sıcaklık hesaplama
    TEMP_Bar30 = 2000L + ((int64_t)dT * C[6] / 8388608LL);

    if (TEMP_Bar30 < 2000)
    { // Düşük sıcaklık telafisi
        Ti = (3 * (int64_t)dT * dT) / 8589934592LL;
        OFFi = (3 * (TEMP_Bar30 - 2000) * (TEMP_Bar30 - 2000)) / 2;
        SENSi = (5 * (TEMP_Bar30 - 2000) * (TEMP_Bar30 - 2000)) / 8;
        if (TEMP_Bar30 < -1500)
        { // Çok düşük sıcaklık telafisi
            OFFi += 7 * (TEMP_Bar30 + 1500) * (TEMP_Bar30 + 1500);
            SENSi += 4 * (TEMP_Bar30 + 1500) * (TEMP_Bar30 + 1500);
        }
    }
    else
    { // Yüksek sıcaklık telafisi
        Ti = (2 * (int64_t)dT * dT) / 137438953472LL;
        OFFi = (TEMP_Bar30 - 2000) * (TEMP_Bar30 - 2000) / 16;
        SENSi = 0;
    }

    // Telafi edilmiş değerleri hesapla
    OFF2 = OFF - OFFi;
    SENS2 = SENS - SENSi;
    TEMP_Bar30 -= Ti;

    // Telafi edilmiş basınç hesaplama

    P = (((D1 * SENS2) / 2097152L - OFF2) / 8192L) / 10;
    return P;
    // DİREKT OLARAK CM VERİYOR
    // BUNLARIN HEPSİ DATASHEETTE VAR
}

void ITG3205_Init(void)
{
    uint8_t data[2];
    // Power Management: PLL, X ekseni referans
    data[0] = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDR, 0x3E, 1, data, 1, 100);
    // DLPF ve Full Scale Ayarları (2000dps, 256Hz)
    data[0] = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDR, 0x16, 1, data, 1, 100);
}

void ADXL345_Init(void)
{
    uint8_t data[2];
    // Measurement Mode
    data[0] = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x2D, 1, data, 1, 100);
    // +/-4g, Full Resolution
    data[0] = 0x09;
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x31, 1, data, 1, 100);
}

void HMC5883L_Init(void)
{
    uint8_t data[2];
    // Ölçüm Sıklığı ve Mod (75Hz, Sürekli)
    data[0] = 0xF0; // 8 örnek, 75Hz
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x00, 1, data, 1, 100);
    data[0] = 0x00; // Sürekli ölçüm
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, data, 1, 100);
}

void HMC5883L_Read(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x03, 1, buffer, 6, 100);
    *mx = (buffer[0] << 8) | buffer[1]; // X ekseni
    *my = (buffer[4] << 8) | buffer[5]; // Y ekseni (HMC5883L veri sayfasına göre sıra X, Z, Y)
    *mz = (buffer[2] << 8) | buffer[3]; // Z ekseni
}

void ADXL345_Read(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, 0x32, 1, buffer, 6, 100);
    *ax = (buffer[1] << 8) | buffer[0];
    *ay = (buffer[3] << 8) | buffer[2];
    *az = (buffer[5] << 8) | buffer[4];
}

void ITG3205_Read(int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buffer[8];
    HAL_I2C_Mem_Read(&hi2c1, ITG3205_ADDR, 0x1B, 1, buffer, 8, 100);
    *temp = (buffer[0] << 8) | buffer[1];
    *gx = (buffer[2] << 8) | buffer[3];
    *gy = (buffer[4] << 8) | buffer[5];
    *gz = (buffer[6] << 8) | buffer[7];
}

void HMC5883L_Read_Calibrated(float *mx, float *my, float *mz)
{
    int16_t mxx, myy, mzz;
    HMC5883L_Read(&mxx, &myy, &mzz);

    *mx = (mxx - offset_x) / scale_x;
    *my = (myy - offset_y) / scale_y;
    *mz = (mzz - offset_z) / scale_z;
}
