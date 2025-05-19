#include "i2c.h"
#include "sensors.h"

#define ITG3205_ADDR (0x68 << 1) // AD0 LOW ise 0x68, HIGH ise 0x69
#define ADXL345_ADDR (0x53 << 1) // SDO LOW ise 0x53
#define HMC5883L_ADDR (0x1E << 1)
#define BAR30_I2C_ADDR (0x76 << 1)

// MS5837 register/command definitions
#define MS5837_CMD_RESET        0x1E
#define MS5837_CMD_ADC_READ     0x00 // Her okumada bu adresi kullan
#define MS5837_CMD_CONVERT_D1_8192 0x4A
#define MS5837_CMD_CONVERT_D2_8192 0x5A
#define MS5837_CMD_PROM_READ    0xA0 // PROM okuma başlangıcı

#define HARD_IRON_X   (294.3650406f)
#define HARD_IRON_Y   (-1240.36481506f)
#define HARD_IRON_Z   (-566.6978993f)

const float soft_iron[3][3] = {
    {0.018215949f,  -0.0000906315f,  0.0001179308f},
    {-0.0000906315f,  0.018266505f,  0.0001862537f},
    {0.0001179308f,   0.0001862537f, 0.018167255f}
};

uint16_t C[8];   // PROM doğrulama katsayıları
uint32_t D1, D2;
int32_t TEMP_Bar30;
int32_t P;

// Manyetometre ofset ve ölçek faktörleri
float offset_x = 0, offset_y = 0, offset_z = 0;
float offset_x_mg = 0, offset_y_mg = 0, offset_z_mg = 0;
float scale_x = 1, scale_y = 1, scale_z = 1;

void BAR30_Read(float *depth)
{
    uint8_t buffer[3];

    // Basınç (D1) ölçümünü başlat
    HAL_I2C_Mem_Write(&hi2c1,
                      BAR30_I2C_ADDR,
                      MS5837_CMD_CONVERT_D1_8192,
                      I2C_MEMADD_SIZE_8BIT,
                      NULL, 0,
                      HAL_MAX_DELAY);
    osDelay(20);
    // https://discuss.bluerobotics.com/t/ms5837-fast-sampling/478/3 FIRAT ABİYE SOR BUNU DA HAL DELAY KISMINI VE OKUMA HIZINI

    // ADC verisini oku (basınç)
    HAL_I2C_Mem_Read(&hi2c1,
                     BAR30_I2C_ADDR,
                     MS5837_CMD_ADC_READ,
                     I2C_MEMADD_SIZE_8BIT,
                     buffer, sizeof(buffer),
                     HAL_MAX_DELAY);
    D1 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2]; // SADECE BASINCI OKUMAK İSTİYORUM

    // Sıcaklık (D2) ölçümünü başlat
    HAL_I2C_Mem_Write(&hi2c1,
                      BAR30_I2C_ADDR,
                      MS5837_CMD_CONVERT_D2_8192,
                      I2C_MEMADD_SIZE_8BIT,
                      NULL, 0,
                      HAL_MAX_DELAY);
    osDelay(20);

    // ADC verisini oku (sıcaklık)
    HAL_I2C_Mem_Read(&hi2c1,
                     BAR30_I2C_ADDR,
                     MS5837_CMD_ADC_READ,
                     I2C_MEMADD_SIZE_8BIT,
                     buffer, sizeof(buffer),
                     HAL_MAX_DELAY);
    D2 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2]; // SADECE SICAKLIĞI OKUMAK İSTİYORUM

    // 2048 OSR de MIN 3.72 MAX 4.54, ORTALAMA OLARAKTA 4.13 SANİYEDE DÖNÜŞÜM YAPIYOR.

    *depth = calculate();
}

int BAR30_init(void)
{ // GERİ DÖNÜŞ DEĞERİ BOOL OLARAK DEĞİŞTİRİLECEK
    uint8_t data[2];

    // RESET komutunu gönder
    HAL_I2C_Mem_Write(&hi2c1,
                      BAR30_I2C_ADDR,
                      MS5837_CMD_RESET,
                      I2C_MEMADD_SIZE_8BIT,
                      NULL, 0,
                      HAL_MAX_DELAY);
    osDelay(10);

    // PROM veri katsayılarını oku
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1,
                         BAR30_I2C_ADDR,
                         MS5837_CMD_PROM_READ + (i * 2),
                         I2C_MEMADD_SIZE_8BIT,
                         data, sizeof(data),
                         HAL_MAX_DELAY);

        C[i] = ((uint16_t)data[0] << 8) | data[1]; // 16 bitlik veriyi arraye depoluyoruz
    }

    // CRC kontrolü
    uint8_t crcCalculated = crc4(C); // DATASHEETTE AYNI KOD VAR
    uint8_t crcRead       = C[0] >> 12;

    if (crcCalculated != crcRead)
    { // CRC eşleşmiyor ise hata
        return 1;
    }

    return 0;
}

uint8_t crc4(uint16_t n_prom[])
{ // DATASHEETTE BÖYLE VERİLMİŞ
    uint16_t n_rem = 0;
    uint8_t crc = 0;

    n_prom[0] &= 0x0FFF;  // PROM verisinin 12 bitlik kısmını al

    for (uint8_t i = 0; i < 16; i++) {
        if (i & 1) {
            n_rem ^= (n_prom[i >> 1] & 0x00FF);
        } else {
            n_rem ^= (n_prom[i >> 1] >> 8);
        }

        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem <<= 1;
            }
        }
    }

    crc = (n_rem >> 12) & 0x0F;  // CRC'nin son 4 biti
    return crc ^ 0x00;
}

float calculate()
{
    // Değişken tanımlamaları
    int32_t dT = D2 - ((uint32_t)C[5] * 256L);
    int64_t SENS = ((int64_t)C[1] * 32768L) + (((int64_t)C[3] * dT) / 256L);
    int64_t OFF  = ((int64_t)C[2] * 65536L) + (((int64_t)C[4] * dT) / 128L);
    int32_t Ti = 0, OFFi = 0, SENSi = 0;
    int64_t OFF2, SENS2;

    P = (((D1 * SENS) / 2097152L - OFF) / 8192L) / 10;  // DİREKT OLARAK CM VERİYOR
    TEMP_Bar30 = 2000L + ((int64_t)dT * C[6] / 8388608LL);

    if (TEMP_Bar30 < 2000) {
        Ti    = (3 * (int64_t)dT * dT) / 8589934592LL;
        OFFi  = (3 * (TEMP_Bar30 - 2000) * (TEMP_Bar30 - 2000)) / 2;
        SENSi = (5 * (TEMP_Bar30 - 2000) * (TEMP_Bar30 - 2000)) / 8;
        if (TEMP_Bar30 < -1500) {
            OFFi  += 7 * (TEMP_Bar30 + 1500) * (TEMP_Bar30 + 1500);
            SENSi += 4 * (TEMP_Bar30 + 1500) * (TEMP_Bar30 + 1500);
        }
    } else {
        Ti    = (2 * (int64_t)dT * dT) / 137438953472LL;
        OFFi  = ((TEMP_Bar30 - 2000) * (TEMP_Bar30 - 2000)) / 16;
        SENSi = 0;
    }

    OFF2  = OFF  - OFFi;
    SENS2 = SENS - SENSi;
    TEMP_Bar30 -= Ti;

    P = (((D1 * SENS2) / 2097152L - OFF2) / 8192L) / 10.0;
    return P;
}

void ITG3205_Init(void)
{
    uint8_t data = 0x01; // Power Management: PLL, X ekseni referans
    HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDR, 0x3E, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    data = 0x18; // DLPF ve Full Scale Ayarları (2000dps, 256Hz)
    HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDR, 0x16, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

void ADXL345_Init(void)
{
    uint8_t data = 0x08; // Measurement Mode
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x2D, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    data = 0x09; // +/-4g, Full Resolution
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x31, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

void apply_mag_calibration(int16_t raw_x, int16_t raw_y, int16_t raw_z,
		int16_t *cal_x, int16_t *cal_y, int16_t *cal_z)
{
    float x = raw_x - HARD_IRON_X;
    float y = raw_y - HARD_IRON_Y;
    float z = raw_z - HARD_IRON_Z;

    *cal_x = soft_iron[0][0]*x + soft_iron[0][1]*y + soft_iron[0][2]*z;
    *cal_y = soft_iron[1][0]*x + soft_iron[1][1]*y + soft_iron[1][2]*z;
    *cal_z = soft_iron[2][0]*x + soft_iron[2][1]*y + soft_iron[2][2]*z;
}


void HMC5883L_Init(void) {
    uint8_t data;

    // Configuration Register A: 8 örnek, 15Hz, normal ölçüm
    data = 0xF0;
    HAL_I2C_Mem_Write(&hi2c1, 0x3C, 0x00, 1, &data, 1, 100);

    // Configuration Register B: Gain ayarı (örnek: ±1.3 Gauss → 1090 LSB/Gauss)
    data = 0x20;  // Gain = 1090 LSB/Gauss (default olan 1.3 Ga)
    HAL_I2C_Mem_Write(&hi2c1, 0x3C, 0x00, 1, &data, 1, 100);

    // Mode Register: Sürekli ölçüm modu
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, 0x3C, 0x02, 1, &data, 1, 100);

    osDelay(10);
}


void HMC5883L_Read(int16_t  *mx, int16_t  *my, int16_t  *mz) {
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(&hi2c1, 0x3D, 0x03, 1, buffer, 6, 100); // 6 byte okuyor kaydıra kaydıra görülüyor


    *mx = (buffer[0] << 8) | buffer[1];  // X ekseni
    *my = (buffer[4] << 8) | buffer[5];  // Y ekseni (HMC5883L datashette sıra X, Z, Y)
    *mz = (buffer[2] << 8) | buffer[3];  // Z ekseni
    *mx = *mx * 0.092f;
    *my = *my * 0.092f;
    *mz = *mz * 0.092f;

//    printf("%d,%d,%d\r\n", *mx, *my, *mz);
    osDelay(15); // BU KALKABİLİR


}

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

void ADXL345_Read(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, 0x32, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);
    *ax = (buffer[1] << 8) | buffer[0];
    *ay = (buffer[3] << 8) | buffer[2];
    *az = (buffer[5] << 8) | buffer[4];
}

void ITG3205_Read(int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buffer[8];
    HAL_I2C_Mem_Read(&hi2c1, ITG3205_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, buffer, 8, 100);
    *temp = (buffer[0] << 8) | buffer[1];
    *gx   = (buffer[2] << 8) | buffer[3];
    *gy   = (buffer[4] << 8) | buffer[5];
    *gz   = (buffer[6] << 8) | buffer[7];
}

void HMC5883L_Read_Calibrated(float *mx, float *my, float *mz)
{
    int16_t mxx, myy, mzz;
    HMC5883L_Read(&mxx, &myy, &mzz);

    *mx = (mxx - offset_x) / scale_x;
    *my = (myy - offset_y) / scale_y;
    *mz = (mzz - offset_z) / scale_z;
}
