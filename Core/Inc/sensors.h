#ifndef SENSORS_H
#define SENSORS_H

#ifdef __cplusplus
extern "C"
{
#endif

    void ITG3205_Init(void);

    void ADXL345_Init(void);

    void HMC5883L_Init(void);

    void HMC5883L_Read(int16_t* mx, int16_t* my, int16_t* mz);

    void ADXL345_Read(int16_t* ax, int16_t* ay, int16_t* az);
    
    void ITG3205_Read(int16_t* temp, int16_t* gx, int16_t* gy, int16_t* gz);

    void HMC5883L_Read_Calibrated(float* mx, float* my, float* mz);

    void BAR_30_Read();

    int BAR30_init(void); // return value bool yapılacak

    void calculate();

    uint8_t crc4(uint16_t);

#ifdef __cplusplus
}
#endif

#endif // SENSORS_H
