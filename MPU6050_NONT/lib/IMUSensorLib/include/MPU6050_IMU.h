/**
 * @file MPU6050_IMU.h
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Classe para o sensor MPU6050
 * @version 0.1
 * @date 18-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include "IMUSensor.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define MPU6050_PIN_SDA GPIO_NUM_33 // Pino de comunicação com MPU6050
#define MPU6050_PIN_SCL GPIO_NUM_32 // Pino de comunicação com MPU6050
#define MPU6050_FREQUENCY 400000    // Frequência de comunicação com MPU6050

/**
 * @brief Classe com os métodos para o sensor
 * MPU6050.
 */
class MPU6050IMU : public IMUSensor
{
public:
    /**
     * @brief Constrói um novo objeto da classe
     * MPU6050IMU.
     */
    MPU6050IMU();

    /**
     * @brief Configura os offsets e chama a inicialização da MPU6050.
     * @param wire Interface I2C que comunica com a MPU. 
     * @return true - Caso inicie normalmente.
     * @return false - Caso contrário.
     */
    bool begin(TwoWire &wire);

    /**
     * @brief Inicializa e faz as configuralções iniciais da MPU6050.
     * 
     * @param wire Interface I2C que comunica com a MPU.
     * @param offsets Offsets.
     * @return true - Caso inicie normalmente.
     * @return false - Caso contrário.
     */
    bool begin(TwoWire &wire, IMUOffsets_t offsets);

    /**
     * @brief Faz a leitura e armazena os dados provenientes do sensor.
     * 
     */
    void acquireData();

    /**
     * @brief Calibrar o sensor.
     * 
     * @return IMUOffsets_t - Offsets calculados à 
     * partir da calibração.
     */
    IMUOffsets_t calibrate();

    /**
     * @brief Obtem os offsets atuais do sensor.
     * 
     * @return IMUOffsets_t - Offsets atuais do sensor.
     */
    IMUOffsets_t getCurrentOffsets();

    /**
     * @brief Altera os offsets do dispositivo.
     * 
     * @param newOffsets Novos offsets que serão utilizados
     * pelo dispositivo
     */
    void setOffsets(IMUOffsets_t newOffsets);

private:
    MPU6050 m_mpu;               // Objeto da classe MPU6050 utilizado para acessar os métodos da lib.          
    uint8_t m_deviceStatus;      // Status de funcionamento dispositivo (== 0 -> Funcionando).
    bool m_dmpStatus;            // Status de funcionamento do DMP.
    int16_t m_deviceOffsets[6];  // Offsets configurados para o dispositivo [aX, aY, aZ, gX, gY, gZ].
};

extern MPU6050IMU MPU;