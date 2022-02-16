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
     * @brief Inicializa e faz a configuração inicial da
     * MPU6050.
     * @param wire Interface I2C que comunica com a MPU. 
     * @return true - Caso inicie normalmente.
     * @return false - Caso contrário.
     */
    bool begin(TwoWire &wire);

    /**
     * @brief Iniciar a thread que realiza as medições.
     * 
     * @param frequency Frequência que a thread irá executar as medições (a cada x milissegundos).
     */
    void start(int frequency);

    /**
     * @brief Parar a thread que realiza as medições.
     * 
     */
    void stop();

private:
    /**
     * @brief Calibrar o sensor.
     * 
     * @return true Caso a calibração seja bem sucedida.
     * @return false Caso contrário.
     */
    bool calibrate();
    
    /**
     * @brief Faz a leitura e o armazenamento de novos dados
     * dos sensores da MPU (Já convertidos).
     */
    void updateData();

    /**
     * @brief Função superficial que permite acessar a 
     * função base para a realização da readTask.
     */
    static void wrapper(void * parameter);
    
    MPU6050 m_mpu;                      // Objeto da classe MPU6050 utilizado para acessar os métodos da lib.          
    uint8_t m_deviceStatus;             // Status de funcionamento dispositivo (== 0 -> Funcionando).
    bool m_dmpStatus;                   // Status de funcionamento do DMP.
    int16_t m_deviceOffsets[6];         // Offsets configurados para o dispositivo [aX, aY, aZ, gX, gY, gZ].
};