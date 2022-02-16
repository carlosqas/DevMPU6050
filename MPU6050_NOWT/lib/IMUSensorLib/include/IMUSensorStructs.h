/**
 * @file IMUSensorStructs.h
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Arquivo destinado ao armazenamento de structs voltadas para o IMUSensor
 * @version 0.1
 * @date 18-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <vector>

#include "IMUSensorEnums.h" 

/**
 * @brief Dados de leitura dos eixos do IMU.
 * 
 */
struct IMUAxisData_t
{
public:
    IMUAxisData_t()
    {
        Time = 0;
        Temperature = 0.0;
        Acc_X = Acc_Y = Acc_Z = 0.0;
        Gyro_X = Gyro_Y = Gyro_Z = 0.0;
        Yaw = Pitch = Roll = 0.0;
    }

    /**
     * @brief Horário da leitura.
     * 
     */
    unsigned long Time;
    /**
     * @brief Temperatura do sensor.
     * 
     */
    double Temperature;
    /**
     * @brief Eixo X do acelerômetro.
     * 
     */
    double Acc_X;
    /**
     * @brief Eixo Y do acelerômetro.
     * 
     */
    double Acc_Y;
    /**
     * @brief Eixo Z do acelerômetro.
     * 
     */
    double Acc_Z;
    /**
     * @brief Eixo X do giroscópio.
     * 
     */
    double Gyro_X;
    /**
     * @brief Eixo Y do giroscópio.
     * 
     */
    double Gyro_Y;
    /**
     * @brief Eixo Z do giroscópio.
     * 
     */
    double Gyro_Z;    

    /**
     * @brief Ângulo do eixo de controle
     * Yaw.
     */
    double Yaw;

    /**
     * @brief Ângulo do eixo de controle
     * Pitch.
     */
    double Pitch;

    /**
     * @brief Ângulo do eixo de controle
     * Roll.
     */
    double Roll;
};

/**
 * @brief Dados do tombamento.
 * 
 */
struct IMUTippingData_t
{
public:
    /**
     * @brief Lado que houve o tombamento.
     * 
     */
    IMUTippingSide_e Side;

    /**
     * @brief Há quanto tempo começou o tombamento.
     * 
     */
    unsigned long StartTime;

    /**
     * @brief Vetor circular contendo histórico de leitura
     * dos sensores durante o evento.
     */
    std::vector<IMUAxisData_t> AxisMeasurements;
    
};

/**
 * @brief Dados de início de movimentação.
 * 
 */
struct IMUMovementData_t
{
public:
    /**
     * @brief Tempo de início.
     * 
     */
    unsigned long StartTime;
};

/**
 * @brief Dados de parada.
 * 
 */
struct IMUStopData_t
{
public:
    /**
     * @brief Tempo de início.
     * 
     */
    unsigned long StartTime;
};

/**
 * @brief Dados de tamper.
 * 
 */
struct IMUTamperData_t
{
public:
    /**
     * @brief Tempo de início.
     * 
     */
    unsigned long StartTime;
};

/**
 * @brief Configurações de detecção de tombamento.
 * 
 */
struct IMUTippingSettings_t
{
public:
    /**
     * @brief Constrói um novo objeto da struct IMUTippingSettings_t.
     * 
     */
    IMUTippingSettings_t()
    {
        TippingStartThreshold = 0.0;
        MinimumSamples = 0.0;
    }

    /**
     * @brief Limiar em graus para início da detecção de tombamento.
     * 
     */
    double TippingStartThreshold;

    /**
     * @brief Amostragem mínima para determinar se é um tombamento.
     * 
     */
    uint16_t MinimumSamples;
};

/**
 * @brief Configurações para detecção de movimento.
 * 
 */
struct IMUMovementSettings_t
{
public:
    /**
     * @brief Constrói um novo objeto da struct IMUMovementSettings_t.
     * 
     */
    IMUMovementSettings_t()
    {
        MovementInterval = 0.0;
        MinimumSamples = 0;
    }

    /**
     * @brief Intervalo que o módulo da aceleração deve ultrapassar
     * para identificar um movimento.
     * Accell > (1g + Interval) && Accel < (1g - Interval)
     */
    double MovementInterval;

    /**
     * @brief Quantidade mínima de amostras para confirmar que está em movimento.
     * 
     */
    uint8_t MinimumSamples;
};

/**
 * @brief Configurações para detecção de parada.
 * 
 */
struct IMUStopSettings_t
{
public:
    /**
     * @brief Constrói um novo objeto da struct IMUStopSettings_t.
     * 
     */
    IMUStopSettings_t()
    {
        StopInterval = 0.0;
        MinimumSamples = 0;
    }

    /**
     * @brief Intervalo que o módulo da aceleração deve ficar contido
     * para identificar que está parado.
     * Accell < (1g + Interval) && Accel > (1g - Interval)
     */
    double StopInterval;

    /**
     * @brief Quantidade mínima de amostras para confirmar que está parando.
     * 
     */
    uint8_t MinimumSamples;
};

/**
 * @brief Configurações para a detecção de tamper.
 * 
 */
struct IMUTamperSettings_t
{
public:
    /**
     * @brief Constrói um objeto da struct IMUTamperSettings_t.
     * 
     */
    IMUTamperSettings_t()
    {
        TamperTime = 0;
        MinimumSamples = 0;
    }

    /**
     * @brief Se o automóvel tombar e continuar em movimento por (TamperTime)
     * segundos, irá detectar tamper.
     */
    uint8_t TamperTime;

    /**
     * @brief Quantidade mínima de amostras para confirmar que está em
     * tamper.
     */
    uint8_t MinimumSamples;
};

struct IMUOffsets_t
{
public:
    IMUOffsets_t()
    {
        XAccelOffset = 0;
        YAccelOffset = 0;
        ZAccelOffset = 0;
        XGyroOffset = 0;
        YGyroOffset = 0;
        ZGyroOffset = 0;
    }

    IMUOffsets_t(int16_t xAccelOffset, int16_t yAccelOffset, int16_t zAccelOffset, 
              int16_t xGyroOffset, int16_t yGyroOffset, int16_t zGyroOffset)
    {
        XAccelOffset = xAccelOffset;
        YAccelOffset = yAccelOffset;
        ZAccelOffset = zAccelOffset;
        XGyroOffset = xGyroOffset;
        YGyroOffset = yGyroOffset;
        ZGyroOffset = zGyroOffset;
    }

    /**
     * @brief Offset do acelerômetro no eixo X.
     * 
     */
    int16_t XAccelOffset;
    /**
     * @brief Offset do acelerômetro no eixo Y.
     * 
     */
    int16_t YAccelOffset;
    /**
     * @brief Offset do acelerômetro no eixo Z.
     * 
     */    
    int16_t ZAccelOffset;
    /**
     * @brief Offset do giroscópio no eixo X.
     * 
     */    
    int16_t XGyroOffset;
    /**
     * @brief Offset do giroscópio no eixo Y.
     * 
     */    
    int16_t YGyroOffset;
    /**
     * @brief Offset do giroscópio no eixo Z.
     * 
     */    
    int16_t ZGyroOffset;
};