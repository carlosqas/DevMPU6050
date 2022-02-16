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
 * @brief Configurações de detecção de tombamento.
 * 
 */
struct IMUTippingSettings_t
{
public:
    /**
     * @brief Limiar em graus para início da detecção de tombamento.
     * 
     */
    double TippingStartThreshold;

    /**
     * @brief Amostragem mínima para determinar se é um tombamento.
     * 
     */
    double MinimumSamples;
};

/**
 * @brief Configurações para detecção de movimento.
 * 
 */
struct IMUMovementSettings_t
{
public:
    /**
     * @brief Aceleração mínima no eixo longitudinal.
     * 
     */
    double StartThreshold;

    /**
     * @brief Quantidade mínima de amostras para confirmar que está em movimento.
     * 
     */
    double MinimumSamples;
};

/**
 * @brief Configurações para detecção de parada.
 * 
 */
struct IMUStopSettings_t
{
public:
    /**
     * @brief Aceleração negativa mínima no eixo longitudinal.
     * 
     */
    double StartThreshold;

    /**
     * @brief Quantidade mínima de amostras para confirmar que está parando.
     * 
     */
    double MinimumSamples;
};