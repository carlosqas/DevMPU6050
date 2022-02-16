/**
 * @file IMUSensor.h
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Superclasse dos sensores IMU.
 * @version 0.1
 * @date 17-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <Arduino.h>
#include <algorithm>
#include <list>
#include <math.h>
#include <vector>
#include <Wire.h>

#include "CircularBuffer.h"
#include "I2Cdev.h"
#include "IMUSensorStructs.h"

const int g_historySize = 100; // Tamanho do histórico de leituras do sensor.

/**
 * @brief Superclasse de sensores IMU
 * 
 */
class IMUSensor
{
protected:
    /**
     * @brief Inicializar o IMU.
     * 
     * @param wire Interface I2C.
     * @return true - Se o sensor iniciar corretamente.
     * @return false - Caso contrário.
     */
    bool begin(TwoWire &wire);

    /**
     * @brief Adiciona novas informações no buffer histórico
     * de medidas do sensor.
     * @param measurement Medida a ser adicionada no buffer.
     */
    void addMeasurement(IMUAxisData_t measurement);

    /**
     * @brief Realiza a reinicialização do vetor de medidas
     * do sensor.
     */
    void resetMeasurements();

    /**
     * @brief Define o estado atual do equipamento de acordo
     * com as flags de estado.
     */
    void updateState();

    /**
     * @brief Realiza a leitura dos dados provenientes do sensor.
     * 
     */
    virtual void acquireData() = 0;

    /**
     * @brief Realiza a detecção de tombamento.
     * 
     */
    void detectTipping();

    /**
     * @brief Realiza a detecção de movimento.
     * 
     */
    void detectMovement();

    /**
     * @brief Realiza a detecção de parada.
     * 
     */
    void detectStop();

    /**
     * @brief Realiza a detecção de tamper.
     * 
     */
    void detectTamper();

    /**
     * @brief Verifica se o dispositivo foi configurado.
     * 
     * @return true - Caso tenha sido configurado.
     * @return false - Caso contrário.
     */
    bool checkConfigurations();

    bool m_tipped;                    // Flag de tombamento.
    bool m_moving;                    // Flag de movimento.
    bool m_tamper;                    // Flag de tamper.
    DeviceState_e m_devState;         // Estado atual do automóvel.

public:
    /**
     * @brief Realiza a calibração do sensor.
     * 
     * @return IMUOffsets_t - Offsets provenientes da calibração.
     */
    virtual IMUOffsets_t calibrate() = 0;

    /**
     * @brief Configurar parâmetros de detecção de tombamento.
     * 
     * @param settings Parâmetros de detecção de tombamento.
     */
    void configureTipping(IMUTippingSettings_t settings);

    /**
     * @brief Configurar o detector de movimento.
     * 
     * @param settings Configurações do detector de movimento.
     */
    void configureMovementDetection(IMUMovementSettings_t settings);

    /**
     * @brief Configurar o detector de parada.
     * 
     * @param settings Configurações do detector de parada.
     */
    void configureStopDetection(IMUStopSettings_t settings);

    /**
     * @brief Configurar o detector de tamper.
     * 
     * @param settings Configurações do detector de tamper.
     */
    void configureTamperDetection(IMUTamperSettings_t settings);
    
    /**
     * @brief Retornar a última leitura dos eixos do acelerômetro
     * e do giroscópio.
     * @return IMUAxisData_t - Eixos do acelerômetro e do giroscópio.
     */
    IMUAxisData_t getAxisData();    
    
    /**
     * @brief Itera as medições do sensor.
     * 
     */
    void handle();

     /**
     * @brief Retorna o estado de tombamento do equipamento.
     * 
     * @return true - Caso esteja tombado.
     * @return false - Caso contrário.
     */
    bool getTippedState();

    /**
     * @brief Retorna o estado de movimento do equipamento.
     * 
     * @return true - Caso esteja em movimento.
     * @return false - Caso contrário.
     */
    bool getMovingState();

    /**
     * @brief Retorna o estado de tamper do equipamento.
     * 
     * @return true - Caso esteja em tamper.
     * @return false - Caso contrário.
     */
    bool getTamperState();


    /**
     * @brief Retorna os dados históricos de tombamento por referência.
     * 
     * @param tipingStruct Struct que armazenará os dados de tombamento.
     */
    void getTippedData(IMUTippingData_t &tipingStruct);
    
    /**
     * @brief Retorna os dados históricos da detecção de tombamento.
     * 
     * @param movementStruct Struct que armazenará os dados de movimento.
     */
    void getMovementData(IMUMovementData_t &movementStruct);

    /**
     * @brief Retorna os dados históricos de detecção de parada.
     * 
     * @param stopStruct Struct que armazenará os dados de parada.
     */
    void getStopData(IMUStopData_t &stopStruct);

    /**
     * @brief Retorna os dados históricos de detecção de tamper.
     * 
     * @param tamperStruct Struct que armazenará os dados de tamper.
     */
    void getTamperData(IMUTamperData_t &tamperStruct);

    /**
     * @brief Retorna o status atual do automóvel.
     * 
     * @return DeviceState_e - Estado do automóvel.
     */
    DeviceState_e getDevState();

    /**
     * @brief Template para o retorno dos Offsets ativos no sensor.
     * 
     * @return IMUOffsets_t - Struct contendo os offsets ativos do sensor.
     */
    virtual IMUOffsets_t getCurrentOffsets() = 0;

    /**
     * @brief Altera os offsets do dispositivo.
     * 
     * @param newOffsets Novos offsets que serão utilizados
     * pelo dispositivo
     */
    virtual void setOffsets(IMUOffsets_t newOffsets) = 0;

private: 
    IMUTippingSettings_t m_tippingSettings;   // Configurações para a detecção de tombamento.
    IMUMovementSettings_t m_movementSettings; // Configurações para a detecção de movimento.
    IMUStopSettings_t m_stopSettings;         // Configurações para a detecção de parada.
    IMUTamperSettings_t m_tamperSettings;     // Configurações para a detecção de tamper.
    IMUTippingData_t m_tippingData;     // Dados de tombamento.
    IMUMovementData_t m_movementData;   // Dados de movimento.
    IMUStopData_t m_stopData;           // Dados de parada.
    IMUTamperData_t m_tamperData;       // Dados de tamper.
    CircularBuffer <IMUAxisData_t, g_historySize> m_axisData; // Buffer circular com dados históricos das medidas do sensor.
};