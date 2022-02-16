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
#include <vector>
#include <Wire.h>

#include "I2Cdev.h"
#include "CircularBuffer.h"
#include "IIMUObserver.h"
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
     * @brief Realiza a calibração do sensor.
     * 
     * @return true - Caso a calibração seja bem sucedida.
     * @return false - Caso contrário.
     */
    virtual bool calibrate() = 0;

    /**
     * @brief Atualiza o buffer circular de leituras do 
     * sensor.
     */
    virtual void updateData() = 0;

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

    bool m_threadRunning;                  // Flag que indica se a thread de leitura está ativada.
    bool m_tipped;                         // Flag de tombamento.
    bool m_moving;                         // Flag de movimento.
    bool m_semaphoreInitialized;           // Flag que indica o funcionamento do semáforo.
    int m_readFrequency;                   // Frequência da leitura do sensor.
    SemaphoreHandle_t m_imuSemaphore;      // Semaforização de processos sensíveis.

public:
    /**
     * @brief Inicializar o IMU.
     * 
     * @param wire Interface I2C.
     * @return true - Se o sensor iniciar corretamente.
     * @return false - Caso contrário.
     */
    bool begin(TwoWire &wire);
    
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
     * @brief Retornar a última leitura dos eixos do acelerômetro
     * e do giroscópio.
     * @return IMUAxisData_t - Eixos do acelerômetro e do giroscópio.
     */
    IMUAxisData_t getAxisData();    
    
    /**
     * @brief Iniciar a thread que realiza as medições.
     * 
     * @param frequency Frequência que a thread irá executar as medições (a cada x milissegundos).
     */
    virtual void start(int frequency) = 0;

    /**
     * @brief Parar a thread que realiza as medições.
     * 
     */
    virtual void stop() = 0;

    /**
     * @brief Retornar se a thread está rodando ou não.
     * 
     * @return true Caso esteja rodando.
     * @return false Caso contrário.
     */
    bool isRunning();

    /**
     * @brief Registrar um novo observador.
     * 
     * @param observer Observador.
     */
    void attach(IIMUObserver *observer);

    /**
     * @brief Desregistrar um observador.
     * 
     * @param observer Observador.
     */
    void detach(IIMUObserver *observer);

    /**
     * @brief Notificar os observadores sobre um evento de tombamento.
     * 
     * @param data Dados de tombamento.
     */
    void notifyTipping(IMUTippingData_t data);
    
    /**
     * @brief Notificar os observadores sobre um evento de movimentação.
     * 
     * @param data Dados de movimentação.
     */
    void notifyMovement(IMUMovementData_t data);
    
    /**
     * @brief Notificar os observadores sobre um evento de parada.
     * 
     * @param data Dados de parada.
     */
    void notifyStop(IMUStopData_t data);

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

private: 
    IMUTippingSettings_t m_tippingSettings;         // Configurações para a detecção de tombamento.
    IMUMovementSettings_t m_movementSettings;       // Configurações para a detecção de movimento.
    IMUStopSettings_t m_stopSettings;               // Configurações para a detecção de parada.
    std::vector<IIMUObserver *> m_subscribers;       // Vetor com observadores da classe.
    CircularBuffer <IMUAxisData_t, g_historySize> m_axisData; // Buffer circular com dados históricos das medidas do sensor.
};