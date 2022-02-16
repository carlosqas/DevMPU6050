/**
 * @file Debug.h
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Arquivo de implementação da classe Debug.
 * @version 0.1
 * @date 30-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <Arduino.h>
#include <math.h>

#include "IMUSensorLib.h"

/**
 * @brief Classe que realizará debug e teste
 * das funções e estados do device.
 */
class DebugClass
{
public:
    /**
     * @brief Inicia os valores padrão dos atributos da classe
     * e inicia a interface serial selecionada para debug.
     * @param newSerial Interface serial utilizada para debug.
     */
    void begin(HardwareSerial *newSerial);

    /**
     * @brief Itera o serviço de Debug.
     * 
     */
    void handle();

    /**
     * @brief Atrela ao serviço de debug um dispositivo
     * do tipo IMUSensor.
     * @param device Dispostivo a ser atrelado.
     */
    void setDevice(IMUSensor *device);

    /**
     * @brief Muda o valor da flag que mostra ou não as 
     * informações de Yaw, Pitch e Roll.
     * @param newValue Novo valor.
     */
    void setShowYPR(bool newValue);

    /**
     * @brief Muda o valor da flag que mostra ou não as
     * informações do acelerômetro.
     * @param newValue Novo valor.
     */
    void setShowAcc(bool newValue);

    /**
     * @brief Muda o valor da flag que mostra ou não as 
     * informações de giroscópio.
     * @param newValue Novo valor.
     */
    void setShowGyro(bool newValue);

    /**
     * @brief Muda o valor da flag que mostra ou não as
     * informações do estado atual do dispositivo.
     * @param newValue Novo valor.
     */
    void setShowDevState(bool newValue);
    
    /**
     * @brief Muda o valor da flag que mostra ou não as
     * informações de uso de memória.
     * @param newValue Novo valor.
     */
    void setShowMemUsage(bool newValue);

    /**
     * @brief Muda o valor da flag que mostra ou não as
     * informações de temperatura.
     * @param newValue Novo valor.
     */
    void setShowTemperature(bool newValue);

private:
    /**
     * @brief Função que realiza o print de Yaw, Pitch
     * e Roll.
     */
    void YPR();

    /**
     * @brief Função que realiza o print das informações
     * do acelerômetro.
     */
    void Acc();

    /**
     * @brief Função que realiza o print das informações
     * do giroscópio.
     */
    void Gyro();

    /**
     * @brief Função que realiza o print das informações
     * do estado atual do dispositivo.
     */
    void DeviceState();

    /**
     * @brief Função que realiza o print das informações
     * de uso de memória.
     */
    void MemoryUsage();

    /**
     * @brief Função que realiza o print das informações
     * de temperatura.
     */
    void Temperature();

    IMUSensor *m_device;      // Dispositivo que será observado para obter as informações.
    HardwareSerial *m_serial; // Serial que será utilizada para debug.
    DeviceState_e m_devState; // Status atual do dispositivo.
    bool m_showYPR;           // Flag que indica se informações de YPR são mostradas.
    bool m_showAcc;           // Flag que indica se informações de Acc são mostradas.
    bool m_showGyro;          // Flag que indica se informações de Gyro são mostradas.
    bool m_showDevState;      // Flag que indica se informações de estado são mostradas.
    bool m_showMemUsage;      // Flag que indica se o uso de memória é mostrado.
    bool m_showTemperature;   // Flag que indica se informações de temperature são mostradas.
};

extern DebugClass Debug;