/**
 * @file IIMUObserver.h
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Superclasse dos observadores de eventos provindos de sensores IMU.
 * @version 0.1
 * @date 18-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include "IMUSensorStructs.h"

/**
 * @brief Interface para o observador da IMU.
 * 
 */
class IIMUObserver
{
public:
    /**
     * @brief Função virtual a ser implementada pelo observador para reagir ao evento de tombamento.
     * 
     */
    virtual void OnTipping(IMUTippingData_t data) = 0;

    /**
     * @brief Função virtual a ser implementada pelo observador para reagir ao evento de movimento.
     * 
     */
    virtual void OnMovement(IMUMovementData_t data) = 0;
    
    /**
     * @brief Função virtual a ser implementada pelo observador para reagir ao evento de parada.
     * 
     */
    virtual void OnStop(IMUStopData_t data) = 0;
};