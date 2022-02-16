/**
 * @file Watcher.h
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Classe criada apenas para observação de eventos do IMUSensor.
 * @version 0.1
 * @date 22-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include "IMUSensorLib.h"

/**
 * @brief Classe que carecteriza um observador genérico.
 * 
 */
class WatcherClass : public IIMUObserver
{
public:
    /**
     * @brief Método construtor do WatcherClass.
     * 
     */
    WatcherClass(){}
    /**
     * @brief Evento ocorrido quando o objeto tombar.
     *  
     */
    void OnTipping(IMUTippingData_t data);

    /**
     * @brief Evento ocorrido quando o objeto se movimentar.
     * 
     */
    void OnMovement(IMUMovementData_t data);
    
    /**
     * @brief Evento ocorrido quando o objeto parar.
     * 
     */
    void OnStop(IMUStopData_t data);
};