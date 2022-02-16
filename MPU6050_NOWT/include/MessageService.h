/**
 * @file MessageService.h
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief 
 * @version 0.1
 * @date 30-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include "IMUSensorLib.h"
#include "DebugService.h"

/**
 * @brief Classe que contém os atributos e 
 * métodos do serviço de mensageria.
 */
class MessageServiceClass
{
public:
    /**
     * @brief Inicia o serviço de mensageria.
     * 
     */
    void begin();

    /**
     * @brief Itera o serviço de mensageria.
     * 
     */
    void handle();

    /**
     * @brief Atrela o dispositivo que sofrerá modificações de 
     * configuração.
     * @param device 
     */
    void setDevice(IMUSensor *device);

private:
    IMUSensor *m_device; // Dispositivo que irá alterar.
};

extern MessageServiceClass MessageService;