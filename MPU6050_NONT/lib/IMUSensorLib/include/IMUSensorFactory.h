/**
 * @file IMUSensorFactory.h
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Header para a factory de objetos de diferentes sensores.
 * @version 0.1
 * @date 18-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include "MPU6050_IMU.h"

/**
 * @brief Permite a criação do objeto IMUSensor de acordo com o modelo escolhido.
 * 
 */
class IMUFactory
{
public:
    /**
     * @brief Cria o IMUSensor de acordo com o IMU escolhido.
     * 
     * @param model Modelo do IMU.
     * @param wire Ponteiro para a interface I2C.
     * @return IMUSensor* - Ponteiro para o IMU criado.
     */
    static IMUSensor* create(IMUModel_e model, TwoWire &wire)
    {
        IMUSensor *sensor;

        switch (model)
        {
        case IMUModel_e::IMU_MODEL_MPU6050:
            sensor = new MPU6050IMU();
            ((MPU6050IMU *)sensor)->begin(wire);
            break;
        default:
            break;
        }
        
        return sensor;
    }
};