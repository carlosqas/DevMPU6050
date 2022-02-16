/**
 * @file IMUSensorEnums.h
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Arquivo destinado ao armazenamentos dos Enums relacionados ao IMUSensor
 * @version 0.1
 * @date 18-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

/**
 * @brief Lado do tombamento.
 * 
 */
enum IMUTippingSide_e
{
    IMU_TIP_SIDE_LEFT = 0,
    IMU_TIP_SIDE_RIGHT
};

/**
 * @brief Modelo de IMU.
 * 
 */
enum IMUModel_e
{
    // MPU6050
    IMU_MODEL_MPU6050 = 0
};

/**
 * @brief Estado atual do dispositivo.
 * 
 */
enum DeviceState_e
{
    // Parado.
    STATE_STOPPED = 0,
    // Em movimento.
    STATE_MOVING,
    // Tombado.
    STATE_TIPPED,
    // Tamper
    STATE_TAMPER
};