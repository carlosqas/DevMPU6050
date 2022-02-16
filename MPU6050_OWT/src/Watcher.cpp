/**
 * @file Watcher.cpp
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Implementação das funções da classe Watcher.
 * @version 0.1
 * @date 22-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Watcher.h"

void WatcherClass::OnTipping(IMUTippingData_t data)
{
    Serial.printf("\n\n--- Tipping Data ---");
    Serial.printf("\nTime: %d", data.StartTime);
    Serial.printf("\nSide: %s", (data.Side == 0) ? "left" : "right");
    
    // for(auto axisData : data.AxisMeasurements)
    // {
    //     Serial.printf("\n\n--- Axis data ---");
    //     Serial.printf("\nAccX: %.2f", axisData.Acc_X);
    //     Serial.printf("\nAccY: %.2f", axisData.Acc_Y);
    //     Serial.printf("\nAccZ: %.2f", axisData.Acc_Z);
    //     Serial.printf("\nGyroX: %.2f", axisData.Gyro_X);
    //     Serial.printf("\nGyroY: %.2f", axisData.Gyro_Y);
    //     Serial.printf("\nGyroZ: %.2f", axisData.Gyro_Z);
    //     Serial.printf("\nYaw: %.2f", axisData.Yaw);
    //     Serial.printf("\nPitch: %.2f", axisData.Pitch);
    //     Serial.printf("\nRoll: %.2f", axisData.Roll);
    //     Serial.printf("\nTime: %d", axisData.Time);
    // }
}

void WatcherClass::OnMovement(IMUMovementData_t data)
{
    Serial.printf("\n\n--- Movement Data ---");
    Serial.printf("\nTime: %d", data.StartTime);
}

void WatcherClass::OnStop(IMUStopData_t data)
{
    Serial.printf("\n\n--- Stop Data ---");
    Serial.printf("\nTime: %d", data.StartTime);
}