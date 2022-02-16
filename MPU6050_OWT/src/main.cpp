/**
 * @file main.cpp
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Firmware de teste das funcionalidades da MPU6050.
 * @version 0.1
 * @date 17-11-2021
 * 
 * @copyright Copyright (c) 2021
 */

#include "IMUSensorLib.h"
#include "Watcher.h"

IMUSensor *imu;

void setup() 
{
    Serial.begin(115200);
    
    WatcherClass* watcher = new WatcherClass();

    // Configurando sensibilidade das detecções.
    IMUTippingSettings_t tippingSettings;
    tippingSettings.MinimumSamples = 25;
    tippingSettings.TippingStartThreshold = 140;

    IMUMovementSettings_t movementSettings;
    movementSettings.MinimumSamples = 10;
    movementSettings.StartThreshold = 0.7;

    IMUStopSettings_t stopSettings;
    stopSettings.MinimumSamples = 10;
    stopSettings.StartThreshold = 0.7;

    // Instanciando o sensor e configurando-o.
    imu = IMUFactory::create(IMUModel_e::IMU_MODEL_MPU6050, Wire);
    imu->attach(watcher);
    imu->configureTipping(tippingSettings);
    imu->configureMovementDetection(movementSettings);
    imu->configureStopDetection(stopSettings);
    imu->start(250);
}

void loop() 
{ 
    if(imu->getTippedState())
        imu->stop();
}