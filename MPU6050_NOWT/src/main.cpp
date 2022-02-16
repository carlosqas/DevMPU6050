/**
 * @file main.cpp
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Firmware de teste das funcionalidades da MPU6050.
 * @version 0.1
 * @date 17-11-2021
 * 
 * @copyright Copyright (c) 2021
 */
#include <vector>

#include "DebugService.h"
#include "MessageService.h"

unsigned long g_readDelay = 200;
bool g_calibrate = false;

void setup() 
{
    Debug.begin(&Serial);

    while(1)
    {
        Serial.printf("\nWould you like to calibrate? (0 or 1)");
        
        if(Serial.available())
        {
            uint8_t currentByte = Serial.read();
            if(currentByte == 0x00 || currentByte == 0x01)
            {
                g_calibrate = (currentByte == 1) ? true : false;
                break;
            }
        }

        delay(750);
    }

    // Configurando a sensibilidade da detecção dos eventos.
    IMUTippingSettings_t tippingSettings;
    tippingSettings.MinimumSamples = 17;
    tippingSettings.TippingStartThreshold = 140;

    IMUMovementSettings_t movementSettings;
    movementSettings.MinimumSamples = 4;
    movementSettings.MovementInterval = 0.07;

    IMUStopSettings_t stopSettings;
    stopSettings.MinimumSamples = 17;
    stopSettings.StopInterval = 0.06;

    IMUTamperSettings_t tamperSettings;
    tamperSettings.TamperTime = 7;
    tamperSettings.MinimumSamples = 5;

    // Instanciando o sensor e configurando-o.
    MPU.begin(Wire);

    if(g_calibrate)
    {
        IMUOffsets_t newOffsets = MPU.calibrate();
        Serial.printf("\n\nAccOffsets[x, y, z]: [%d, %d, %d]", newOffsets.XAccelOffset, 
                                                               newOffsets.YAccelOffset, 
                                                               newOffsets.ZAccelOffset);

        Serial.printf("\nGyroOffsets[x, y, z]: [%d, %d, %d]\n", newOffsets.XGyroOffset, 
                                                                newOffsets.YGyroOffset, 
                                                                newOffsets.ZGyroOffset);
    } 
    else
    {
        IMUOffsets_t currentOffsets = MPU.getCurrentOffsets();
        Serial.printf("\n\nAccOffsets[x, y, z]: [%d, %d, %d]", currentOffsets.XAccelOffset, 
                                                               currentOffsets.YAccelOffset, 
                                                               currentOffsets.ZAccelOffset);

        Serial.printf("\nGyroOffsets[x, y, z]: [%d, %d, %d]\n", currentOffsets.XGyroOffset, 
                                                                currentOffsets.YGyroOffset, 
                                                                currentOffsets.ZGyroOffset);
    }
    
    MPU.configureTipping(tippingSettings);
    MPU.configureMovementDetection(movementSettings);
    MPU.configureStopDetection(stopSettings);
    MPU.configureTamperDetection(tamperSettings);
    MPU.start(g_readDelay);

    Debug.setDevice(&MPU);
    MessageService.setDevice(&MPU);
}

void loop() 
{ 
    MessageService.handle();
    Debug.handle();
    delay(g_readDelay);
}