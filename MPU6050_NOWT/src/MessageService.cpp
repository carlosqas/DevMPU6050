/**
 * @file MessageService.cpp
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief 
 * @version 0.1
 * @date 30-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "MessageService.h"

void MessageServiceClass::begin()
{

}

void MessageServiceClass::handle()
{
    if(Serial.available())
    {
        std::vector<uint8_t> buffer;
        while(Serial.available())
        {
            buffer.push_back(Serial.read());
            delay(7);
        }

        if(buffer[0] == 0x90)
        {
            // Configuração de Sensibilidade
            switch (buffer[1])
            {
            // Configurando sensibilidade do Tipping.
            case 0x01:
            {
                IMUTippingSettings_t newSettings;
                newSettings.MinimumSamples = buffer[2];
                newSettings.TippingStartThreshold = buffer[3];
                Serial.printf("\nTipping Config >> MinSamples: %d | StartThreshold: %.2f", newSettings.MinimumSamples, newSettings.TippingStartThreshold);
                m_device->configureTipping(newSettings);
                break;
            }
            // Configurando sensibilidade do Movement.
            case 0x02:
            {
                IMUMovementSettings_t newSettings;
                newSettings.MinimumSamples = buffer[2];
                newSettings.MovementInterval = ((double)buffer[3])/100;
                Serial.printf("\nMovement Config >> MinSamples: %d | MovementInterval: Acc > %.2f || Acc < %.2f", newSettings.MinimumSamples, 1 + newSettings.MovementInterval, 1 - newSettings.MovementInterval);
                m_device->configureMovementDetection(newSettings);
                break;
            }
            // Configurando sensibilidade do Stop.
            case 0x03:
            {
                IMUStopSettings_t newSettings;
                newSettings.MinimumSamples = buffer[2];
                newSettings.StopInterval = ((double)buffer[3])/100;
                Serial.printf("\nStop Config >> MinSamples: %d | StopInterval: Acc < %.2f && Acc > %.2f", newSettings.MinimumSamples, 1 + newSettings.StopInterval, 1 - newSettings.StopInterval);
                m_device->configureStopDetection(newSettings);
                break;
            }
            // Configurando sensibilidade do Tamper.
            case 0x04:
            {
                IMUTamperSettings_t newSettings;
                newSettings.MinimumSamples = buffer[2];
                newSettings.TamperTime = buffer[3];
                Serial.printf("\nTamper Config >> MinSamples: %d | TamperTime: %d s", newSettings.MinimumSamples, newSettings.TamperTime);
                m_device->configureTamperDetection(newSettings);
                break;
            }
            case 0x05:
                ESP.restart();
                break;
            default:
                break;
            }
        }
        else if(buffer[0] == 0x80)
        {
            // Configuração de Debug.
            switch (buffer[1])
            {
            case 0x01:
                Debug.setShowYPR(buffer[2] == 0x01);
                break;
            case 0x02:
                Debug.setShowAcc(buffer[2] == 0x01);
                break;
            case 0x03:
                Debug.setShowGyro(buffer[2] == 0x01);
                break;
            case 0x04:
                Debug.setShowDevState(buffer[2] == 0x01);
                break;
            case 0x05:
                Debug.setShowMemUsage(buffer[2] == 0x01);
                break;
            case 0x06:
                Debug.setShowTemperature(buffer[2] == 0x01);
                break;
            default:
                break;
            }
        }
    }
}

void MessageServiceClass::setDevice(IMUSensor *device)
{
    m_device = device;
}

MessageServiceClass MessageService;