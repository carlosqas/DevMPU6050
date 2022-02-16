/**
 * @file DebugService.cpp
 * @author Carlos Eduardo Marques Assunção Torres (carlos.eduardo.qas@outlook.com)
 * @brief 
 * @version 0.1
 * @date 30-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "DebugService.h"

unsigned long g_lastMemUse = 0; // Millis() em que foi mostrado o uso de memória pela última vez.

void DebugClass::begin(HardwareSerial *newSerial)
{
    m_serial = newSerial;
    m_serial->begin(115200);

    m_showYPR = true;
    m_showAcc = true;
    m_showGyro = false;
    m_showDevState = true;
    m_showMemUsage = false;
    m_showTemperature = false;
}

void DebugClass::handle()
{
    m_devState = m_device->getDevState();

    switch (m_devState)
    {
    case DeviceState_e::STATE_STOPPED:
    {
        IMUStopData_t newData;
        m_device->getStopData(newData);
        break;
    }
    case DeviceState_e::STATE_MOVING:
    {
        IMUMovementData_t newData;
        m_device->getMovementData(newData);
        break;
    }
    case DeviceState_e::STATE_TIPPED:
    {
        IMUTippingData_t newData;
        m_device->getTippedData(newData);
        break;
    }
    case DeviceState_e::STATE_TAMPER:
    {
        IMUTamperData_t newData;
        m_device->getTamperData(newData);
        break;
    }
    default:
        break;
    }

    if(m_showAcc || m_showDevState || m_showGyro || m_showYPR)
        m_serial->printf("\n");
        
    YPR();
    Acc();
    Gyro();
    DeviceState();
    memUsage();
    Temperature();
}

void DebugClass::setDevice(IMUSensor *device)
{
    m_device = device;
}

void DebugClass::setShowYPR(bool newValue)
{
    m_showYPR = newValue;
}

void DebugClass::setShowAcc(bool newValue)
{
    m_showAcc = newValue;
}

void DebugClass::setShowGyro(bool newValue)
{
    m_showGyro = newValue;
}

void DebugClass::setShowDevState(bool newValue)
{
    m_showDevState = newValue;
}

void DebugClass::setShowMemUsage(bool newValue)
{
    m_showMemUsage = newValue;
}

void DebugClass::setShowTemperature(bool newValue)
{
    m_showTemperature = newValue;
}

void DebugClass::YPR()
{
    if(m_showYPR)
    {
        IMUAxisData_t newData = m_device->getAxisData();
        m_serial->printf(" | YPR: %.2f, %.2f, %.2f", newData.Yaw, newData.Pitch, newData.Roll);
    }
}

void DebugClass::Acc()
{
    if(m_showAcc)
    {
        IMUAxisData_t newData = m_device->getAxisData();
        double geralAccel = sqrt(pow(newData.Acc_X, 2) + pow(newData.Acc_Y, 2) + pow(newData.Acc_Z, 2));
        m_serial->printf(" | Acc: %.2f, %.2f, %.2f, geral: %.4f", newData.Acc_X, newData.Acc_Y, newData.Acc_Z, geralAccel);
    }
}

void DebugClass::Gyro()
{
    if(m_showGyro)
    {
        IMUAxisData_t newData = m_device->getAxisData();
        m_serial->printf(" | Gyro: %.2f, %.2f, %.2f", newData.Gyro_X, newData.Gyro_Y, newData.Gyro_Z);
    }
}

void DebugClass::DeviceState()
{
    if(m_showDevState)
    {
        switch (m_devState)
        {
        case DeviceState_e::STATE_STOPPED:
            m_serial->printf(" | parado |");
            break;
        case DeviceState_e::STATE_MOVING:
            m_serial->printf(" | em movimento |");
            break;
        case DeviceState_e::STATE_TIPPED:
            m_serial->printf(" | tombado |");
            break;
        case DeviceState_e::STATE_TAMPER:
            m_serial->printf(" | tamper |");
            break;
        default:
            m_serial->printf(" | estado do dispositivo nao reconhecido |");
            break;
        }
    }
}

void DebugClass::memUsage()
{
    if(m_showMemUsage)
    {
        if((millis() - g_lastMemUse) > 10000)
        {
            m_serial->printf("\nFree heap: %d | PSRam: %d", ESP.getFreeHeap(), ESP.getFreePsram());
            g_lastMemUse = millis();
        }
    }
}

void DebugClass::Temperature()
{
    if(m_showTemperature)
    {
        IMUAxisData_t newData = m_device->getAxisData();
        m_serial->printf("\nTemperature: %.2f", newData.Temperature);
    }
}

DebugClass Debug;