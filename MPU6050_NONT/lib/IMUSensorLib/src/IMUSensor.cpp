/**
 * @file IMUSensor.cpp
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Arquivo de implementação das fundões da classe IMUSensor.
 * @version 0.1
 * @date 18-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "IMUSensor.h"

uint16_t g_tippedCount = 0;   // Contador de quantas amostras foram analisadas para determinar tombamento.
uint16_t g_movementCount = 0; // Contador de quantas amostras foram analisadas para determinar movimento.
uint16_t g_stopCount = 0;     // Contador de quantas amostras foram analisadas para determinar parada.
uint16_t g_tamperCount = 0;   // Contador de quantas amostras foram analisadas para determinar tamper.
unsigned long g_firstTip = 0;       // Millis() em que ocorreu a primeira leitura de tombamento.
unsigned long g_firstMovement = 0;  // Millis() em que ocorreu a primeira leitura de movimento.
unsigned long g_firstStop = 0;      // Millis() em que ocorreu a primeira leitura de parada.
unsigned long g_firstTamper = 0;    // Millis() em que ocorreu a primeira leitura de tamper.
unsigned long g_firstMovingTip = 0; // Millis() em que é identificado um tombamento com movimento.

bool IMUSensor::begin(TwoWire &wire)
{
    return true;
}

void IMUSensor::configureTipping(IMUTippingSettings_t settings)
{
    m_tippingSettings.MinimumSamples = settings.MinimumSamples;
    m_tippingSettings.TippingStartThreshold = settings.TippingStartThreshold;
}

void IMUSensor::configureMovementDetection(IMUMovementSettings_t settings)
{
    m_movementSettings.MinimumSamples = settings.MinimumSamples;
    m_movementSettings.MovementInterval = settings.MovementInterval;
}

void IMUSensor::configureStopDetection(IMUStopSettings_t settings)
{
    m_stopSettings.MinimumSamples = settings.MinimumSamples;
    m_stopSettings.StopInterval = settings.StopInterval;
}

void IMUSensor::configureTamperDetection(IMUTamperSettings_t settings)
{
    m_tamperSettings.MinimumSamples = settings.MinimumSamples;
    m_tamperSettings.TamperTime = settings.TamperTime;
}

IMUAxisData_t IMUSensor::getAxisData()
{
    IMUAxisData_t lastData;

    if(!m_axisData.isEmpty())
        lastData = m_axisData.last();

    return lastData;
}

void IMUSensor::handle()
{
    acquireData();

    detectTipping();
    detectTamper();
    if(m_moving)
        detectStop();
    else
        detectMovement();

    updateState();
}

bool IMUSensor::getTippedState()
{
    return m_tipped;
}

bool IMUSensor::getMovingState()
{
    return m_moving;
}

bool IMUSensor::getTamperState()
{
    return m_tamper;
}

void IMUSensor::detectTipping()
{
    if(m_axisData.isEmpty())
        return;

    IMUAxisData_t lastData = getAxisData();

    if(abs(lastData.Roll) > 90)
    {
        if(abs(lastData.Pitch) < m_tippingSettings.TippingStartThreshold)
        {
            if(g_tippedCount == 0)
                g_firstTip = lastData.Time;
            g_tippedCount++;
        }
        else
            g_tippedCount = 0;
    }
    else
    {
        if(abs(lastData.Pitch) > (180 - m_tippingSettings.TippingStartThreshold))
        {
            if(g_tippedCount == 0)
                g_firstTip = lastData.Time;
            g_tippedCount++;
        }
        else
            g_tippedCount = 0;
    }

    if(g_tippedCount >= m_tippingSettings.MinimumSamples && m_axisData.isFull())
    {
        m_tippingData.AxisMeasurements.clear();
        m_tipped = true;
        m_tippingData.Side = (lastData.Pitch > 0) ? IMUTippingSide_e::IMU_TIP_SIDE_LEFT : IMUTippingSide_e::IMU_TIP_SIDE_RIGHT;
        m_tippingData.StartTime = g_firstTip;

        for(int i = 0; i < g_historySize; i++)
            m_tippingData.AxisMeasurements.push_back(m_axisData[i]);
    }
    else
        m_tipped = false;
}

void IMUSensor::detectMovement()
{
    if(m_axisData.isEmpty())
        return;

    IMUAxisData_t lastData = getAxisData();

    double moduleAcc = sqrt(pow(lastData.Acc_X, 2) + pow(lastData.Acc_Y, 2) + pow(lastData.Acc_Z, 2));

    if(moduleAcc < (1 - m_movementSettings.MovementInterval) || moduleAcc > (1 + m_movementSettings.MovementInterval))
    {
        if(g_movementCount == 0)
            g_firstMovement = lastData.Time;
        g_movementCount++;
    }

    if(g_movementCount >= m_movementSettings.MinimumSamples)
    {
        g_stopCount = 0;
        m_moving = true;
        m_movementData.StartTime = g_firstMovement;
    }
}

void IMUSensor::detectStop()
{
    if(m_axisData.isEmpty())
        return;

    IMUAxisData_t lastData = getAxisData();

    double moduleAcc = sqrt(pow(lastData.Acc_X, 2) + pow(lastData.Acc_Y, 2) + pow(lastData.Acc_Z, 2));

    if(moduleAcc > (1 - m_stopSettings.StopInterval) && moduleAcc < (1 + m_stopSettings.StopInterval))
    {
        if(g_stopCount == 0)
            g_firstStop = lastData.Time;
        g_stopCount++;
    }
    else
        g_stopCount = 0;

    if(g_stopCount >= m_stopSettings.MinimumSamples)
    {
        g_movementCount = 0;
        m_moving = false;
        m_stopData.StartTime = g_firstStop;
    }
}

void IMUSensor::detectTamper()
{
    if(m_axisData.isEmpty())
        return;
    
    IMUAxisData_t lastData = getAxisData();

    if(abs(lastData.Acc_Z) > abs(lastData.Acc_Y) && abs(lastData.Acc_Z) > abs(lastData.Acc_X))
    {
        if(g_tamperCount == 0)
            g_firstTamper = millis();
        g_tamperCount++;
    }
    else
        g_tamperCount = 0;
    
    if(g_tamperCount >= m_tamperSettings.MinimumSamples)
    {
        m_tamper = true;
        m_tamperData.StartTime = g_firstTamper;
    }
    else 
        m_tamper = false;
}

void IMUSensor::addMeasurement(IMUAxisData_t measurement)
{
    m_axisData.push(measurement);
}

void IMUSensor::resetMeasurements()
{
    m_axisData.clear();
}

void IMUSensor::getTippedData(IMUTippingData_t &tippingStruct)
{
    tippingStruct.StartTime = m_tippingData.StartTime;
    tippingStruct.Side = m_tippingData.Side;
    tippingStruct.AxisMeasurements.assign(m_tippingData.AxisMeasurements.begin(), m_tippingData.AxisMeasurements.end());    
}

void IMUSensor::getMovementData(IMUMovementData_t &movementStruct)
{
    movementStruct.StartTime = m_movementData.StartTime;    
}

void IMUSensor::getStopData(IMUStopData_t &stopStruct)
{
    stopStruct.StartTime = m_stopData.StartTime;    
}

void IMUSensor::getTamperData(IMUTamperData_t &tamperStruct)
{
    tamperStruct.StartTime = m_tamperData.StartTime;
}

DeviceState_e IMUSensor::getDevState()
{
    return m_devState;
}

void IMUSensor::updateState()
{
    if(m_tamper)
        m_devState = DeviceState_e::STATE_TAMPER;
    else if(m_tipped && !m_moving)
        m_devState = DeviceState_e::STATE_TIPPED;
    else if(m_tipped && m_moving)
    {
        if(g_firstMovingTip == 0)
            g_firstMovingTip = millis();
        else if(abs(millis() - g_firstMovingTip) > (m_tamperSettings.TamperTime * 1000))
        {
            m_devState = DeviceState_e::STATE_TAMPER;
            g_firstMovingTip = 0;
        }
    }
    else if(!m_tipped && m_moving)
        m_devState = DeviceState_e::STATE_MOVING;
    else
        m_devState = DeviceState_e::STATE_STOPPED;

    if(!(m_moving && m_tipped))
        g_firstMovingTip = 0;
}

bool IMUSensor::checkConfigurations()
{
    bool TipSetted = m_tippingSettings.MinimumSamples != 0; 
    bool MovementSetted = m_movementSettings.MinimumSamples != 0; 
    bool StopSetted = m_stopSettings.MinimumSamples != 0; 
    bool TamperSetted = m_tamperSettings.MinimumSamples != 0; 
    return (TipSetted || MovementSetted || StopSetted || TamperSetted);
}