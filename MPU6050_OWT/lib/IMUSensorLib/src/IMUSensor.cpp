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
unsigned long g_firstTip = 0;       // Millis() em que ocorreu a primeira leitura de tombamento.
unsigned long g_firstMovement = 0;  // Millis() em que ocorreu a primeira leitura de movimento.
unsigned long g_firstStop = 0;      // Millis() em que ocorreu a primeira leitura de parada.

bool IMUSensor::begin(TwoWire &wire)
{
    m_imuSemaphore = xSemaphoreCreateMutex();
    m_semaphoreInitialized = m_imuSemaphore != NULL;
    
    return m_semaphoreInitialized;
}

void IMUSensor::configureTipping(IMUTippingSettings_t settings)
{
    m_tippingSettings.MinimumSamples = settings.MinimumSamples;
    m_tippingSettings.TippingStartThreshold = settings.TippingStartThreshold;
}

void IMUSensor::configureMovementDetection(IMUMovementSettings_t settings)
{
    m_movementSettings.MinimumSamples = settings.MinimumSamples;
    m_movementSettings.StartThreshold = settings.StartThreshold;
}

void IMUSensor::configureStopDetection(IMUStopSettings_t settings)
{
    m_stopSettings.MinimumSamples = settings.MinimumSamples;
    m_stopSettings.StartThreshold = settings.StartThreshold;
}

IMUAxisData_t IMUSensor::getAxisData()
{
    IMUAxisData_t lastData;

    if(!m_axisData.isEmpty() && m_semaphoreInitialized)
    {
        xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
        lastData = m_axisData.last();
        xSemaphoreGive(m_imuSemaphore);
    }

    return lastData;
}

bool IMUSensor::isRunning()
{
    if(!m_semaphoreInitialized)
        return false;

    bool threadRunning = false;

    xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
    threadRunning = m_threadRunning;
    xSemaphoreGive(m_imuSemaphore);
    
    return threadRunning;
}

void IMUSensor::attach(IIMUObserver *observer)
{
    m_subscribers.push_back(observer);
}

void IMUSensor::detach(IIMUObserver *observer)
{
    m_subscribers.erase(std::remove(m_subscribers.begin(), m_subscribers.end(), observer), m_subscribers.end());
}

void IMUSensor::notifyTipping(IMUTippingData_t data)
{  
    if(m_subscribers.empty())
        Serial.printf("\n[IMUSensor] Sem observadores.");
    else
    {
        for(auto observers : m_subscribers)
            observers->OnTipping(data);
    }
}

void IMUSensor::notifyMovement(IMUMovementData_t data)
{
    if(m_subscribers.empty())
        Serial.printf("\n[IMUSensor] Sem observadores.");
    else
    {
        for(auto observer : m_subscribers)
            observer->OnMovement(data);
    }
}

void IMUSensor::notifyStop(IMUStopData_t data)
{
    if(m_subscribers.empty())
        Serial.printf("\n[IMUSensor] Sem observadores.");
    else
    {
        for(auto observer : m_subscribers)
            observer->OnStop(data);
    }
}

bool IMUSensor::getTippedState()
{
    if(!m_semaphoreInitialized)
        return false;

    bool tipped = false;

    xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
    tipped = m_tipped;
    xSemaphoreGive(m_imuSemaphore);

    return tipped;
}

bool IMUSensor::getMovingState()
{
    if(!m_semaphoreInitialized)
        return false;

    bool moving = false;

    xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
    moving = m_moving;
    xSemaphoreGive(m_imuSemaphore);

    return moving;
}

void IMUSensor::detectTipping()
{
    if(m_axisData.isEmpty() || !m_semaphoreInitialized)
        return;

    IMUAxisData_t lastData = getAxisData();

    /**
     * Devido ao novo comportamento do sensor, o Roll deve ser analisado
     * para definir se há um tombamento.
     */
    if(abs(lastData.Roll) > 90)
    {
        if(abs(lastData.Pitch) < m_tippingSettings.TippingStartThreshold)
        {
            if(g_tippedCount == 0)
                g_firstTip = lastData.Time;
            g_tippedCount++;
            Serial.printf("\n[IMUSensor] Tombamento detectado.");
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
            Serial.printf("\n[IMUSensor] Tombamento detectado.");
        }
        else
            g_tippedCount = 0;
    }

    if(g_tippedCount >= m_tippingSettings.MinimumSamples && m_axisData.isFull())
    {
        IMUTippingData_t tippedData;

        xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
        m_tipped = true;
        xSemaphoreGive(m_imuSemaphore);

        tippedData.Side = (lastData.Acc_X > 0) ? IMUTippingSide_e::IMU_TIP_SIDE_LEFT : IMUTippingSide_e::IMU_TIP_SIDE_RIGHT;
        tippedData.StartTime = g_firstTip;

        for(int i = 0; i < g_historySize; i++)
            tippedData.AxisMeasurements.push_back(m_axisData[i]);
        
        notifyTipping(tippedData);
    }
    else
    {
        xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
        m_tipped = false;
        xSemaphoreGive(m_imuSemaphore);
    }
}

void IMUSensor::detectMovement()
{
    if(m_axisData.isEmpty() || !m_semaphoreInitialized)
        return;

    IMUAxisData_t lastData = getAxisData();

    if(lastData.Acc_Z > m_movementSettings.StartThreshold)
    {
        if(g_movementCount == 0)
            g_firstMovement = lastData.Time;
        Serial.printf("\n[IMUSensor] Movimento detectado");
        g_movementCount++;
    }
    else
        g_movementCount = 0;

    if(g_movementCount >= m_movementSettings.MinimumSamples)
    {
        IMUMovementData_t movementData;

        xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
        m_moving = true;
        xSemaphoreGive(m_imuSemaphore);

        movementData.StartTime = g_firstMovement;
        notifyMovement(movementData);
    }
    else
    {
        xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
        m_moving = false;
        xSemaphoreGive(m_imuSemaphore);
    }
}

void IMUSensor::detectStop()
{
    if(m_axisData.isEmpty() || !m_semaphoreInitialized)
        return;

    IMUAxisData_t lastData = getAxisData();

    if(lastData.Acc_Z < (m_movementSettings.StartThreshold * -1))
    {
        if(g_stopCount == 0)
            g_firstStop = lastData.Time;
        Serial.printf("\n[IMUSensor] Parada detectada.");
        g_stopCount++;
    }
    else
        g_stopCount = 0;

    if(g_stopCount >= m_stopSettings.MinimumSamples)
    {
        IMUStopData_t stopData;

        xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
        m_moving = false;
        xSemaphoreGive(m_imuSemaphore);

        stopData.StartTime = g_firstStop;
        notifyStop(stopData);
    }
}

void IMUSensor::addMeasurement(IMUAxisData_t measurement)
{
    if(!m_semaphoreInitialized)
        return;

    xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
    m_axisData.push(measurement);
    xSemaphoreGive(m_imuSemaphore);
}

void IMUSensor::resetMeasurements()
{
    m_axisData.clear();
}