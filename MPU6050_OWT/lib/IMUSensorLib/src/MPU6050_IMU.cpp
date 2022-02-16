/**
 * @file MPU6050_IMU.cpp
 * @author Carlos Eduardo Marques Assunção Torres (carlos.torres@vido-la.com.br)
 * @brief Arquivo de implementação das funções da classe MPU6050IMU.
 * @version 0.1
 * @date 18-11-2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "MPU6050_IMU.h"

uint16_t g_fifoPacketSize;          // Tamanho esperado do pacote do DMP (Padrão: 42 bytes)
uint16_t g_fifoCount;               // Quantos bytes o FIFO possui atualmente.
uint8_t g_fifoBuffer[64];           // Buffer para armezamento do FIFO.
VectorFloat g_gravity;              // [x, y, z]    Vetor para informações da gravidade.
Quaternion g_quart;                 // [w, x, y, z] Vetor para informações de quaternion.
const float g_degreeRad = 180/M_PI; // Termo de conversão de radianos para graus.
float g_YPR[3];                     // Buffer de leitura do Yaw, Pitch e Roll
unsigned long g_timeLastRead = 0;   // Millis() em que foi feito último registro de leitura no histórico.

TaskHandle_t g_readTaskHandle = NULL; // Handle da task de leitura.

MPU6050IMU::MPU6050IMU()
{
    m_deviceStatus = 0;
    m_dmpStatus = false;
    m_moving = false;
    m_tipped = false;
    m_threadRunning = false;
}

bool MPU6050IMU::begin(TwoWire &wire)
{
    wire.begin(33, 32, 400000);
    m_mpu.initialize();

    if(!m_mpu.testConnection())
    {
        Serial.printf("\n[MPU6050IMU] Conexao com a MPU falhou!");
        return false;
    }

    m_deviceStatus = m_mpu.dmpInitialize();

    // Mudar esses valores por Offsets default ou por Offsets lidos da memória.
    m_mpu.setXAccelOffset(506);
    m_mpu.setYAccelOffset(385);
    m_mpu.setZAccelOffset(1158);
    m_mpu.setXGyroOffset(-45);
    m_mpu.setYGyroOffset(-73);
    m_mpu.setZGyroOffset(-27);

    if(m_deviceStatus != 0)
    {
        Serial.printf("\n[MPU6050IMU] Conexão com o DMP falhou!");
        return false;
    }

    m_mpu.setDMPEnabled(true);
    m_dmpStatus = true;
    g_fifoPacketSize = m_mpu.dmpGetFIFOPacketSize();

    if(!IMUSensor::begin(wire))
    {
        Serial.printf("\n[MPU6050IMU] Falha na criacao do semaforo!");
        return false;
    }

    Serial.printf("\n[MPU6050IMU] Conexao com a MPU iniciada.");

    return true;
}

bool MPU6050IMU::calibrate()
{
    Serial.printf("\n[MPU6050IMU] Iniciando processo de calibracao...");
    m_mpu.CalibrateAccel(10);
    m_mpu.CalibrateGyro(10);
    Serial.printf("\n[MPU6050IMU] Calibracao concluida!");

    // Depois adicionar verificações para checar se a calibração realmente funcionou.
    return true;
}

void MPU6050IMU::updateData()
{
    if(!m_dmpStatus)
        return;
    
    g_fifoCount = m_mpu.getFIFOCount();

    if(g_fifoCount > 1023)
    {
        m_mpu.resetFIFO();
        Serial.printf("\n[MPU6050IMU] FIFO overflow!");
    }
    else
    {
        while (g_fifoCount < g_fifoPacketSize) 
            g_fifoCount = m_mpu.getFIFOCount();

        g_fifoCount -= g_fifoPacketSize;
        m_mpu.dmpGetCurrentFIFOPacket(g_fifoBuffer);

        m_mpu.dmpGetQuaternion(&g_quart, g_fifoBuffer);
        m_mpu.dmpGetGravity(&g_gravity, &g_quart);
        m_mpu.dmpGetYawPitchRoll(g_YPR, &g_quart, &g_gravity);
        
        IMUAxisData_t data;
        data.Yaw  = g_YPR[0] * g_degreeRad;
        data.Pitch = g_YPR[1] * g_degreeRad;
        data.Roll = g_YPR[2] * g_degreeRad;
        data.Acc_X = (double) m_mpu.getAccelerationX()/16384;
        data.Acc_Y = (double) m_mpu.getAccelerationY()/16384;
        data.Acc_Z = (double) m_mpu.getAccelerationZ()/16384;
        data.Gyro_X = (double) m_mpu.getRotationX()/131;
        data.Gyro_Y = (double) m_mpu.getRotationY()/131;
        data.Gyro_Z = (double) m_mpu.getRotationZ()/131;
        data.Time = millis();

        if(abs(millis() - g_timeLastRead) > m_readFrequency)
        {
            Serial.printf("\n\nYPR: [%.2f]   [%.2f]   [%.2f]", data.Yaw, data.Pitch, data.Roll);

            g_timeLastRead = millis();
            addMeasurement(data);

            detectTipping();
            if(m_moving)
                detectStop();
            else
                detectMovement();
        }
    }
}

void MPU6050IMU::start(int frequency)
{
    if(!m_threadRunning && m_dmpStatus && m_mpu.testConnection() && m_semaphoreInitialized)
    {
        m_readFrequency = frequency;
        xTaskCreate(wrapper, "[MPU6050]readTask", 10000, this, 1, &g_readTaskHandle);

        xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
        m_threadRunning = true;
        xSemaphoreGive(m_imuSemaphore);

        Serial.printf("\n[MPU6050IMU] Thread de leitura iniciada.");
    }
}

void MPU6050IMU::stop()
{
    if(m_threadRunning && g_readTaskHandle != NULL && m_semaphoreInitialized)
    {
        xSemaphoreTake(m_imuSemaphore, portMAX_DELAY);
        m_threadRunning = false;
        xSemaphoreGive(m_imuSemaphore);

        resetMeasurements();

        vTaskDelete(g_readTaskHandle);
        g_readTaskHandle = NULL;
    }
}

void MPU6050IMU::wrapper(void * parameter)
{
    for(;;)
        static_cast<MPU6050IMU*>(parameter)->updateData();
}