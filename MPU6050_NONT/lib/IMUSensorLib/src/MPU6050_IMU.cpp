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

MPU6050IMU::MPU6050IMU()
{
    m_deviceStatus = 0;
    m_dmpStatus = false;
    m_moving = false;
    m_tipped = false;
    m_tamper = false;
    m_devState = DeviceState_e::STATE_STOPPED;
}

bool MPU6050IMU::begin(TwoWire &wire)
{
    IMUOffsets_t offsets = IMUOffsets_t(-2314, 777, 1054, 144, 24, 16);

    return begin(wire, offsets);
}

bool MPU6050IMU::begin(TwoWire &wire, IMUOffsets_t offsets)
{
    wire.begin(MPU6050_PIN_SDA, MPU6050_PIN_SCL, MPU6050_FREQUENCY);
    m_mpu.initialize();

    if(!m_mpu.testConnection())
        return false;

    m_deviceStatus = m_mpu.dmpInitialize();

    m_mpu.setXAccelOffset(offsets.XAccelOffset);
    m_mpu.setYAccelOffset(offsets.YAccelOffset);
    m_mpu.setZAccelOffset(offsets.ZAccelOffset);
    m_mpu.setXGyroOffset(offsets.XGyroOffset);
    m_mpu.setYGyroOffset(offsets.YGyroOffset);
    m_mpu.setZGyroOffset(offsets.ZGyroOffset);

    if(m_deviceStatus != 0)
        return false;

    m_mpu.setDMPEnabled(true);
    m_dmpStatus = true;
    g_fifoPacketSize = m_mpu.dmpGetFIFOPacketSize();

    if(!IMUSensor::begin(wire))
        return false;
    
    return true;
}

IMUOffsets_t MPU6050IMU::calibrate()
{
    m_mpu.CalibrateAccel(10);
    m_mpu.CalibrateGyro(10);

    IMUOffsets_t newOffsets = IMUOffsets_t();

    newOffsets.XAccelOffset = m_mpu.getXAccelOffset();
    newOffsets.YAccelOffset = m_mpu.getYAccelOffset();
    newOffsets.ZAccelOffset = m_mpu.getZAccelOffset();
    newOffsets.XGyroOffset = m_mpu.getXGyroOffset();
    newOffsets.YGyroOffset = m_mpu.getYGyroOffset();
    newOffsets.ZGyroOffset = m_mpu.getZGyroOffset();

    return newOffsets;
}

void MPU6050IMU::acquireData()
{
    if(!m_dmpStatus || !checkConfigurations() || !m_mpu.testConnection())
        return;
    
    g_fifoCount = m_mpu.getFIFOCount();

    if(g_fifoCount > 1023)
    {
        m_mpu.resetFIFO();
        Serial.printf("\nFIFO Overflow!");
        g_fifoCount = m_mpu.getFIFOCount();
    }

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
    data.Temperature = ((double) m_mpu.getTemperature()/340) + 36.53;
    data.Time = millis();

    g_timeLastRead = millis();
    addMeasurement(data);
}

IMUOffsets_t MPU6050IMU::getCurrentOffsets()
{
    IMUOffsets_t currentOffsets;

    currentOffsets.XAccelOffset = m_mpu.getXAccelOffset();
    currentOffsets.YAccelOffset = m_mpu.getYAccelOffset();
    currentOffsets.ZAccelOffset = m_mpu.getZAccelOffset();
    currentOffsets.XGyroOffset = m_mpu.getXGyroOffset();
    currentOffsets.YGyroOffset = m_mpu.getYGyroOffset();
    currentOffsets.ZGyroOffset = m_mpu.getZGyroOffset();

    return currentOffsets;
}

void MPU6050IMU::setOffsets(IMUOffsets_t newOffsets)
{
    m_mpu.setXAccelOffset(newOffsets.XAccelOffset);
    m_mpu.setYAccelOffset(newOffsets.YAccelOffset);
    m_mpu.setZAccelOffset(newOffsets.ZAccelOffset);
    m_mpu.setXGyroOffset(newOffsets.XGyroOffset);
    m_mpu.setYGyroOffset(newOffsets.YGyroOffset);
    m_mpu.setZGyroOffset(newOffsets.ZGyroOffset);
}

MPU6050IMU MPU;