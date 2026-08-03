/**
 * @file VL53L0X.cpp
 * @author Joel von Rotz (joel.vonrotz@maxonmotor.com)
 * @brief 
 * @version 0.1
 * @date 2019-07-30
 * 
 * @copyright Copyright (c) 2019, maxon motor ag - all rights reserved
 * 
 */
#include "VL53L0X.h"
#include "mbed.h"

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_platform.h"

/**
 * @brief Construct a new VL53L0X object
 * 
 * The constructor is used to assign the needed values to the VL53L0X-Dev structure.
 * 
 * @param p_i2c_device A pointer to an mbed I2C-object
 */
VL53L0X::VL53L0X(I2C* p_i2c_device)
{
  m_vl_dev.i2c_frequency = 400000;
  m_vl_dev.i2c_device    = p_i2c_device;
  m_vl_dev.i2c_address   = ADDRESS_DEFAULT;

}

/**
 * @brief Set new Device Address
 * 
 * Replaces the current I2C-Address with a new one. Every value can be used.
 * Currently there is a small problem on reseting the mbed board: after reset (no power reset) the sensor still
 * uses it's new slave address. A power-reset is needed to reset the proximity sensors to their factory settings.
 * 
 * @param address   New desired 7-Bit address
 * @return true     no success
 * @return false    success
 */
bool VL53L0X::setDeviceAddress(uint8_t address)
{
  if(VL53L0X_SetDeviceAddress(&m_vl_dev, address << 1) == 0)
  {
    m_vl_dev.i2c_address = address;
    return 0;
  }
  return 1;
}

/**
 * @brief Initializes the Sensor
 * 
 * After constructing a VL53L0X object, the sensor needs to be initialized by calling this funciton.
 * 
 * @return true   never going to happen
 * @return false  maybe a success
 */
bool VL53L0X::init()
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;

  VL53L0X_GetVersion(&m_vl_version);

  if (Status == VL53L0X_ERROR_NONE)
  {
    Status = VL53L0X_DataInit(&m_vl_dev); // Data initialization
  }
  if (Status == VL53L0X_ERROR_NONE)
  {
    Status = VL53L0X_GetDeviceInfo(&m_vl_dev, &m_vl_deviceinfo);
  }
  return 0;
}

/**
 * @brief Set Mode to Continuous
 * 
 * Configures the data acquisition of the sensor to 'continuous' 
 * 
 * @return true   never going to happen
 * @return false  maybe a success
 */
bool VL53L0X::setModeContinuous()
{
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  VL53L0X_StaticInit(&m_vl_dev); // Device Initialization
  VL53L0X_PerformRefCalibration(&m_vl_dev,    &VhvSettings, &PhaseCal); // Device Initialization
  VL53L0X_PerformRefSpadManagement(&m_vl_dev, &refSpadCount, &isApertureSpads); // Device Initialization
  VL53L0X_SetDeviceMode(&m_vl_dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
  
  return 0;
}

/**
 * @brief Starts the Continuous Measurement Session
 * 
 * @return true   never going to happen
 * @return false  maybe a success
 */
bool VL53L0X::startContinuous()
{
  VL53L0X_StartMeasurement(&m_vl_dev);
  return 0;
}

/**
 * @brief Returns measured Range in Millimeters
 * 
 * @return uint16_t distance in [mm]
 */
uint16_t VL53L0X::getRangeMillimeters()
{
  VL53L0X_GetRangingMeasurementData(&m_vl_dev, &m_vl_results);
  return m_vl_results.RangeMilliMeter;
}
