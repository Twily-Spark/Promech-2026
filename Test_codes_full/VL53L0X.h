/**
 * @file VL53L0X.h
 * @author Joel von Rotz (joel.vonrotz@maxonmotor.com)
 * @brief 
 * @version 0.1
 * @date 2019-07-30
 * 
 * @copyright Copyright (c) 2019, maxon motor ag - all rights reserved
 * 
 */
#ifndef VL53L0X_H
#define VL53L0X_H

#include "mbed.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_def.h"

#define ADDRESS_DEFAULT (0x29)

/**
 * @brief Class for Proximity Sensor VL53L0X
 * 
 * The VL53L0X is a laser based proximity sensor which can measure up to 2m.
 * This library is a wrapper for the API from STMicrocontrollers and the goal of this
 * library is the easier access to this sensor. 
 * 
 * Something really cool about these sensors is the ability to change the I2C-address.
 * So you can hook multiple, identical sensors on the same I2C-line.
 * 
 * <h2>Examples</h2>
 * 
 * <h3>One Sensor</h3>
 * Using one sensor is pretty simple, connect the I2C-pins to the sensor (Shutdown should be pulled high
 * using a pull-up resistor).
 * 
 * @code
 * #include "mbed.h"
 * #include "VL53L0X.h"
 *
 * I2C         i2c(p9, p10);
 * VL53L0X     vl_sensor(&i2c);
 * DigitalOut  vl_shutdown(p11);
 * Serial      usb(USBTX, USBRX, 115200);
 *
 * int main(void)
 * {
 *   usb.printf("Single VL53L0X\n\n\r");
 *  
 *   vl_shutdown = 1;  //turn VL53L0X on
 *   vl_sensor.init();
 *   vl_sensor.setModeContinuous();
 *   vl_sensor.startContinuous();
 *   while(1)
 *   {
 *     usb.printf("%4imm\n\r", vl_sensor.getRangeMillimeters());
 *   }
 * }
 * @endcode
 * 
 * <h3>Multiple Sensors</h3>
 * To use multiple sensors, at the start of the programm every sensor needs to be
 * turned off by setting their shutdown pin to low. Now one sensor is turned on,
 * and configured with the new address. After that's done, the next sensor can be
 * configured.
 * 
 * @code
 * #include "mbed.h"
 * #include "VL53L0X.h"
 *
 * I2C         i2c(p9, p10);
 * VL53L0X     vl_sensors[6] = {(&i2c),(&i2c),(&i2c),(&i2c),(&i2c),(&i2c)};
 * BusOut      vl_shutdown(p11,p12,p13,p14,p15,p16);
 * Serial      usb(USBTX, USBRX, 115200);
 *
 * int main(void)
 * {
 *   usb.printf("Multiple VL53L0X\n\n\r");
 * 
 *   uint8_t expander_shutdown_mask = 1;
 *   for(uint8_t i = 0; i < 6 ; i++)
 *   {
 *     vl_shutdown = expander_shutdown_mask;
 *     expander_shutdown_mask = (expander_shutdown_mask << 1) + 1;
 *     vl_sensors[i].init();
 *     vl_sensors[i].setDeviceAddress(0x40 + i);
 *     vl_sensors[i].setModeContinuous();
 *     vl_sensors[i].startContinuous();
 *   }
 *   uint16_t results[6];
 *   while(1)
 *   {
 *     for(uint8_t i = 0; i < 6 ; i++)
 *     {
 *       results[i] = vl_sensors[i].getRangeMillimeters(); 
 *     }
 *     usb.printf("1: %4imm 2: %4imm 3: %4imm) 4: %4imm 5: %4imm 6: %4imm)\n\r", results[0], results[1], results[2], results[3], results[4], results[5]);
 *   }
 * }
 * @endcode
 * 
 * <strong>NOTE</strong> - The library is gigantic, so it is recommended to use this library
 * on devices with enough storage.
 * 
 */
class VL53L0X
{
public:
  VL53L0X(I2C* p_i2c_device);

  bool     setDeviceAddress(uint8_t address = ADDRESS_DEFAULT);

  bool     init();

  bool     setModeContinuous();
  bool     startContinuous();
  uint16_t getRangeMillimeters();
private:
  VL53L0X_Dev_t                     m_vl_dev;
  VL53L0X_RangingMeasurementData_t  m_vl_results,m_old_results;

  VL53L0X_Version_t                 m_vl_version;
  VL53L0X_DeviceInfo_t              m_vl_deviceinfo;

};

#endif  /* */
