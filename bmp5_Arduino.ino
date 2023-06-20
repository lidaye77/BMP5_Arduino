/**
 * Copyright (C) Bosch Sensortec GmbH. All Rights Reserved. Confidential.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet. Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchaser's own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

/*!
 * @file bmp5_Arduino.ino
 *
 * @brief
 * Example for using BMP5XX to get the data of temperature and pressure.
 * This works by running an endless loop in the get_sensor_data() function.
 */

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/#include <Wire.h>

#include "bmp5.h"
static uint8_t dev_addr = BMP5_I2C_ADDR_PRIM;

/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
 * @brief           Write operation in Wire
 *
 * param[in]        reg_addr        register address
 * param[in]       	reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 * param[in]        intf_ptr        interface pointer
 *
 * @return          result of the bus communication function
 */


int8_t bmp5_write(uint8_t reg_addr, uint8_t* reg_data_ptr, uint16_t data_len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
	
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);    /* Set register address to start writing to */
 
    /* Write the data */
    for (int index = 0; index < data_len; index++) {
        Wire.write(reg_data_ptr[index]);
    }
 
    return (int8_t)Wire.endTransmission();
}

/*!
 * @brief           Read operation in either Wire or SPI
 *
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
int8_t bmp5_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    int8_t comResult = 0;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);                    /* Set register address to start reading from */
    comResult = Wire.endTransmission();
 
    delayMicroseconds(150);                 /* Precautionary response delay */
    Wire.requestFrom(dev_addr, (uint8_t)data_len);    /* Request data */
 
    int index = 0;
    while (Wire.available())  /* The slave device may send less than requested (burst read) */
    {
        reg_data_ptr[index] = Wire.read();
        index++;
    }
 
    return comResult;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms    time in microseconds
 *
 * @return          none
 */
void bmp5_sleep(uint32_t t_ms)
{
    uint64_t t_real_ms = t_ms /1000;
    delay(t_real_ms);
}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    return (int64_t) millis() * 1000;
}

/*!
 * @brief       Main function which configures BMP5XX and then reads and processes the data from sensor based
 *              on ODR and OSR
 *
 * @return      none
 */
void setup()
{
	int8_t rslt;
    struct bmp5_dev dev;
    struct bmp5_osr_odr_press_config osr_odr_press_cfg = { 0 };
	
	/* Init I2C and serial communication */
    Wire.begin();
    Serial.begin(115200);

	/*Configure the structure of BMP5XX*/
    dev.dev_id = BMP5_I2C_ADDR_PRIM;
    dev.read = bmp5_read;
    dev.write = bmp5_write;
    dev.intf = BMP5_I2C_INTF;
    dev.intf_ptr = &dev_addr;
    dev.delay_us = bmp5_sleep;

    /* Call to the function which initializes the BMP5XX*/
    rslt = bmp5_init(&dev);


    if (rslt == BMP5_OK)
    {
        rslt = set_config(&osr_odr_press_cfg, &dev);
    }
	else
	{
		Serial.println("bmp5_init failed !");
		return;
	}
    
	if (rslt == BMP5_OK)
    {
        rslt = get_sensor_data(&osr_odr_press_cfg, &dev);
    }
	else
	{
		Serial.println("set_config failed !");
		return;
	}
	
	if (rslt == BMP5_OK)
    {
        Serial.println("TimeStamp ->> Data, Pressure (Pa), Temperature (deg C)\n");
    }
	else
	{
		Serial.println("get_sensor_data failed !");
		return;
	}
	
    return;
  
}

void loop()
{
}

static int8_t set_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    int8_t rslt;
    struct bmp5_iir_config set_iir_cfg;
    struct bmp5_int_source_select int_source_select;

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);

    if (rslt == BMP5_OK)
    {
        /* Get default odr */
        rslt = bmp5_get_osr_odr_press_config(osr_odr_press_cfg, dev);

        if (rslt == BMP5_OK)
        {
            /* Set ODR as 30Hz */
            osr_odr_press_cfg->odr = BMP5_ODR_30_HZ;

            /* Enable pressure */
            osr_odr_press_cfg->press_en = BMP5_ENABLE;

            /* Set Over-sampling rate with respect to odr */
            osr_odr_press_cfg->osr_t = BMP5_OVERSAMPLING_1X;
            osr_odr_press_cfg->osr_p = BMP5_OVERSAMPLING_16X;

            rslt = bmp5_set_osr_odr_press_config(osr_odr_press_cfg, dev);
        }

        if (rslt == BMP5_OK)
        {
            set_iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
            set_iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
            set_iir_cfg.shdw_set_iir_t = BMP5_ENABLE;
            set_iir_cfg.shdw_set_iir_p = BMP5_ENABLE;

            rslt = bmp5_set_iir_config(&set_iir_cfg, dev);
        }

        /* Set powermode as normal */
        rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, dev);
    }

    return rslt;
}

static int8_t get_sensor_data(const struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    int8_t rslt = 0;
    uint64_t idx = 0;
    uint8_t int_status;
    struct bmp5_sensor_data sensor_data;
    uint32_t time_ms = 0;
    uint8_t reg_data = 0;

    while (1)
    {
        time_ms = get_timestamp_us()/1000;
        rslt = bmp5_get_sensor_data(&sensor_data, osr_odr_press_cfg, dev);

        if (rslt == BMP5_OK)
        {
#ifdef BMP5_USE_FIXED_POINT
			Serial.print("Index: ");
			Serial.print(idx);
			Serial.print(" TimeStamp: ");
			Serial.print(time_ms);
			Serial.print(" Pressure: ");
            Serial.print(sensor_data.pressure);
			Serial.print(" Temperature: ");
			Serial.println(sensor_data.temperature);
#else
			Serial.print("Index: ");
			Serial.print(idx);
			Serial.print(" TimeStamp: ");
			Serial.print(time_ms);
			Serial.print(" Pressure: ");
            Serial.print(sensor_data.pressure);
			Serial.print(" Temperature: ");
			Serial.println(sensor_data.temperature);
#endif
		}
        idx++;
    }

    return rslt;
}
