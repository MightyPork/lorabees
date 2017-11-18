//
// Created by MightyPork on 2017/11/17.
//

#include <hw.h>
#include <hw_i2c.h>
#include "voc_sensor.h"

struct bme680_dev gas_sensor;

static void user_delay_ms(uint32_t period)
{
	/*
	 * Return control or wait,
	 * for a period amount of milliseconds
	 */
	HAL_Delay(period);
}

#if 0
static int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

	/*
	 * The parameter dev_id can be used as a variable to select which Chip Select pin has
	 * to be set low to activate the relevant device on the SPI bus
	 */

	/*
	 * Data on the bus should be like
	 * |----------------+---------------------+-------------|
	 * | MOSI           | MISO                | Chip Select |
	 * |----------------+---------------------|-------------|
	 * | (don't care)   | (don't care)        | HIGH        |
	 * | (reg_addr)     | (don't care)        | LOW         |
	 * | (don't care)   | (reg_data[0])       | LOW         |
	 * | (....)         | (....)              | LOW         |
	 * | (don't care)   | (reg_data[len - 1]) | LOW         |
	 * | (don't care)   | (don't care)        | HIGH        |
	 * |----------------+---------------------|-------------|
	 */

	return rslt;
}

static int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

	/*
	 * The parameter dev_id can be used as a variable to select which Chip Select pin has
	 * to be set low to activate the relevant device on the SPI bus
	 */

	/*
	 * Data on the bus should be like
	 * |---------------------+--------------+-------------|
	 * | MOSI                | MISO         | Chip Select |
	 * |---------------------+--------------|-------------|
	 * | (don't care)        | (don't care) | HIGH        |
	 * | (reg_addr)          | (don't care) | LOW         |
	 * | (reg_data[0])       | (don't care) | LOW         |
	 * | (....)              | (....)       | LOW         |
	 * | (reg_data[len - 1]) | (don't care) | LOW         |
	 * | (don't care)        | (don't care) | HIGH        |
	 * |---------------------+--------------|-------------|
	 */

	return rslt;
}
#endif

static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

	/*
	 * The parameter dev_id can be used as a variable to store the I2C address of the device
	 */

	/*
	 * Data on the bus should be like
	 * |------------+---------------------|
	 * | I2C action | Data                |
	 * |------------+---------------------|
	 * | Start      | -                   |
	 * | Write      | (reg_addr)          |
	 * | Stop       | -                   |
	 * | Start      | -                   |
	 * | Read       | (reg_data[0])       |
	 * | Read       | (....)              |
	 * | Read       | (reg_data[len - 1]) |
	 * | Stop       | -                   |
	 * |------------+---------------------|
	 */
	HAL_I2C_Master_Transmit(&hi2c1, dev_id<<1, &reg_addr, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, dev_id<<1, reg_data, len, 100);

	return rslt;
}

static int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	/*
	 * The parameter dev_id can be used as a variable to store the I2C address of the device
	 */

	/*
	 * Data on the bus should be like
	 * |------------+---------------------|
	 * | I2C action | Data                |
	 * |------------+---------------------|
	 * | Start      | -                   |
	 * | Write      | (reg_addr)          |
	 * | Write      | (reg_data[0])       |
	 * | Write      | (....)              |
	 * | Write      | (reg_data[len - 1]) |
	 * | Stop       | -                   |
	 * |------------+---------------------|
	 */
	uint8_t data[64];
	data[0] = reg_addr;
	for (int i = 0; i < len; i++) {
		data[i+1] = reg_data[i];
	}
	HAL_I2C_Master_Transmit(&hi2c1, dev_id<<1, &data[0], (uint16_t) (len + 1), 100);

	return BME680_OK;
}


void voc_init(void)
{
	int8_t rslt;
	gas_sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
	gas_sensor.intf = BME680_I2C_INTF;
	gas_sensor.read = user_i2c_read;
	gas_sensor.write = user_i2c_write;
	gas_sensor.delay_ms = user_delay_ms;

	PRINTF("BME680 initializing...\r\n");
	rslt = bme680_init(&gas_sensor);
	assert_param(rslt == BME680_OK);


	PRINTF("BME680 configuring...\r\n");
	/* Set the temperature, pressure and humidity settings */
	gas_sensor.tph_sett.os_hum = BME680_OS_2X;
	gas_sensor.tph_sett.os_pres = BME680_OS_4X;
	gas_sensor.tph_sett.os_temp = BME680_OS_8X;
	gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

	/* Set the remaining gas sensor settings and link the heating profile */
	gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
	/* Create a ramp heat waveform in 3 steps */
	gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
	gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

	/* Select the power mode */
	/* Must be set before writing the sensor configuration */
	gas_sensor.power_mode = BME680_FORCED_MODE;

	/* Set the required sensor settings needed */
	uint8_t set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;

	/* Set the desired sensor configuration */
	rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);
	assert_param(rslt == BME680_OK);
}

void voc_measure(void)
{
	/* Set the power mode */
	int8_t rslt;

	rslt = bme680_set_sensor_mode(&gas_sensor);
	assert_param(rslt == BME680_OK);

	/* Get the total measurement duration so as to sleep or wait till the
	 * measurement is complete */
	uint16_t meas_period;
	bme680_get_profile_dur(&meas_period, &gas_sensor);
	HAL_Delay(meas_period); /* Delay till the measurement is ready */
	struct bme680_field_data data;

	rslt = bme680_get_sensor_data(&data, &gas_sensor);
	assert_param(rslt == BME680_OK);

	PRINTF("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
		   data.pressure / 100.0f, data.humidity / 1000.0f );
	/* Avoid using measurements from an unstable heating setup */
	if(data.status & BME680_GASM_VALID_MSK)
		PRINTF(", G: %d ohms", data.gas_resistance);

	PRINTF("\r\n");
}
