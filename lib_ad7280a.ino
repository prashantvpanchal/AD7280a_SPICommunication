/*
 *  lib_ad7280a.c
 *  AD7280A Lithium Ion Battery Monitoring System
 *  Copyright 2011 Analog Devices Inc.
 *  Licensed under the GPL-2.
 *  Remodified from Linux/drivers/staging/iio/adc/ad7280a.c
 * 	Created: 12/31/2012 4:44:02 AM
 *  Modifying author: prashant veerbrahma panchal
 */ 
 
// Builds the CRC table. Returns not relevant. But fills the crc_tab
static void ad7280_crc8_build_table(uint8_t *crc_tab)
{
	uint8_t bit, crc;
	int16_t cnt, i;

	for (cnt = 0; cnt < 256; cnt++) {
		crc = cnt;
		for (i = 0; i < 8; i++) {
			bit = crc & HIGHBIT;
			crc <<= 1;
			if (bit)
				crc ^= POLYNOM;
		}
		crc_tab[cnt] = crc;
	}
}

// Calculates the CRC for the transmission packets: Returns CRC(8bit)
static uint8_t ad7280_calc_crc8(uint8_t *crc_tab, uint32_t val)
{
	uint8_t crc;

	crc = crc_tab[(uint16_t)(val >> 16 & 0xFF)];
	crc = crc_tab[crc ^ (val >> 8 & 0xFF)];

	return  crc ^ (val & 0xFF);
}

// Check crc routine for the received packets. Returns 0 if correct else -1.
static int8_t ad7280_check_crc(struct ad7280_state *st, uint32_t val)
{
	uint8_t crc = ad7280_calc_crc8(st->crc_tab, val >> 10);

	if (crc != ((val >> 2) & 0xFF))
		return -1;

	return 0;
}

// Delay 
static void ad7280_delay(struct ad7280_state *st) 
{
delay(st->readback_delay_ms);
}

// Send valid frame on SPI. Returns not relevant. but val will contain the received value.
static int8_t __ad7280_read32(struct spi_device *spi, uint32_t *val)
{
	*val = AD7280A_READ_TXVAL; //0xF800030A
	transferspi32(val); // recieved value will be in <val>
	return 0;
}

// Send Data (val) to devices. Returns not relevant.
static int8_t ad7280_write(struct ad7280_state *st, uint32_t devaddr,
			uint32_t addr, uint8_t all, uint32_t val)
{
	uint32_t reg = (devaddr << 27 | addr << 21 |
			(val & 0xFF) << 13 | all << 12);

	reg |= ad7280_calc_crc8(st->crc_tab, reg >> 11) << 3 | 0x2;
	reg = (uint32_t)(reg);

	//return spi_write(st->spi, &reg, 4);  Wrtite to SPI.
	transferspi32(&reg);
	return	0;
}

// Read from a specific device register : Returns register value
static int8_t ad7280_read(struct ad7280_state *st, uint32_t devaddr,
			uint32_t addr)
{
	uint32_t tmp;

	/* turns off the read operation on all parts */
	ad7280_write(st, AD7280A_DEVADDR_MASTER, AD7280A_CONTROL_HB, 1,
			AD7280A_CTRL_HB_CONV_INPUT_ALL |
			AD7280A_CTRL_HB_CONV_RES_READ_NO |
			st->ctrl_hb);


	/* turns on the read operation on the addressed part */
	ad7280_write(st, devaddr, AD7280A_CONTROL_HB, 0,
			AD7280A_CTRL_HB_CONV_INPUT_ALL |
			AD7280A_CTRL_HB_CONV_RES_READ_ALL |
			st->ctrl_hb);


	/* Set the address of the register on the device to be read from */
	ad7280_write(st, devaddr, AD7280A_READ, 0, addr << 2);

	/* Valid read so that the value can be received*/
	__ad7280_read32(st->spi, &tmp);

	if (ad7280_check_crc(st, tmp)) // Check the received CRC
		return -1;

	if (((tmp >> 27) != devaddr) || (((tmp >> 21) & 0x3F) != addr))
		return -1; // Check if the received device and register address is correct.

	return (tmp >> 13) & 0xFF;
}

// Read from an Analog channel with start of Conversion as CS start. Returns the value of the ADC count received.
static uint16_t ad7280_read_channel(struct ad7280_state *st, uint32_t devaddr,
			       uint32_t addr)
{
	int8_t ret;
	uint32_t tmp;

	/* Write on Read Register of "the device" the Register address of channel*/
	ret = ad7280_write(st, devaddr, AD7280A_READ, 0, addr << 2);
	
	/* Write on Ctrl_HB of "all device" to convert all but not to read all.*/
	ret = ad7280_write(st, AD7280A_DEVADDR_MASTER, AD7280A_CONTROL_HB, 1,
			AD7280A_CTRL_HB_CONV_INPUT_ALL |
			AD7280A_CTRL_HB_CONV_RES_READ_NO |
			st->ctrl_hb);
	
	/* Write on Ctrl_HB of "the device" to convert all and to read all 
	and to start the conversion as CS start*/
	ret = ad7280_write(st, devaddr, AD7280A_CONTROL_HB, 0,
			AD7280A_CTRL_HB_CONV_INPUT_ALL |
			AD7280A_CTRL_HB_CONV_RES_READ_ALL |
			AD7280A_CTRL_HB_CONV_START_CS |
			st->ctrl_hb);

	/*Delay*/		
	ad7280_delay(st);

	/*Valid write to get the channel value*/
	__ad7280_read32(st->spi, &tmp);

	if (ad7280_check_crc(st, tmp)) // Check the received CRC
		return 0;

	if (((tmp >> 27) != devaddr) || (((tmp >> 23) & 0xF) != addr))
		return 1; // Check if the received device and register address is correct.

	return (tmp >> 11) & 0xFFF;
}

// Read from all devices the channels of VIN and AUX at CS frame start. Returns the sum of all the voltages.
static uint32_t ad7280_read_all_channels(struct ad7280_state *st, uint32_t cnt,
			     uint16_t *array)
{
	uint8_t  ret;
	uint32_t i;
	uint32_t tmp, sum = 0;
	/* Write to all the read registers of all the devices the address
	of first channel register i.e AD7280A_CELL_VOLTAGE_1*/
	ret = ad7280_write(st, AD7280A_DEVADDR_MASTER, AD7280A_READ, 1,
			   AD7280A_CELL_VOLTAGE_1 << 2);

	/*Write to all the Control HB of all the devices, 
	select all channels, read all, Start conversion at CS Start*/
	ret = ad7280_write(st, AD7280A_DEVADDR_MASTER, AD7280A_CONTROL_HB, 1,
			AD7280A_CTRL_HB_CONV_INPUT_ALL |
			AD7280A_CTRL_HB_CONV_RES_READ_ALL |
			AD7280A_CTRL_HB_CONV_START_CS |
			st->ctrl_hb);

	ad7280_delay(st);

	for (i = 0; i < cnt; i++) {
		__ad7280_read32(st->spi, &tmp);

		if (ad7280_check_crc(st, tmp))
			return 0xFFF;

			array[i] = ((tmp >> 11) & 0xFFF);
		/* only sum cell voltages */
		if (((tmp >> 23) & 0xF) <= AD7280A_CELL_VOLTAGE_6)
		{
			sum += ((tmp >> 11) & 0xFFF);
			//Serial.println(sum,HEX); //
		}
	}

	return sum;
}

// Daisy chain setup returns the number of devices or error code.
static int8_t ad7280_chain_setup(struct ad7280_state *st)
{
	uint32_t val;
	uint8_t n;
	// To reset the AD7280a.
	ad7280_write(st, AD7280A_DEVADDR_MASTER, AD7280A_CONTROL_LB, 1,
			AD7280A_CTRL_LB_DAISY_CHAIN_RB_EN |
			AD7280A_CTRL_LB_LOCK_DEV_ADDR |
			AD7280A_CTRL_LB_MUST_SET |
			AD7280A_CTRL_LB_SWRST |
			st->ctrl_lb);
	// Lock to the new address and 
	ad7280_write(st, AD7280A_DEVADDR_MASTER, AD7280A_CONTROL_LB, 1,
			AD7280A_CTRL_LB_DAISY_CHAIN_RB_EN |
			AD7280A_CTRL_LB_LOCK_DEV_ADDR |
			AD7280A_CTRL_LB_MUST_SET |
			st->ctrl_lb);
	
	// Set the read register of all the devices to AD7280A_CONTROL_LB
	ad7280_write(st, AD7280A_DEVADDR_MASTER, AD7280A_READ, 1,
			AD7280A_CONTROL_LB << 2);

	for (n = 0; n <= AD7280A_MAX_CHAIN; n++) {
		__ad7280_read32(st->spi, &val);
		
		if (val == 0) // If we get an Empty frame then we end the enumeration.
			return n - 1; // Minimum is 0 device excluding the master

		if (ad7280_check_crc(st, val))
			return 200;

		if (n != AD7280A_DEVADDR(val >> 27))
			return 200;
	}

	return 255;
}
