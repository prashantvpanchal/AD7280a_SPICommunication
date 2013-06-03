/*
 *  AD7280a_SPICommunication.ino
 *
 * 	Created: 12/31/2012 4:44:02 AM
 *  Author: prashant veerbrahma panchal
 *  Description : This program is developed during my internship. one can test the AD7280a EVal board or the AD7280a chained devices using Arduino platform.
 *  The library lib_ad7280a was modified from the Linux device driver of AD7280a.
 *  The library is made a bit more portable. The program is tested with the AD7280a EVal Board and Arduino MEGA.
 */ 
 
#include <Arduino.h>
#include <SPI.h>
#include "lib_ad7280a.h"


#ifndef _SS
#define _SS 45
#endif

#define _PDEBUG


struct ad7280_state ADinst;
uint16_t valCh[24]; // Statically assigned the memory for only 2 two AD7280a channels. If you need more then declare more or write your own routine.
void setup()
{
  Serial.begin(9600);
  Serial.println("Setup SPI32 for AD7280a chip");
  setup_spi32();
  ADinst.readback_delay_ms = 10; // 10 ms wait time.
  ADinst.cell_threshhigh = 0xFF;
  ADinst.aux_threshhigh = 0xFF;
  ADinst.ctrl_hb = 0x00;
  ADinst.ctrl_lb = 0x10; // D4 of Ctrl_lb must be 1 ( reserved Bit )
  ADinst.cell_threshlow = 0x00;
  ADinst.aux_threshlow = 0x00;


return ;
}


void loop()
{
uint32_t val;
byte a;

do
{

	a = Serial.read(); 
	//Serial.println(a,DEC);
	if( a == 'a')	// Datasheet Initialise
	{
		Serial.println("");
		Serial.println("");
		Serial.println("Proc 1: Datasheet Initialise");
 
		val=0x01C2B6E2;
		transferspi32(&val);
		Serial.print("Check routine returns ");
		a = ad7280_check_crc( &ADinst, val);
		Serial.println(a,BIN);
 
		val=0x038716CA;
		transferspi32(&val);
		Serial.print("Check routine returns ");
		a = ad7280_check_crc( &ADinst, val);
		Serial.println(a,BIN);

		val=0xF800030A;
		transferspi32(&val);
		Serial.print("Check routine returns ");
		a = ad7280_check_crc( &ADinst, val);
		Serial.println(a,BIN);

		val=0xF800030A;
		transferspi32(&val);		
		Serial.print("Check routine returns ");
		a = ad7280_check_crc( &ADinst, val);
		Serial.println(a,BIN);
	}

	if( a == 'b') // Building the CRC Table
	{
		Serial.println("");
		Serial.println("");
		Serial.println("Proc 2: CRC Table");
		ad7280_crc8_build_table( &ADinst.crc_tab[0]); 
		Serial.println("CRC Table Built");
    }
	
	if( a == 'c') // CRC write check
	{
		Serial.println("");
		Serial.println("");
		Serial.println("Proc 3: CRC Write Calc Check");
		val = 0x81A183A2; // To transmit 
		show32bit(&val);
		a = ad7280_calc_crc8(&ADinst.crc_tab[0], val>>11);
		Serial.println(a,BIN);
	}
	
	if ( a == 'd') // CRC read check 
	{
		Serial.println("");
		Serial.println("");
		Serial.println("Proc 4: CRC Read Calc Check");
		val = 0x01C28668;  // recieived
		show32bit(&val);
		Serial.print("Check routine returns ");
		a = ad7280_check_crc( &ADinst, val);
		Serial.println(a,BIN);
		Serial.print("CRC returned ");
		a = ad7280_calc_crc8(&ADinst.crc_tab[0], val >> 10);
		Serial.println(a,BIN);
	}

	if( a == 'e')  // All Setup
	{
		Serial.println("");
		Serial.println("");
		Serial.println("Proc 5: All Setup");
		Serial.println("CRC table setup");
		ad7280_crc8_build_table(&ADinst.crc_tab[0]);
		Serial.println("Daisy Chain Setup");
		a = ad7280_chain_setup( &ADinst );
		Serial.print("No of Slaves found: ");
		ADinst.slave_num = a;
		Serial.println(ADinst.slave_num,DEC);
		ADinst.scan_cnt = (ADinst.slave_num + 1) * AD7280A_NUM_CH;
		Serial.print("Total channels found: ");
		Serial.println(ADinst.scan_cnt,DEC);
		Serial.println("Write Bal Reg Zero");
		delay(10);
		ad7280_write(&ADinst, AD7280A_DEVADDR(1),
		AD7280A_CELL_BALANCE, 0, 0);
		delay(10);
		ad7280_write(&ADinst, AD7280A_DEVADDR(0),
		AD7280A_CELL_BALANCE, 0, 0);
		Serial.println("=================");
	}

	if( a == 'f')  // Read Master channels
	{
		uint16_t recv_data;
		Serial.println("");
		Serial.println("");
		Serial.println("=================");
		Serial.println("Proc 6: Read Master channels");

		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR_MASTER,
						AD7280A_CELL_VOLTAGE_1);
		Serial.print("VIN_1 : ");
		Serial.println(recv_data,DEC);
		
		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR_MASTER,
						AD7280A_CELL_VOLTAGE_2);
		Serial.print("VIN_2 : ");
		Serial.println(recv_data,DEC);

		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR_MASTER,
						AD7280A_CELL_VOLTAGE_3);
		Serial.print("VIN_3 : ");
		Serial.println(recv_data,DEC);
		
		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR_MASTER,
						AD7280A_CELL_VOLTAGE_4);
		Serial.print("VIN_4 : ");
		Serial.println(recv_data,DEC);
		
		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR_MASTER,
						AD7280A_CELL_VOLTAGE_5);
		Serial.print("VIN_5 : ");
		Serial.println(recv_data,DEC);
		
		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR_MASTER,
						AD7280A_CELL_VOLTAGE_6);
		Serial.print("VIN_6 : ");
		Serial.println(recv_data,DEC);
	}

	if( a == 'g')  // Read Slave channels
	{
		uint16_t recv_data;
		Serial.println("");
		Serial.println("");
		Serial.println("=================");
		Serial.println("Proc 7: Read Slave channels");

		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR(1),
						AD7280A_CELL_VOLTAGE_1);
		Serial.print("VIN_1 : ");
		Serial.println(recv_data,DEC);

		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR(1),
						AD7280A_CELL_VOLTAGE_2);
		Serial.print("VIN_2 : ");
		Serial.println(recv_data,DEC);

		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR(1),
						AD7280A_CELL_VOLTAGE_3);
		Serial.print("VIN_3 : ");
		Serial.println(recv_data,DEC);
		
		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR(1),
						AD7280A_CELL_VOLTAGE_4);
		Serial.print("VIN_4 : ");
		Serial.println(recv_data,DEC);
		
		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR(1),
						AD7280A_CELL_VOLTAGE_5);
		Serial.print("VIN_5 : ");
		Serial.println(recv_data,DEC);
			
		recv_data = ad7280_read_channel( &ADinst, AD7280A_DEVADDR(1),
						AD7280A_CELL_VOLTAGE_6);
		Serial.print("VIN_6 : ");
		Serial.println(recv_data,DEC);
	}

	if( a == 'h')  // Read all and print sum
	{
		uint16_t sum;
		float result;
		Serial.println("");
		Serial.println("");
		Serial.println("=================");
		Serial.print("Proc 8: Read all and Total Voltage in mV: ");

		for (a = 0 ; a < 24 ; a ++ )
			valCh[0]=0;

		sum = ad7280_read_all_channels(&ADinst, ADinst.scan_cnt,
					&valCh[0]);
		result = sum * 0.976000 + 1000*ADinst.scan_cnt/2 ; // channel ADC counts is 0.976 mill volts
		Serial.println(result,DEC);
	}


	if( a == 'i')  // Print all the channels
	{	
		uint8_t i;
		float result;
		Serial.println("");
		Serial.println("");
		Serial.println("=================");
		Serial.println("Proc 9: Print all the ADC counts of channels");	
		ad7280_read_all_channels(&ADinst, ADinst.scan_cnt,
					&valCh[0]);
		for(i =0 ; i < ADinst.scan_cnt ; i++ )
		{
			if( ((i/6)%2) == 0 )
			//if(1)
			{
			Serial.print("VIN_");
			Serial.print( (1+i),DEC );
			Serial.print(": ");
			Serial.println(valCh[i],DEC);
			}
			else
			{
			Serial.print("AUX_");
			Serial.print( (1+i),DEC );
			Serial.print(": ");
			Serial.println(valCh[i],DEC);
			}
		}
	}

	delay(500);
}while(1);

delay(1000);
Serial.println("Continue main loop");
return;
}

void setup_spi32()
{
  SPI.begin();
  SPI.setDataMode(SPI_MODE1);
  SPI.setBitOrder(MSBFIRST); // MSB first.
  SPI.setClockDivider(SPI_CLOCK_DIV16); // 1 Mhz Clk fo SPI. 16Mhz Div 16 = 1 Mhz.s
  pinMode(_SS, OUTPUT);
}

void transferspi32( uint32_t *val)
{
  byte h_h; // D31-D24 received
  byte h_l; // D23-D16 received
  byte l_h; // D15 -D8 received
  byte l_l; // D07 -D0 received

#ifndef _PDEBUG
  h_h = ((*val)>>24) & 0xFF;
  h_l = ((*val)>>16) & 0xFF;
  l_h = ((*val)>>8) & 0xFF;
  l_l = (*val) & 0xFF;
  Serial.println("=================");
  Serial.println("Sending SPI32bit");
  Serial.print("HEX: ");
  Serial.print(h_h,HEX);
  Serial.print(",");
  Serial.print(h_l,HEX);
  Serial.print(",");
  Serial.print(l_h,HEX);
  Serial.print(",");
  Serial.println(l_l,HEX);
  Serial.print("BIN: ");
  Serial.print(h_h,BIN);
  Serial.print(",");
  Serial.print(h_l,BIN);
  Serial.print(",");
  Serial.print(l_h,BIN);
  Serial.print(",");
  Serial.println(l_l,BIN);
#endif

  digitalWrite(_SS,LOW);
  h_h = SPI.transfer( ((*val)>>24) & 0xFF );  // D31-D24
  h_l = SPI.transfer( ((*val)>>16) & 0xFF );  // D23-D16
  l_h = SPI.transfer( ((*val)>>8) & 0xFF );   // D15-D08
  l_l = SPI.transfer( (*val) & 0xFF );        // D07-D00
  digitalWrite(_SS,HIGH);
  *val = (uint32_t) (((uint32_t) h_h <<24) | ((uint32_t) h_l <<16) | ((uint32_t) l_h << 8) | l_l);

 #ifndef _PDEBUG
  
  Serial.println("Receiving SPI32bit");
  Serial.print("HEX: ");
  Serial.print(h_h,HEX);
  Serial.print(",");
  Serial.print(h_l,HEX);
  Serial.print(",");
  Serial.print(l_h,HEX);
  Serial.print(",");
  Serial.println(l_l,HEX);
  Serial.print("BIN: ");
  Serial.print(h_h,BIN);
  Serial.print(",");
  Serial.print(h_l,BIN);
  Serial.print(",");
  Serial.print(l_h,BIN);
  Serial.print(",");
  Serial.println(l_l,BIN);
#endif

  return ; // Always return zero. Even if the things are not successful
}

void show32bit( uint32_t *val)
{
  byte h_h; // D31-D24 received
  byte h_l; // D23-D16 received
  byte l_h; // D15 -D8 received
  byte l_l; // D07 -D0 received

  h_h = ((*val)>>24) & 0xFF;
  h_l = ((*val)>>16) & 0xFF;
  l_h = ((*val)>>8) & 0xFF;
  l_l = (*val) & 0xFF;
  Serial.print("HEX: ");
  Serial.print(h_h,HEX);
  Serial.print(",");
  Serial.print(h_l,HEX);
  Serial.print(",");
  Serial.print(l_h,HEX);
  Serial.print(",");
  Serial.println(l_l,HEX);
  Serial.print("BIN: ");
  Serial.print(h_h,BIN);
  Serial.print(",");
  Serial.print(h_l,BIN);
  Serial.print(",");
  Serial.print(l_h,BIN);
  Serial.print(",");
  Serial.println(l_l,BIN);
  return;
}

