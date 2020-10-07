/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "pretty_effect.h"

/*
 This code displays some fancy graphics on the 320x240 LCD on an ESP-WROVER_KIT board.
 This example demonstrates the use of both spi_device_transmit as well as
 spi_device_queue_trans/spi_device_get_trans_result and pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

//User Bank 0
#define WHO_AM_I        0x00
#define USER_CTRL       0x03
#define PWR_MGMT_1      0x06
#define PWR_MGMT_2      0x07
#define INT_STATUS_1    0x1a
#define MEAS_ON         0x01
#define ACCEL_XOUT_H    0x2d
#define ACCEL_XOUT_L   0x2e
#define ACCEL_YOUT_H    0x2f
#define ACCEL_YOUT_L   0x30
#define ACCEL_ZOUT_H    0x31
#define ACCEL_ZOUT_L   0x32
#define GYRO_XOUT_H   0x33
#define GYRO_XOUT_L  0x34
#define GYRO_YOUT_H   0x35
#define GYRO_YOUT_L  0x36
#define GYRO_ZOUT_H   0x37
#define GYRO_ZOUT_L  0x38


//Bank 3
#define MAG_X_H    0x12
#define MAG_X_L   0x11
#define MAG_Y_H    0x14
#define MAG_Y_L   0x13
#define MAG_Z_H    0x16
#define MAG_Z_L   0x15
#define MAG_CNTL2 0x31
#define MAG_SINGLE 0x01
#define MAG_CONT1 0x02

#define REG_BANK_SEL    0x7f
#define BANK0           0x00
#define BANK1           0x10
#define BANK2           0x20
#define BANK3           0x30

#define MAG_CONT_1      0x03 
#define MAG_CNTL3      0x32   

#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22

#define PIN_NUM_DC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 5

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16
#define INV_MAX_SERIAL_READ 16

/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
// typedef struct {
//     uint8_t cmd;
//     uint8_t data[16];
//     uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
// } lcd_init_cmd_t;

// typedef enum {
//     LCD_TYPE_ILI = 1,
//     LCD_TYPE_ST,
//     LCD_TYPE_MAX,
// } type_lcd_t;



//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}


#define GPIO_OUTPUT_PIN_SEL (1ULL << PIN_NUM_CS)

void cs_gpio_setting()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //SET AS OUTPUT MODE
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set, e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void cs_low()
{
    gpio_set_level(PIN_NUM_CS, 0);
}

void cs_high()
{
    gpio_set_level(PIN_NUM_CS, 1);
}

bool imu_read_trans(spi_device_handle_t spi, uint8_t reg, uint8_t *data)
{
    //esp_err_t ret;
    spi_transaction_t t;

    cs_low(); // CSN low, initialize SPI communication...

    reg |=0x80;
    memset(&t, 0, sizeof(t));   //Zero out the transaction
    t.length = 8;               //Command is 8 bits
    t.tx_buffer = &reg;         //The data is the cmd itself
    spi_device_transmit(spi, &t);

    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.flags = SPI_TRANS_USE_RXDATA;
    spi_device_transmit(spi, &t);

    cs_high();                  //CSN high, terminate SPI communication

    *data = t.rx_data[0];

    return true;
}

bool imu_write_trans(spi_device_handle_t spi, uint8_t reg, uint8_t data)
{
    //esp_err_t ret;
    spi_transaction_t t;
    uint8_t buf[2];

    cs_low(); // CSN low, initialize SPI communication...

    reg &= 0x7f;
    buf[0] = reg;
    buf[1] = data;
    memset(&t, 0, sizeof(t));   //Zero out the transaction
    t.length = 8 * 2;           //Command is 8 bits
    t.tx_buffer = buf;          //The data is the cmd itself
    
    spi_device_transmit(spi, &t);

    cs_high();                  //CSN high, terminate SPI communication

    return true;
}

//read lsm6ds3 chip id;
void imu_read(spi_device_handle_t spi, uint8_t reg1, uint8_t reg2, int user, uint16_t *value)
{
    uint8_t id1;
    uint8_t id2;
    switch(user)
    {
        case 0:
            imu_write_trans(spi, REG_BANK_SEL, BANK0);
            break;
        case 1:
            imu_write_trans(spi, REG_BANK_SEL, BANK1);
            break;
        case 2:
            imu_write_trans(spi, REG_BANK_SEL, BANK2);
            break;
        case 3:
            imu_write_trans(spi, REG_BANK_SEL, BANK3);
            break;
        default:
            printf("ERROR! Enter a value between 0 and 3.\n");
    }
    if (reg2 == 0)
    {
        imu_read_trans(spi, reg1, &id1);
        printf("0x%02X\n", id1);
        *value = id1;
    }
    else
    {
        imu_read_trans(spi, reg1, &id1);
        imu_read_trans(spi, reg2, &id2);
        *value = (id1 << 8) | id2;
        printf("LCD ID: 0x%04X\n", *value);
    }
}

void imu_write(spi_device_handle_t spi, uint8_t write_reg, uint8_t write_data, int user)
{
    switch(user)
    {
        case 0:
            imu_write_trans(spi, REG_BANK_SEL, BANK0);
            break;
        case 1:
            imu_write_trans(spi, REG_BANK_SEL, BANK1);
            break;
        case 2:
            imu_write_trans(spi, REG_BANK_SEL, BANK2);
            break;
        case 3:
            imu_write_trans(spi, REG_BANK_SEL, BANK3);
            break;
        default:
            printf("ERROR! Enter a value between 0 and 3.\n");
    }
    imu_write_trans(spi, write_reg, write_data);
    uint8_t read_data;
    imu_read_trans(spi,write_reg, &read_data);
    printf("read data from register 0x%02X: 0x%02X\n", write_reg, read_data);
}

void get_accel(spi_device_handle_t spi)
{
    uint16_t value;
    double decimal;
    //printf("X Accel bit value");
    imu_read(spi, ACCEL_XOUT_H, ACCEL_XOUT_L, 0, &value);
    decimal = value / 16384.0;
    printf("X Accel decimal value %2.2f\n",decimal);
    //printf("Y Accel bit value:");
    //imu_read(spi, ACCEL_YOUT_H, ACCEL_YOUT_L, 0, &value);
    //decimal = value / 16384.0;
    //printf("Y Accel decimal value %2.2f\nZ Accel bit value",decimal);
    //imu_read(spi, ACCEL_YOUT_H, ACCEL_YOUT_L, 0, &value);
    //decimal = value / 16384.0;
    //printf("Z Accel decimal value %2.2f\n", decimal);
}

void get_gyro(spi_device_handle_t spi)
{
    uint16_t value;
    double decimal;

    printf("X Gyro bit value");
    imu_read(spi, GYRO_XOUT_H, GYRO_XOUT_L, 0, &value);
    decimal = value / 131.0;
    printf("X Gyro decimal value %2.2f\n",decimal);

    printf("Y Gyro bit value");
    imu_read(spi, GYRO_YOUT_H, GYRO_YOUT_L, 0, &value);
    decimal = value / 131.0;
    printf("Y Gyro decimal value %2.2f\n",decimal);

    printf("Z Gyro bit value");
    imu_read(spi, GYRO_ZOUT_H, GYRO_ZOUT_L, 0, &value);
    decimal = value / 131.0;
    printf("Z Gyro decimal value %2.2f\n",decimal);
}

void get_magnetometer(spi_device_handle_t spi)
{
    uint16_t value;
    double decimal;

    printf("X Mag bit value");
    imu_read(spi, MAG_X_H, MAG_X_L, 3, &value);
    decimal = value / 0.15;
    printf("X Mag decimal value %2.2f\n",decimal);

    printf("Y Mag bit value");
    imu_read(spi, MAG_Y_H, MAG_Y_L, 3, &value);
    decimal = value / 0.15;
    printf("Y Mag decimal value %2.2f\n",decimal);

    printf("Z Mag bit value");
    imu_read(spi, MAG_Z_H, MAG_Z_L, 3, &value);
    decimal = value / 0.15;
    printf("Z Gyro decimal value %2.2f\n",decimal);
}

void imu_initialize(spi_device_handle_t spi)
{
    imu_write(spi, PWR_MGMT_1, 0x01, 0);    //initialize the 
    imu_write(spi, PWR_MGMT_2, 0x00, 0);
    imu_write(spi, USER_CTRL, 0xF0, 0);
    imu_write(spi, MAG_CNTL3, 0x01, 3);
    imu_write(spi, MAG_CNTL2, 0x08, 3);
}

int inv_icm20948_firmware_load(struct inv_icm20948 * s, const unsigned char *data_start, unsigned short size_start, unsigned short load_addr)
{ 
    int write_size;
    int result;
    unsigned short memaddr;
    const unsigned char *data;
    unsigned short size;
    unsigned char data_cmp[16];
    int flag = 0;
		
    // Write DMP memory
    data = data_start;
    size = size_start;
    memaddr = load_addr;
    while (size > 0) {
        write_size = min(size, 16);
        if ((memaddr & 0xff) + write_size > 0x100) {
            // Moved across a bank
            write_size = (memaddr & 0xff) + write_size - 0x100;
        }
        result = inv_icm20948_write_mems(s, memaddr, write_size, (unsigned char *)data);
        if (result)  
            return result;
        data += write_size;
        size -= write_size;
        memaddr += write_size;
    }

    // Verify DMP memory

    data = data_start;
    size = size_start;
    memaddr = load_addr;
    while (size > 0) {
        write_size = min(size, 16);
        if ((memaddr & 0xff) + write_size > 0x100) {
            // Moved across a bank
            write_size = (memaddr & 0xff) + write_size - 0x100;
        }
        result = inv_icm20948_read_mems(s, memaddr, write_size, data_cmp);
        if (result)
            flag++; // Error, DMP not written correctly
        if (memcmp(data_cmp, data, write_size))
            return -1;
        data += write_size;
        size -= write_size;
        memaddr += write_size;
    }

#if defined(WIN32)   
    //if(!flag)
      // inv_log("DMP Firmware was updated successfully..\r\n");
#endif

    return 0;
}

int inv_icm20948_write_mems(struct inv_icm20948 * s, unsigned short reg, unsigned int length, const unsigned char *data)
{
    int result=0;
    unsigned int bytesWritten = 0;
    unsigned int thisLen;
    unsigned char lBankSelected;
    unsigned char lStartAddrSelected;
    
    unsigned char power_state = inv_icm20948_get_chip_power_state(s);

    if(!data)
        return -1;
    
    if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
        result = inv_icm20948_set_chip_power_state(s, CHIP_AWAKE, 1);

    result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 0);
            
	result |= inv_set_bank(s, 0);
    
    lBankSelected = (reg >> 8);
	if (lBankSelected != s->lLastBankSelected)
	{
		result |= inv_icm20948_write_reg(s, REG_MEM_BANK_SEL, &lBankSelected, 1);
		if (result)
			return result;
		s->lLastBankSelected = lBankSelected;
	}
    
    while (bytesWritten < length) 
    {
        lStartAddrSelected = (reg & 0xff);
        /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
           Contents are changed after read or write of the selected memory.
           This register must be written prior to each access to initialize the register to the proper starting address.
           The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */
        result |= inv_icm20948_write_reg(s, REG_MEM_START_ADDR, &lStartAddrSelected, 1);
        if (result)
            return result;
        
        thisLen = min(INV_MAX_SERIAL_WRITE, length-bytesWritten);
        
        /* Write data */ 
        result |= inv_icm20948_write_reg(s, REG_MEM_R_W, &data[bytesWritten], thisLen);
        if (result)
            return result;
        
        bytesWritten += thisLen;
        reg += thisLen;
    }

    //Enable LP_EN since we disabled it at begining of this function.
    result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 1);

    return result;
}

int inv_icm20948_read_mems(struct inv_icm20948 * s, unsigned short reg, unsigned int length, unsigned char *data)
{
	int result=0;
	unsigned int bytesWritten = 0;
	unsigned int thisLen;
	unsigned char i, dat[INV_MAX_SERIAL_READ] = {0};
	unsigned char power_state = inv_icm20948_get_chip_power_state(s);
	unsigned char lBankSelected;
	unsigned char lStartAddrSelected;

	if(!data)
		return -1;

	if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
		result = inv_icm20948_set_chip_power_state(s, CHIP_AWAKE, 1);

	if(check_reg_access_lp_disable(s, reg))
		result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 0);

	result |= inv_set_bank(s, 0);

	lBankSelected = (reg >> 8);
	if (lBankSelected != s->lLastBankSelected)
	{
		result |= inv_icm20948_write_reg(s, REG_MEM_BANK_SEL, &lBankSelected, 1);
		if (result)
			return result;
		s->lLastBankSelected = lBankSelected;
	}

	while (bytesWritten < length) 
	{
		lStartAddrSelected = (reg & 0xff);
		/* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
		   Contents are changed after read or write of the selected memory.
		   This register must be written prior to each access to initialize the register to the proper starting address.
		   The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */
		result |= inv_icm20948_write_reg(s, REG_MEM_START_ADDR, &lStartAddrSelected, 1);
		if (result)
			return result;
		
		thisLen = min(INV_MAX_SERIAL_READ, length-bytesWritten);
		/* Write data */
		if(s->base_state.serial_interface == SERIAL_INTERFACE_SPI) {
			result |= inv_icm20948_read_reg(s, REG_MEM_R_W, &dat[bytesWritten], thisLen);
		} else {
			result |= inv_icm20948_read_reg(s, REG_MEM_R_W, &data[bytesWritten], thisLen);
		}
		if (result)
			return result;
		
		bytesWritten += thisLen;
		reg += thisLen;
	}

	if(s->base_state.serial_interface == SERIAL_INTERFACE_SPI) {
		for (i=0; i< length; i++) {
			*data= dat[i];
			 data++;
		}
	}

	//Enable LP_EN if we disabled it at begining of this function.
	if(check_reg_access_lp_disable(s, reg))
		result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 1);

	return result;
}

void app_main(void)
{
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*320*2+8
    };
    spi_device_interface_config_t devcfg={

        .clock_speed_hz=6*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=-1,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

    //Initialize the SPI bus
    ret=spi_bus_initialize(1, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(1, &devcfg, &spi);
    //inv_icm20948_firmware_load(s, image, size, 0x90);
    
    
    ESP_ERROR_CHECK(ret);

    cs_gpio_setting();
    imu_initialize(spi);
    
    //Start reading and writing using the SPI connection
    while(1)
    {
        get_accel(spi);
        get_gyro(spi);
        get_magnetometer(spi);
        vTaskDelay(1);
    }
}
