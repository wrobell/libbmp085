/*
 * bmp085 - BMP085 pressure sensor library
 *
 * Copyright (C) 2013 by Artur Wroblewski <wrobell@pld-linux.org>
 *
 * Also contains code from
 * 
 *      http://www.john.geek.nz/2013/02/update-bosch-bmp085-source-raspberry-pi/
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include "smbus.h" 

#define BSWAP16(v) (v << 8) & 0xFF00 | (res >> 8) & 0xFF

// fixme: remove hardcodings
#define BMP085_I2C_ADDRESS 0x77
#define BMP085_DEV "/dev/i2c-1"

const unsigned char BMP085_OSS = 3;

// BMP085 calibration data
static short int ac1;
static short int ac2; 
static short int ac3; 
static unsigned short int ac4;
static unsigned short int ac5;
static unsigned short int ac6;
static short int b1; 
static short int b2;
static short int mb;
static short int mc;
static short int md;

static int b5; 
static int i2c_fd;

/**
 * Read 16-bit integer from BMP085.
 */
static __s32 read_swap16(int fd, __u8 address) {
    __s32 res = i2c_smbus_read_word_data(fd, address);
    if (res < 0)
        return -1;
    res = BSWAP16(res);
    return res;
}


/**
 * Calibrate BMP085 sensor.
 */
static void calibrate(int fd) {
    ac1 = read_swap16(fd, 0xAA);
    ac2 = read_swap16(fd, 0xAC);
    ac3 = read_swap16(fd, 0xAE);
    ac4 = read_swap16(fd, 0xB0);
    ac5 = read_swap16(fd, 0xB2);
    ac6 = read_swap16(fd, 0xB4);
    b1 = read_swap16(fd, 0xB6);
    b2 = read_swap16(fd, 0xB8);
    mb = read_swap16(fd, 0xBA);
    mc = read_swap16(fd, 0xBC);
    md = read_swap16(fd, 0xBE);
}

/**
 * Initialize and calibrate BMP085 sensor.
 */
int bmp085_init() {
    if ((i2c_fd = open(BMP085_DEV, O_RDWR)) < 0)
        return -1;
    
    // Set the port options and set the address of the device
    if (ioctl(i2c_fd, I2C_SLAVE, BMP085_I2C_ADDRESS) < 0)
        return -1;

    calibrate(i2c_fd);
}

/**
 * Read pressure from BMP085 sensor.
 *
 * The unit of returned value is Pascal.
 */
unsigned int bmp085_read_pressure() {
    int x1, x2, x3, b3, b6, p;
    unsigned int b4, b7;
    unsigned int up = 0;
    __u8 values[3];

    // request pressure reading (uncalibrated)
    i2c_smbus_write_byte_data(i2c_fd, 0xF4, 0x34 + (BMP085_OSS << 6));

    // wait for conversion, delay time dependent on oversampling setting
    usleep((2 + (3 << BMP085_OSS)) * 1000);
    i2c_smbus_read_i2c_block_data(i2c_fd, 0xF6, 3, values);
    up = (((unsigned int) values[0] << 16) | ((unsigned int) values[1] << 8) | (unsigned int) values[2]) >> (8 - BMP085_OSS);

    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6) >> 12) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int)ac1) * 4 + x3) << BMP085_OSS) + 2) >> 2;
  
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned int)(x3 + 32768)) >> 15;
  
    b7 = (unsigned int)(up - b3) * (50000 >> BMP085_OSS);
    if (b7 < 0x80000000)
        p = (b7 << 1) / b4;
    else
        p = (b7 / b4) << 1;
    
    x1 = (p>>8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;
  
    return p;
}

/**
 * Read temperature from BMP085 sensor.
 *
 * The unit of returned value dC.
 */
unsigned int bmp085_read_temp() {
    int ut = 0;
    int x1, x2;

    // read temperature
    i2c_smbus_write_byte_data(i2c_fd, 0xF4, 0x2E);
    usleep(4500);
    ut = read_swap16(i2c_fd, 0xF6);
  
    x1 = ((ut - (int)ac6) * (int)ac5) >> 15;
    x2 = ((int)mc << 11) / (x1 + md);
    b5 = x1 + x2;

    return (b5 + 8) >> 4;
}

/*
 * vim: sw=4:et:ai
 */
