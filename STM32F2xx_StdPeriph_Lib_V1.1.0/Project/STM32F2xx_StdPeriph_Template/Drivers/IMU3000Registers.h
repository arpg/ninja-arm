//-----------------------------------------------------------------------------
// IMU3000Registers.h
//
// This file contains the register definitions for the IMU3000 sensor used in
// the IMU3000Driver class
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
// Address of IMU3000 registers
//-----------------------------------------------------------------------------
#define IMU3000_WHO_AM_I	0x00
#define IMU3000_X_OFFS_USRH			0x0C
#define IMU3000_X_OFFS_USRL			0x0D
#define IMU3000_Y_OFFS_USRH			0x0E
#define IMU3000_Y_OFFS_USRL			0x0F
#define IMU3000_Z_OFFS_USRH			0x10
#define IMU3000_Z_OFFS_USRL			0x11
#define IMU3000_FIFO_EN				0x12
#define IMU3000_AUX_VDDIO			0x13
#define IMU3000_AUX_SLV_ADDR		0x14
#define IMU3000_SMPLRT_DIV			0x15
#define IMU3000_DLPF_FS				0x16
#define IMU3000_INT_CFG				0x17
#define IMU3000_AUX_BURST_AD_DR		0x18
#define IMU3000_INT_STATUS			0x1A
#define IMU3000_TEMP_OUT_H			0x1B
#define IMU3000_TEMP_OUT_L			0x1C
#define IMU3000_GYRO_XOUT_H			0x1D
#define IMU3000_GYRO_XOUT_L			0x1E
#define IMU3000_GYRO_YOUT_H			0x1F
#define IMU3000_GYRO_YOUT_L			0x20
#define IMU3000_GYRO_ZOUT_H			0x21
#define IMU3000_GYRO_ZOUT_L			0x22
#define IMU3000_AUX_XOUT_H			0x23
#define IMU3000_AUX_XOUT_L			0x24
#define IMU3000_AUX_YOUT_H			0x25
#define IMU3000_AUX_YOUT_L			0x26
#define IMU3000_AUX_ZOUT_H			0x27
#define IMU3000_AUX_ZOUT_L			0x28
#define IMU3000_DMP_REG1			0x35
#define IMU3000_DMP_REG2			0x36
#define IMU3000_DMP_REG3			0x37
#define IMU3000_DMP_REG4			0x38
#define IMU3000_DMP_REG5			0x39
#define IMU3000_FIFO_COUNTH			0x3A
#define IMU3000_FIFO_COUNTL			0x3B
#define IMU3000_FIFO_R				0x3C
#define IMU3000_USER_CTRL			0x3D
#define IMU3000_PWR_MGM				0x3E

//-----------------------------------------------------------------------------
// FIFO_EN Values
//-----------------------------------------------------------------------------
#define IMU3000_FIFO_FOOTER			(0x01 << 0)
#define IMU3000_AUX_ZOUT			(0x01 << 1)
#define IMU3000_AUX_YOUT			(0x01 << 2)
#define IMU3000_AUX_XOUT			(0x01 << 3)
#define IMU3000_GYRO_ZOUT			(0x01 << 4)
#define IMU3000_GYRO_YOUT			(0x01 << 5)
#define IMU3000_GYRO_XOUT			(0x01 << 6)
#define IMU3000_TEMP_OUT			(0x01 << 7)

//-----------------------------------------------------------------------------
// DLPF Values
//-----------------------------------------------------------------------------
#define IMU3000_256HZ_8KHZ			(0x00 << 0)	//LP Filter :256Hz Sample Rate:8Khz
#define IMU3000_188HZ_1KHZ			(0x01 << 0)	//LP Filter :188Hz Sample Rate:1Khz
#define IMU3000_98HZ_1KHZ			(0x02 << 0)	//LP Filter :98Hz Sample Rate:1Khz
#define IMU3000_42HZ_1KHZ			(0x03 << 0)	//LP Filter :42Hz Sample Rate:1Khz
#define IMU3000_20HZ_1KHZ			(0x04 << 0)	//LP Filter :20Hz Sample Rate:1Khz
#define IMU3000_10HZ_1KHZ			(0x05 << 0)	//LP Filter :10Hz Sample Rate:1Khz
#define IMU3000_5HZ_1KHZ			(0x06 << 0)	//LP Filter :5Hz Sample Rate:1Khz

#define IMU3000_FS_250				(0x00 << 3)	//Full scale 250 degrees / sec
#define IMU3000_FS_500				(0x01 << 3)	//Full scale 500 degrees / sec
#define IMU3000_FS_1000				(0x02 << 3)	//Full scale 1000 degrees / sec
#define IMU3000_FS_2000				(0x03 << 3)	//Full scale 2000 degrees / sec

//-----------------------------------------------------------------------------
// USER_CTRL Values
//-----------------------------------------------------------------------------
#define IMU3000_GYRO_RST			(0x01 << 0)
#define IMU3000_FIFO_RST			(0x01 << 1)
#define IMU3000_DMP_RST				(0x01 << 2)
#define IMU3000_AUX_IF_RST			(0x01 << 3)
#define IMU3000_AUX_IF_EN			(0x01 << 5)
#define IMU3000_CTRL_FIFO_EN		(0x01 << 6)
#define IMU3000_DMP_EN				(0x01 << 7)

//-----------------------------------------------------------------------------
// PWR_MGM Values
//-----------------------------------------------------------------------------
#define IMU3000_CLK_INT				(0x00 << 0)	//Internal oscillator
#define IMU3000_CLK_XGYRO			(0x01 << 0) //PLL with X Gyro reference
#define IMU3000_CLK_YGYRO			(0x02 << 0)	//PLL with Y Gyro reference
#define IMU3000_CLK_ZGYRO			(0x03 << 0)	//PLL with Z Gyro reference
#define IMU3000_CLK_EXT32			(0x04 << 0)	//PLL with external 32.768kHz reference
#define IMU3000_CLK_EXT19			(0x05 << 0)	//PLL with external 19.2MHz reference
#define IMU3000_CLK_STOP			(0x07 << 0)	//Stop clock and synchronous reset clock state

#define IMU3000_STBY_ZG				(0x01 << 3)
#define IMU3000_STBY_YG				(0x01 << 4)
#define IMU3000_STBY_XG				(0x01 << 5)
#define IMU3000_SLEEP				(0x01 << 6)
#define IMU3000_H_RESET				(0x01 << 7)


#define IMU3000_AUX_YOUT			(0x01 << 2)
#define IMU3000_AUX_XOUT			(0x01 << 3)
#define IMU3000_GYRO_ZOUT			(0x01 << 4)
#define IMU3000_GYRO_YOUT			(0x01 << 5)
#define IMU3000_GYRO_XOUT			(0x01 << 6)
#define IMU3000_TEMP_OUT			(0x01 << 7)

//-----------------------------------------------------------------------------
// INT_STATUS Values
//-----------------------------------------------------------------------------
#define IMU3000_RAW_DATA_RDY		(0x01 << 0)
#define IMU3000_DMP_DONE			(0x01 << 1)
#define IMU3000_IMU_RDY				(0x01 << 2)
#define IMU3000_I2C_MST_ERR			(0x01 << 3)
#define IMU3000_FIFO_FULL			(0x01 << 7)

//-----------------------------------------------------------------------------
// IMU3000_INT_CFG Values
//-----------------------------------------------------------------------------
#define IMU3000_RAW_RDY_EN			(0x01 << 0)
#define IMU3000_DMP_DONE_EN			(0x01 << 1)
#define IMU3000_IMU_RDY_EN			(0x01 << 2)
#define IMU3000_I2C_MST_ERR_EN		(0x01 << 3)
#define IMU3000_INT_ANYRD_2CLEAR	(0x01 << 4)
#define IMU3000_LATCH_INT_EN		(0x01 << 5)
#define IMU3000_OPEN				(0x01 << 6)
#define IMU3000_ACTL				(0x01 << 7)

//-----------------------------------------------------------------------------
// IMU3000_DLPF Values
//-----------------------------------------------------------------------------
#define IMU3000_FS_SEL_250			(0x00 << 4)
#define IMU3000_FS_SEL_500			(0x01 << 4)
#define IMU3000_FS_SEL_1000			(0x02 << 4)
#define IMU3000_FS_SEL_2000			(0x03 << 4)


