//-----------------------------------------------------------------------------
// LSM303Registers.h
//
// This file contains the register definitions for the LSM303DLH sensor used in
// the LSM303Driver class
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Address of IMU3000 registers
//-----------------------------------------------------------------------------
#define LSM303_CTRL_REG1_A			0x20
#define LSM303_CTRL_REG2_A			0x21
#define LSM303_CTRL_REG3_A			0x22
#define LSM303_CTRL_REG4_A			0x23
#define LSM303_CTRL_REG5_A			0x24
#define LSM303_HP_FILTER_RESET_A	0x25
#define LSM303_REFERENCE_A			0x26
#define LSM303_STATUS_REG_A			0x27
#define LSM303_OUT_X_L_A			0x28
#define LSM303_OUT_X_H_A			0x29
#define LSM303_OUT_Y_L_A			0x2A
#define LSM303_OUT_Y_H_A			0x2B
#define LSM303_OUT_Z_L_A			0x2C
#define LSM303_OUT_Z_H_A			0x2D
#define LSM303_INT1_CFG_A			0x30
#define LSM303_INT1_SOURCE_A		0x31
#define LSM303_INT1_THS_A			0x32
#define LSM303_INT1_DURATION_A		0x33
#define LSM303_INT2_CFG_A			0x34
#define LSM303_INT2_SOURCE_A		0x35
#define LSM303_INT2_THS_A			0x36
#define LSM303_INT2_DURATION_A		0x37
#define LSM303_CRA_REG_M			0x00 
#define LSM303_CRB_REG_M			0x01 
#define LSM303_MR_REG_M				0x02 
#define LSM303_OUT_X_H_M			0x03 
#define LSM303_OUT_X_L_M			0x04 
#define LSM303_OUT_Y_H_M			0x05 
#define LSM303_OUT_Y_L_M			0x06 
#define LSM303_OUT_Z_H_M			0x07 
#define LSM303_OUT_Z_L_M			0x08 
#define LSM303_SR_REG_Mg			0x09 
#define LSM303_IRA_REG_M			0x0A 
#define LSM303_IRB_REG_M			0x0B 
#define LSM303_IRC_REG_M			0x0C

//-----------------------------------------------------------------------------
// LSM303_CTRL_REG1_A Values
//-----------------------------------------------------------------------------
#define LSM303_A_ZEN					(0x01 << 0)
#define LSM303_A_YEN					(0x01 << 1)
#define LSM303_A_XEN					(0x01 << 2)

#define LSM303_A_ODR50_LP37				(0x00 << 3)
#define LSM303_A_ODR100_LP74			(0x01 << 3)
#define LSM303_A_ODR400_LP292			(0x02 << 3)
#define LSM303_A_ODR1000_LP780			(0x02 << 3)

#define LSM303_A_POWER_DOWN_MODE		(0x00 << 5)
#define LSM303_A_POWER_NORMAL_MODE		(0x01 << 5)
#define LSM303_A_LOW_POWER_0_5HZ		(0x02 << 5)
#define LSM303_A_LOW_POWER_1HZ			(0x03 << 5)
#define LSM303_A_LOW_POWER_2HZ			(0x04 << 5)
#define LSM303_A_LOW_POWER_5HZ			(0x05 << 5)
#define LSM303_A_LOW_POWER_10HZ			(0x06 << 5)

//-----------------------------------------------------------------------------
// LSM303_CTRL_REG2_A Values
//-----------------------------------------------------------------------------
#define LSM303_A_BOOT					(0x01 << 7)

//-----------------------------------------------------------------------------
// LSM303_CTRL_REG3_A Values
//-----------------------------------------------------------------------------
#define LSM303_A_I1CFG_I1SRC			(0x00 << 0)
#define LSM303_A_I1CFG_I1SRCORI2SRC		(0x01 << 0)
#define LSM303_A_I1CFG_DATA_READY		(0x02 << 0)
#define LSM303_A_I1CFG_BOOT_RUNNING		(0x03 << 0)

#define LSM303_A_LIR2					(0x01 << 2)

#define LSM303_A_I2CFG_I1SRC			(0x00 << 3)
#define LSM303_A_I2CFG_I1SRCORI2SRC		(0x01 << 3)
#define LSM303_A_I2CFG_DATA_READY		(0x02 << 3)
#define LSM303_A_I2CFG_BOOT_RUNNING		(0x03 << 3)

#define LSM303_A_LIR2					(0x01 << 5)
#define LSM303_A_PPOD					(0x01 << 6)
#define LSM303_A_IHL					(0x01 << 7)

//-----------------------------------------------------------------------------
// LSM303_CTRL_REG4_A Values
//-----------------------------------------------------------------------------
#define LSM303_A_ST						(0x01 << 1)
#define LSM303_A_STSIGN					(0x01 << 3)

#define LSM303_A_FS_2G					(0x00 << 4)
#define LSM303_A_FS_4G					(0x01 << 4)
#define LSM303_A_FS_8G					(0x03 << 4)

#define LSM303_A_BLE					(0x01 << 6)
#define LSM303_A_BDU					(0x01 << 7)

//-----------------------------------------------------------------------------
// LSM303_CRA_REG_M Values
//-----------------------------------------------------------------------------
#define LSM303_M_NORMAL_MEASUREMENT		(0x00 << 0)
#define LSM303_M_POSITIVE_BIAS			(0x01 << 0)
#define LSM303_M_NEGATIVE_BIAS			(0x02 << 0)

#define LSM303_M_0_75HZ					(0x00 << 2)
#define LSM303_M_1_5HZ					(0x01 << 2) 
#define LSM303_M_3HZ					(0x02 << 2) 
#define LSM303_M_7_5HZ					(0x03 << 2) 
#define LSM303_M_15HZ					(0x04 << 2) 
#define LSM303_M_30HZ					(0x05 << 2) 
#define LSM303_M_75HZ					(0x06 << 2) 

//-----------------------------------------------------------------------------
// LSM303_CRB_REG_M Values
//-----------------------------------------------------------------------------
#define LSM303_M_1_3GAUSS				(0x01 << 5) 
#define LSM303_M_1_9GAUSS				(0x02 << 5) 
#define LSM303_M_2_5GAUSS				(0x03 << 5) 
#define LSM303_M_4GAUSS					(0x04 << 5) 
#define LSM303_M_4_7GAUSS				(0x05 << 5) 
#define LSM303_M_5_6GAUSS				(0x06 << 5) 
#define LSM303_M_8_1GAUSS				(0x07 << 5) 
		


//-----------------------------------------------------------------------------
// LSM303_MR_REG_M Values
//-----------------------------------------------------------------------------
#define LSM303_M_CONTINUOUS_CONVERSION	(0x00 << 0) 
#define LSM303_M_SINGLE_CONVERSION		(0x01 << 0) 
#define LSM303_M_SLEEP					(0x03 << 0) 