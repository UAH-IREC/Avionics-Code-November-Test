/*

Write stuff here

*/
//#define SHUTUP

#define HEADING_OFFSET -75
#define TARGET_LATITUDE 34.749428
#define TARGET_LONGITUDE -86.655809

#define FIXMATH_NO_CACHE 1

#include <asf.h>
#include <asf/common/utils/interrupt.h>
#include <math.h>
#include <string.h>
#include <util/atomic.h>
#include "drivers/gpio.h"
#include "drivers/imu.h"
#include "drivers/motor_control.h"
#include "drivers/PID.h"
#include "drivers/GPS.h"
#include "interrupts/GPS_Interrupts.h"
#include "tools/RingBuffer.h"
#include "drivers/servo.h"
#include "drivers/Altitude.h"
#include "drivers/MS5607.h"
#include "drivers/bno055.h"
#include "drivers/apbmand.h"
#include "libfixmatrix/fixquat.h"

static void initialize(void);

bool ready = true; // Change this back to false

uint32_t cycles = 0;

uint16_t packetlen(const uint8_t* buff);

uint16_t packetlen(const uint8_t* buff)
{
	uint16_t i = 0;
	for (i = 0; i < 1024; i++)
	{
		if (buff[i] == '\n')
		return i;
	}
	return 1024;
}
uint32_t time_ms;

ISR(TCE0_OVF_vect) //The actual interrupt handling function
{
	time_ms++;
}

MS5607_t MS5607 =
{
	.select_pin = IOPORT_CREATE_PIN(PORTC,4)
};

apbmand_t apbmand = 
{
	.twi = &TWID
};

mf16 Pk, Hk, Rk;
fix16_t sigma_pitot_measurements, sigma_accel_measurements;

#define	I2C_BUFFER_LEN 8
#define I2C0 5
#define	BNO055_I2C_BUS_WRITE_ARRAY_INDEX	((u8)1)

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The API is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
// 	u8 stringpos = BNO055_INIT_VALUE;


// 	array[BNO055_INIT_VALUE] = reg_addr;
// 	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
// 	{
// 		array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] =
// 			*(reg_data + stringpos);
// 	}

	if(cnt > 1)
	{
		printf("The BNO055 Actually does write more than one byte at a time, isn't that surprising.\nI guess it's time to fix the I2C write hack then.");
	}

	cnt = cnt + 1;	// BNO055 Discards the first write, so we make the first value 0
	array[0] = 0;
	array[1] = reg_data[0]; // This breaks if it ever sends more than one byte at a time, but I don't think it does;

	/*
	* Please take the below APIs as your reference for
	* write the data using I2C communication
	* "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write APIs here
	* BNO055_iERROR is an return value of I2C read API
	* Please select your valid return value
	* In the driver BNO055_SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/

	

	twi_package_t readbno055;
	readbno055.addr[0]	   = reg_addr-1; // it's minus one because the BNO055 discards the first write, and the second write one is at the next address 
	readbno055.addr_length = 1;
	readbno055.chip        = dev_addr;
	readbno055.buffer      = array;
	readbno055.length      = cnt;
	readbno055.no_wait     = false;


	BNO055_iERROR = (s8)twi_master_write(&TWID,&readbno055);

// 	printf("I2C Write cnt=%u status=%i Data:  ",cnt,BNO055_iERROR);
// 	for (u8 i = 0; i < cnt; i++)
// 	{
// 		printf(" %x, ",array[i]);
// 	}
// 	printf("\n");

	return (s8)BNO055_iERROR;
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *  will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BNO055_INIT_VALUE};
// 	u8 stringpos = BNO055_INIT_VALUE;
// 
// 	array[BNO055_INIT_VALUE] = reg_addr;

	/* Please take the below API as your reference
	 * for read the data using I2C communication
	 * add your I2C read API here.
	 * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
	 * ARRAY, ARRAY, 1, CNT)"
	 * BNO055_iERROR is an return value of SPI write API
	 * Please select your valid return value
     * In the driver BNO055_SUCCESS defined as 0
     * and FAILURE defined as -1
	 */

	twi_package_t readbno055;
	readbno055.addr[0]	   = reg_addr;
	readbno055.addr_length = 1;
	readbno055.chip        = dev_addr;
	readbno055.buffer      = array;
	readbno055.length      = cnt;
	readbno055.no_wait     = false;

	BNO055_iERROR = (int8_t) twi_master_read(&TWID, &readbno055);
	memcpy(reg_data, array, cnt);
	
/*	printf("I2C Read: %x\n",array[0]);*/

// 	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
// 		*(reg_data + stringpos) = array[stringpos];
	return (s8)BNO055_iERROR;
}

void BNO055_delay_msek(u32 msek)
{
	delay_ms(msek);
}

struct bno055_t bno055; 

static void initialize()
{
	board_init();
	sysclk_init();
	
	sysclk_enable_peripheral_clock(&TCE0);
	sysclk_enable_module(SYSCLK_PORT_E, SYSCLK_HIRES);

	wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_1KCLK);
	//wdt_enable();
	
	sysclk_enable_peripheral_clock(&USARTC0);
	UART_Comms_Init();
	printf("\nRESTART!\n");

	printf("USART Initialized\n");

	pmic_init();
	irq_initialize_vectors();
	cpu_irq_enable();
	
	printf("PMIC Initialized\n");
	
	time_ms = 0;
	TCE0.CTRLA = 0b00000110;
	TCE0.PER = 121;
	TCE0.INTCTRLA = TC_OVFINTLVL_HI_gc;

	wdt_reset();

	init_GPS_pins_and_usart();
	init_gps_buffers();
	init_gps_interrupts();

	printf("GPS Initialized\n");

	printf("Starting BNO055 Init\n");
	twi_options_t m_options = {
		.speed = 400000,
		.speed_reg = TWI_BAUD(32000000, 400000),
	};
	
	sysclk_enable_peripheral_clock(&TWID);
	twi_master_init(&TWID, &m_options);
	twi_master_enable(&TWID);

	bno055.bus_write = BNO055_I2C_bus_write;
	bno055.bus_read = BNO055_I2C_bus_read;
	bno055.delay_msec = BNO055_delay_msek;
	bno055.dev_addr = BNO055_I2C_ADDR2;

	
	int8_t initStatus = bno055_init(&bno055);
	printf("Init Status: %i  (0 is good)\n", initStatus);
	bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	//bno055_set_clk_src(0x01);
	int8_t status = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	uint8_t operation_mode = 0;
	bno055_get_operation_mode(&operation_mode);
	printf("Operation mode is %u, should be %u\nWrite status: %i  (0 is good)\n",operation_mode,BNO055_OPERATION_MODE_NDOF,status);\
	
	printf("IMU Initialized\n");

	apbmand_data_t apbmand_data = read_apbmand(apbmand);
	printf("%i",(int16_t)(apbmand_data.airspeed));

	printf("Differential Pressure Sensor Initialized\n");

	sysclk_enable_peripheral_clock(&SPIC);
	initializespi(&SPIC,&PORTC);
	calibratePressureSensor(&MS5607);
	
	printf("MS5607 Initialized\n");

	printf("\nInitialization Complete!\n\n\n");
	
	//Setup Kalman matrices
	sigma_pitot_measurements = F16(1.0); //This is actually a function of velocity I guess, we should fix that at some point
	sigma_accel_measurements = F16(1.0);
	
	Pk.rows = 2; Pk.columns = 2;
	Hk = Pk; //Set row and column values
	mf16_fill_diagonal(&Hk, F16(1));
	Rk = Hk;
	mf16_fill(&Pk, F16(0));
	Pk.data[0][0] = fix16_sq(sigma_pitot_measurements);
	Pk.data[1][1] = fix16_sq(sigma_accel_measurements);
	
	delay_ms(3000);

	// calibrate (REMOVE FOR FLIGHT)
//  	wdt_disable();
//  	imu_calibrate_mag();
//  	delay_s(10);

}


static void update_sensors()
{
	//imu_data = imu_update(cycles);

}

typedef struct  
{
	//All values in earth frame
	v3d pos; // m
	v3d vel; // m/s
	fix16_t accel; // m/s^2 
	qf16 orientation; //Quaternion from rocket body frame to earth frame
} state_t;

fix16_t det_2x2(mf16* matrix)
{
	return fix16_sub(fix16_mul(matrix->data[0][0], matrix->data[1][1]), fix16_mul(matrix->data[0][1], matrix->data[1][0]));
}


state_t update_kalman_state(const state_t* last_state, v3d accel, v3d vel, qf16 orientation, fix16_t tstep)
{
	//Acceleration and velocity should both be in the rocket body frame already
	//Orientation should be the new vector from the rocket body frame to the earth frame
	//Only filtering the measurements along the rocket's axis, then transforming those into the global frame and integrating them for a new position
	
	//Get reverse quaternion
	qf16_normalize(&orientation, &orientation);
	qf16 earth_to_body;
	qf16_conj(&earth_to_body, &orientation);
	
	//Miscellaneous useful quantities
	//Last velocity and acceleration in the body frame
	v3d last_vel, last_accel;
	qf16_rotate(&last_accel, &earth_to_body, &(last_state->accel));
	qf16_rotate(&last_vel, &earth_to_body, &(last_state->vel));
	
	//Prediction step
	mf16 Fk;
	Fk.columns = 2; Fk.rows = 2;
	Fk.data[0][0] = F16(1);
	Fk.data[0][1] = tstep;
	Fk.data[1][0] = F16(0);
	Fk.data[1][1] = F16(1);
	
	mf16 xk;
	xk.columns = 1; xk.rows = 2;
	xk.data[0][0] = last_vel.x;
	xk.data[1][0] = last_accel.x;
	mf16 xkhat;
	xkhat.columns = 1; xkhat.rows = 2;
	mf16_mul(&xkhat, &Fk, &xk);
	
	mf16 Qk;
	Qk.rows = 2; Qk.columns = 2;
	Qk.data[0][0] = F16(1);
	Qk.data[0][1] = F16(0);
	Qk.data[1][0] = F16(0);
	Qk.data[1][1] = F16(0.1);
	
	mf16_mul(&Pk, &Fk, &Pk);
	mf16_mul_bt(&Pk, &Pk, &Fk);
	mf16_add(&Pk, &Pk, &Qk);
	
	//Update step
	mf16 K, zk; //Kalman gain and measurements
	K.rows = 2; K.columns = 2;
	zk.rows = 2; zk.columns = 1;
	zk.data[0][0] = vel.x;
	zk.data[1][0] = accel.x;
	
	//Compute the Kalman gain
	mf16_mul(&K, &Hk, &Pk);
	mf16_mul_bt(&K, &K, &Hk);
	mf16_add(&K, &K, &Rk);
	
	//Invert K somehow
	//TODO FIXME XXX THIS WON'T WORK find a nice efficient inversion approach and invert K here. Or get around this somehow
	//After inverting
	mf16 temp;
	temp.rows = 2; temp.columns = 2;
	mf16_mul_bt(&temp, &Pk, &Hk);
	mf16_mul(&K, &temp, &K);
	
	//Updated state
	mf16_mul(&temp, &Hk, &xkhat);
	mf16_sub(&temp, &zk, &temp);
	mf16_mul(&temp, &K, &temp);
	mf16_add(&xk, &xkhat, &temp);
	
	//Update Pk
	mf16_mul(&temp, &K, &Hk);
	mf16_mul(&temp, &temp, &Pk);
	mf16_sub(&Pk, &Pk, &temp);
	
	//Integrate
	fix16_t filtered_vel = xk.data[0][0];
	fix16_t filtered_accel = xk.data[1][0];
	//TODO: finish integrating and such
}

uint32_t hrms_to_sec(uint8_t hours, uint8_t minutes, uint8_t seconds);

int main (void)
{
	initialize();
	

	delay_ms(10); //This has to be here, I don't know why.

	wdt_reset();
	
	uint32_t last_time = 0;



	uint8_t gpsbuff[300];
	uint8_t gpsflag = 0, datalen, msgend;
	GPS_data_t GPSdata, tempgpsdata;

	uint8_t gpstmp[85];
	
	
	
	uint8_t got_good_time = 0;
	int32_t gps_local_delta; //local time + this = gps time (ish)


	int32_t initPressure = 0;
	int32_t initTemperature = 0;
	
	readMS5607(&MS5607, &initPressure, &initTemperature);

	
	
	int32_t pressure = 0;
	int32_t temperature = 0;
	int32_t altitude = 0;


	readMS5607(&MS5607, &pressure, &temperature);

	//printf("init pressure: %li   1st real pressure: %li\n",initPressure, pressure);
	printf("Cycles,State,Altitude,Heading,Yaw,Pitch,Roll,ServoEnable,CommandedYaw,CommandedPitch,CurrentLat,CurrentLon,TargetHeading\n");
	v3d position;
	position.x = 0;
	position.y = 0;
	position.z = 0;
	v3d velocity;
	velocity.x = 0;
	velocity.y = 0;
	velocity.z = 0;
	v3d acceleration;
	
	qf16 orientation;

	MS5607.STATE = MS_SENS_STATE_READY;
	uint32_t last_ms_action_time = 0;
	
	apbmand_data_t apbmand_data;
	float nedx, nedy, nedz;
	
	uint16_t last_toggle_time = 0;
	
	uint8_t has_launched = 0;
	uint8_t high_accel_counter = 0;
	while(true)
	{
		//sleepmgr_sleep(SLEEPMGR_STDBY);
		if(/*imu_is_data_ready() ||*/ ready)
		{
			//ready = false;
			wdt_reset();
			cycles++; 

			update_sensors();

			if(MS5607.STATE == MS_SENS_STATE_READY)
			{
				askForPressureMS5607(&MS5607);
				MS5607.STATE = MS_SENS_STATE_ASKING_PRESS;
				last_ms_action_time = time_ms;
			}
			if (MS5607.STATE == MS_SENS_STATE_ASKING_PRESS && time_ms - last_ms_action_time >= MS_SENS_LENGTH_VALUE_READ)
			{
				readRawPressureMS5607(&MS5607);
				askForTemperatureMS5607(&MS5607);
				MS5607.STATE = MS_SENS_STATE_ASKING_TEMP;
				last_ms_action_time = time_ms;
			}
			if(MS5607.STATE == MS_SENS_STATE_ASKING_TEMP && time_ms - last_ms_action_time >= MS_SENS_LENGTH_VALUE_READ)
			{
				readRawTemperatureMS5607(&MS5607);
				calculateValuesMS5607(&MS5607, &pressure, &temperature);
				altitude = Measure_altitude(initPressure,pressure)/100;
				
				MS5607.STATE = MS_SENS_STATE_READY;
				
// 				if(cycles/30 < 220)
// 				{
// 					altitude = cycles/30;
// 				}
// 				else if (cycles/30 < 220 + (400/30))
// 				{
// 					altitude = 220;
// 				}
// 				else if (cycles/30 < 220 + (400/30) + 220)
// 				{
// 					altitude = 440 - (cycles-(220 + (400/30) + 220))/30;
// 				}
// 				else
// 				{
// 					altitude = 0;
// 				}
			}
				
			apbmand_data = read_apbmand(apbmand);

				
			
			/*
			IMU Data:
				- Raw acceleration
				- Raw gyroscope
				- Raw magnetometer
				- Linear (no gravity vector) acceleration
				- Gravity vector
				- Quaternion
					
			Pressure sensor:
				- Pressure
					
			Calculated values:
				- Airspeed
				- World acceleration
				- World velocity
				- World position
			*/
			struct bno055_linear_accel_t bno055_linear_accel;
			bno055_read_linear_accel_xyz(&bno055_linear_accel);
			
			uint32_t accel_mag = bno055_linear_accel.x * bno055_linear_accel.x + bno055_linear_accel.y * bno055_linear_accel.y + bno055_linear_accel.z * bno055_linear_accel.z;
			if (accel_mag > 10000 * 20)
			{
				high_accel_counter++;
				if (high_accel_counter > 10 && !has_launched)
				{
					velocity.x = F16(0); velocity.y = F16(0); velocity.z = F16(0);
					position.x = F16(0); position.y = F16(0); position.z = F16(0);
					has_launched = 1;
				}
			}
			else
			{
				high_accel_counter = 0;
			}
				
			struct bno055_accel_t raw_accel;
			bno055_read_accel_xyz(&raw_accel);
				
			struct bno055_gyro_t raw_gyro;
			bno055_read_gyro_xyz(&raw_gyro);
				
			struct bno055_mag_t raw_mag;
			bno055_read_mag_xyz(&raw_mag);
				
			acceleration.x = fix16_from_float((float)bno055_linear_accel.x / 100.0);
			acceleration.y = fix16_from_float((float)bno055_linear_accel.y / 100.0);
			acceleration.z = fix16_from_float((float)bno055_linear_accel.z / 100.0);

			struct bno055_quaternion_t bno055_quaternion;
			bno055_read_quaternion_wxyz(&bno055_quaternion);
			orientation.a = fix16_from_int(bno055_quaternion.w);
			orientation.b = fix16_from_int(bno055_quaternion.x);
			orientation.c = fix16_from_int(bno055_quaternion.y);
			orientation.d = fix16_from_int(bno055_quaternion.z);
								
			if(bno055_quaternion.w == 0 && bno055_quaternion.x == 0 && bno055_quaternion.y == 0 && bno055_quaternion.z == 0 && bno055_linear_accel.x == 0 && bno055_linear_accel.y == 0 && bno055_linear_accel.z == 0)
			{
				int8_t initStatus = bno055_init(&bno055);
				printf("Init Status: %i  (0 is good)\n", initStatus);
				bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
				//bno055_set_clk_src(0x01);
				int8_t status = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
				uint8_t operation_mode = 0;
				bno055_get_operation_mode(&operation_mode);
				printf("Operation mode is %u, should be %u\nWrite status: %i  (0 is good)\n",operation_mode,BNO055_OPERATION_MODE_NDOF,status);
					
				printf("IMU Initialized\n");
				delay_ms(500);
			}
				
			uint8_t accel_calib = 0;
			uint8_t gyro_calib = 0;
			uint8_t mag_calib = 0;
			uint8_t sys_calib = 0;
			bno055_get_accel_calib_stat(&accel_calib);
			bno055_get_gyro_calib_stat(&gyro_calib);
			bno055_get_mag_calib_stat(&mag_calib);
			bno055_get_sys_calib_stat(&sys_calib);
				
// 				if (accel_calib > 1 && gyro_calib > 1 && mag_calib > 1 && sys_calib > 1)
// 				{
			qf16_normalize(&orientation, &orientation);
			qf16_conj(&orientation, &orientation); //Inverts quaternion (inverse of unit quaternion is the conjugate)

			v3d globalaccel;
			qf16_rotate(&globalaccel, &orientation, &acceleration);
			uint32_t this_time = time_ms;
			if (last_time != 0)
			{
				v3d dv, dp, tmp;
				v3d_mul_s(&dv, &globalaccel, fix16_from_float((float)(this_time - last_time) / 1000.0));
				v3d_add(&velocity, &velocity, &dv);
				v3d_mul_s(&dp, &velocity, fix16_from_float((float)(this_time - last_time) / 1000.0));
				v3d_add(&position, &position, &dp);
			}
			last_time = this_time;

			v3d pos_cm;
			v3d_mul_s(&pos_cm, &position, F16(100));
			printf("%li, "		//Time
					"%i, %i, %i, "		//Position
					"%i, %i, %i, %i, "	//Quaternion
					"%i, %i, %i, "		//Linear acceleration
					"%i, %i, %i, "		//Raw acceleration
					"%i, %i, %i, "		//Raw gyroscope
					"%i, %i, %i, "		//Raw magnetometer
					"%u, %u, %u, %u, "	//Calibration statuses
					"MS5607: %li, %li, %li, "		//Pressure, temperature, altitude
					"%i, %li"						//Airspeed, dynamic pressure
					"%lu, %f, %f, %f"			//GPS time, latitude, longitude, GPS height
					"\n", 
				time_ms, 
				fix16_to_int(pos_cm.x), fix16_to_int(pos_cm.y), fix16_to_int(pos_cm.z),
				bno055_quaternion.w, bno055_quaternion.x, bno055_quaternion.y, bno055_quaternion.z,
				bno055_linear_accel.x, bno055_linear_accel.y, bno055_linear_accel.z,
				raw_accel.x, raw_accel.y, raw_accel.z,
				raw_gyro.x, raw_gyro.y, raw_gyro.z,
				raw_mag.x, raw_mag.y, raw_mag.z,
				accel_calib, gyro_calib, mag_calib, sys_calib,
				pressure, temperature, altitude,
				(int16_t)(apbmand_data.airspeed * 100), (int32_t)apbmand_data.pressure,
				hrms_to_sec(GPSdata.hour, GPSdata.minutes, GPSdata.seconds), GPSdata.latdecimal, GPSdata.londecimal, GPSdata.altitude
				);
/*				}*/
			if ((accel_calib < 1 || gyro_calib < 1 || mag_calib < 1 || sys_calib < 1) && time_ms - last_toggle_time > 100)
			{
				PORTE.OUTTGL = 0xff;
				last_toggle_time = time_ms;
			}
			else
			{
				PORTE.OUT = 0xf0;
			}

// 			printf("%lu,",cycles);
// 			printf("%li,%lu,%lu,",pressure,temperature);
			//printf("%.3f,%.3f,%.3f,%.3f,",yaw_to_heading(imu_data.yaw),imu_data.yaw,imu_data.roll,imu_data.pitch);
			//printf("%.3f,%i,",yawAngle,pitchAngle);
			//printf("%f,%f\n",GPSdata.latdecimal, GPSdata.londecimal);
			
			
			//HEADING IS (imu_data.yaw+180.0)
			//DANIEL PUT CODE HERE

			//printf("\nImu Data: %.3f %.3f %.3f",(imu_data.yaw + 180.0), imu_data.pitch, imu_data.roll);

			//delay_ms(10);
		}
		if (last_finished != SENTENCE_NONE)
		{
			if (last_finished == SENTENCE_GPGGA)
			{
				//printf("GGA!!!\n");
				ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
				{
					memcpy(gpstmp, gpgga_buff, 85);
				}
// 				uint8_t* gpsstr = "$GPGGA,052430.779,3443.3629,N,08638.2812,W,1,05,1.74,188.1,M,-30.4,M,,*58";
// 				strcpy(gpstmp, gpsstr);
				gpstmp[packetlen(gpstmp)] = '\0';
				//printf(gpstmp);
				//putchar('\n');
				
				//usart_serial_write_packet(&XBEE_USART, gpstmp, 85);
				tempgpsdata = getGPSDatafromNMEA(gpstmp, strlen(gpstmp));
				//printf("test");
				//printf("%c",tempgpsdata->hour[0]);
				//printf("test2\n");
				//if (strlen(tempgpsdata->altitude) > 1)
				//	printf("Altitude: %s\n", tempgpsdata->altitude);
				
				//printf("no of sats: %s\n", tempgpsdata->noofsatellites);
				//printf("Altitude: %u, minutes: %u, valid: %u\n", (uint16_t)tempgpsdata->altitude, tempgpsdata->minutes, tempgpsdata->fix_status ? 1 : 0);
				//GPSdata->altitude = tempgpsdata->altitude;
				GPSdata = tempgpsdata;
				//memcpy(GPSdata->noofsatellites, tempgpsdata->noofsatellites, 3);
				last_finished = SENTENCE_NONE;
				
				if (GPSdata.fix_status)
				{
					uint32_t GPS_secs = 3600 * (uint32_t)GPSdata.hour + 60 * (uint32_t)GPSdata.minutes + (uint32_t)GPSdata.seconds;
					uint32_t safetime;
					ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
					{
						safetime = time_ms;
					}
					gps_local_delta = GPS_secs - safetime;
					got_good_time = 1;
				}
				// 				uint16_t delta;
				// 				ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
				// 				{
				// 					delta = time_ms - begin_parse;
				// 				}
				// 				printf("Difference is %u\n", delta);
			}
		}
	}
}

uint32_t hrms_to_sec(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	return 3600 * (uint32_t)hours + 60 * (uint32_t)minutes + seconds;
}