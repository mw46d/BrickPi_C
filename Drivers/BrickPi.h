/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  
*  Jaikrishna T S
*  t.s.jaikrishna<at>gmail.com
*
*  Updated by John Cole, Dexter Industries.   
*
*  Initial date: June 4, 2013
*  Last updated: June 19, 2014
*
* These files have been made available online through a Creative Commons Attribution-ShareAlike 3.0  license.
* (http://creativecommons.org/licenses/by-sa/3.0/)
*
*  This is a library of functions for the RPi to communicate with the BrickPi.
*/
/*
##################################################################################################################
* Debugging:
* - NOTE THAT DEBUGGING ERROR MESSAGES ARE TURNED OFF BY DEFAULT.  To debug, just take the comment out of Line 38.
* 
* If you #define DEBUG in the program, the BrickPi.h drivers will print debug messages to the terminal. One common message is
* "BrickPiRx error: -2", in function BrickPiUpdateValues(). This is caused by an error in the communication with one of the
* microcontrollers on the BrickPi. When this happens, the drivers automatically re-try the communication several times before the
* function gives up and returns -1 (unsuccessful) to the user-program.

* Function BrickPiUpdateValues() will either return 0 (success), or -1 (error that could not automatically be resolved, even after
* re-trying several times). We have rarely had BrickPiUpdateValues() retry more than once before the communication was successful.
* A known cause for "BrickPiRx error: -2" is the RPi splitting the UART message. Sometimes the RPi will send e.g. 3 bytes, wait a
* while, and then send 4 more, when it should have just sent 7 at once. During the pause between the packs of bytes, the BrickPi
* microcontrollers will think the transmission is complete, realize the message doesn't make sense, throw it away, and not return
* a message to the RPi. The RPi will then fail to receive a message in the specified amount of time, timeout, and then retry the
* communication.
*/

#ifndef __BrickPi_h_
#define __BrickPi_h_

#define DEBUG 

#include <unistd.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#ifdef USE_THREADS
#include <boost/thread.hpp>
#endif

#define DEBUG 
__BEGIN_NAMESPACE_STD
#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3

#define PORT_1 0
#define PORT_2 1
#define PORT_3 2
#define PORT_4 3

#define MASK_D0_M 0x01
#define MASK_D1_M 0x02
#define MASK_9V   0x04
#define MASK_D0_S 0x08
#define MASK_D1_S 0x10

#define BYTE_MSG_TYPE               0 // MSG_TYPE is the first byte.
#define MSG_TYPE_CHANGE_ADDR      1 // Change the UART address.
#define MSG_TYPE_SENSOR_TYPE      2 // Change/set the sensor type.
#define MSG_TYPE_VALUES           3 // Set the motor speed and direction, and return the sesnors and encoders.
#define MSG_TYPE_E_STOP           4 // Float motors immidately
#define MSG_TYPE_TIMEOUT_SETTINGS 5 // Set the timeout
#define MSG_TYPE_MOTOR          'M' 		// New Motor commands

#define MSG_RESULT_MOTOR_OK       'M'
#define MSG_RESULT_MOTOR_NOK      MSG_RESULT_MOTOR_OK + 1
#define MSG_RESULT_MOTOR_I8       MSG_RESULT_MOTOR_OK + 2
#define MSG_RESULT_MOTOR_U8       MSG_RESULT_MOTOR_OK + 3
#define MSG_RESULT_MOTOR_I32      MSG_RESULT_MOTOR_OK + 4
#define MSG_RESULT_MOTOR_U32      MSG_RESULT_MOTOR_OK + 5


  // New UART address (MSG_TYPE_CHANGE_ADDR)
    #define BYTE_NEW_ADDRESS     1
  
  // Sensor setup (MSG_TYPE_SENSOR_TYPE)
    #define BYTE_SENSOR_1_TYPE   1
    #define BYTE_SENSOR_2_TYPE   2
  
  // Timeout setup (MSG_TYPE_TIMEOUT_SETTINGS)
    #define BYTE_TIMEOUT 1

#define TYPE_MOTOR_PWM                 0
#define TYPE_MOTOR_SPEED               1
#define TYPE_MOTOR_POSITION            2

#define TYPE_SENSOR_RAW                0 // - 31
#define TYPE_SENSOR_LIGHT_OFF          0
#define TYPE_SENSOR_LIGHT_ON           (MASK_D0_M | MASK_D0_S)
#define TYPE_SENSOR_TOUCH              32
#define TYPE_SENSOR_ULTRASONIC_CONT    33
#define TYPE_SENSOR_ULTRASONIC_SS      34
#define TYPE_SENSOR_RCX_LIGHT          35 // tested minimally
#define TYPE_SENSOR_COLOR_FULL         36
#define TYPE_SENSOR_COLOR_RED          37
#define TYPE_SENSOR_COLOR_GREEN        38
#define TYPE_SENSOR_COLOR_BLUE         39
#define TYPE_SENSOR_COLOR_NONE         40
#define TYPE_SENSOR_I2C                41
#define TYPE_SENSOR_I2C_9V             42


#define TYPE_SENSOR_EV3_US_M0          43
#define TYPE_SENSOR_EV3_US_M1          44
#define TYPE_SENSOR_EV3_US_M2          45
#define TYPE_SENSOR_EV3_US_M3          46
#define TYPE_SENSOR_EV3_US_M4          47
#define TYPE_SENSOR_EV3_US_M5          48
#define TYPE_SENSOR_EV3_US_M6          49

#define TYPE_SENSOR_EV3_COLOR_M0       50
#define TYPE_SENSOR_EV3_COLOR_M1       51
#define TYPE_SENSOR_EV3_COLOR_M2       52
#define TYPE_SENSOR_EV3_COLOR_M3       53
#define TYPE_SENSOR_EV3_COLOR_M4       54
#define TYPE_SENSOR_EV3_COLOR_M5       55

#define TYPE_SENSOR_EV3_GYRO_M0        56
#define TYPE_SENSOR_EV3_GYRO_M1        57
#define TYPE_SENSOR_EV3_GYRO_M2        58
#define TYPE_SENSOR_EV3_GYRO_M3        59
#define TYPE_SENSOR_EV3_GYRO_M4        60

#define TYPE_SENSOR_EV3_INFRARED_M0    61
#define TYPE_SENSOR_EV3_INFRARED_M1    62
#define TYPE_SENSOR_EV3_INFRARED_M2    63
#define TYPE_SENSOR_EV3_INFRARED_M3    64
#define TYPE_SENSOR_EV3_INFRARED_M4    65
#define TYPE_SENSOR_EV3_INFRARED_M5    66

#define TYPE_SENSOR_EV3_TOUCH_0		   67

#define BIT_I2C_MID  0x01  // Do one of those funny clock pulses between writing and reading. defined for each device.
#define BIT_I2C_SAME 0x02  // The transmit data, and the number of bytes to read and write isn't going to change. defined for each device.

#define INDEX_RED   0
#define INDEX_GREEN 1
#define INDEX_BLUE  2
#define INDEX_BLANK 3

//US Fix starts
# define US_I2C_SPEED 10 //#tweak this value 
# define US_I2C_IDX 0
# define LEGO_US_I2C_ADDR 0x02
# define LEGO_US_I2C_DATA_REG 0x42
//US Fix ends

long gotten_bits = 0;

static int BrickPiTx(unsigned char dest, unsigned char ByteCount, unsigned char OutArray[]);
static int BrickPiRx(unsigned char *InBytes, unsigned char *InArray, long timeout);

struct BrickPiStruct{
  unsigned char Address        [2];     // Communication addresses
  unsigned long Timeout           ;     // Communication timeout (how long in ms since the last valid communication before floating the motors). 0 disables the timeout.

/*
  Motors
*/
  int           MotorSpeed     [4];     // Motor speeds, from -255 to 255
  unsigned char MotorEnable    [4];     // Motor enable/disable

/*
  Encoders
*/
  long          EncoderOffset  [4];     // Encoder offsets (not yet implemented)
  long          Encoder        [4];     // Encoder values

/*
  Sensors
*/
  long          Sensor         [4];     // Primary sensor values
  long          SensorArray    [4][4];  // For more sensor values for the sensor (e.g. for color sensor FULL mode).
  unsigned char SensorType     [4];     // Sensor types
  unsigned char SensorSettings [4][8];  // Sensor settings, used for specifying I2C settings.

/*
  I2C
*/
  unsigned char SensorI2CDevices[4];        // How many I2C devices are on each bus (1 - 8).
  unsigned char SensorI2CSpeed  [4];        // The I2C speed.
  unsigned char SensorI2CAddr   [4][8];     // The I2C address of each device on each bus.  
  unsigned char SensorI2CWrite  [4][8];     // How many bytes to write
  unsigned char SensorI2CRead   [4][8];     // How many bytes to read
  unsigned char SensorI2COut    [4][8][16]; // The I2C bytes to write
  unsigned char SensorI2CIn     [4][8][16]; // The I2C input buffers
};

struct BrickPiStruct BrickPi;

//Store the button's of the MINDSENSORS PSP Controller
struct button
{
	unsigned char   l1;
	unsigned char   l2;
	unsigned char   r1;
	unsigned char   r2;
	unsigned char   a;
	unsigned char   b;
	unsigned char   c;
	unsigned char   d;
	unsigned char   tri;
	unsigned char   sqr;
	unsigned char   cir;
	unsigned char   cro;
	unsigned char   ljb;  // joystick button state
	unsigned char   rjb;  // joystick button state

	int   ljx;   // analog value of joystick scaled from -127 to 127
	int   ljy;   // analog value of joystick scaled from -127 to 127
	int   rjx;   // analog value of joystick scaled from -127 to 127
	int   rjy;   // analog value of joystick scaled from -127 to 127
};
//Initialize all the buttons to 0 of the MINDSENSORS PSP controller
struct button init_psp (struct button b)
{
	b.l1 = 0;
	b.l2 = 0;
	b.r1 = 0;
	b.r2 = 0;
	b.a = 0;
	b.b = 0;
	b.c = 0;
	b.d = 0;
	b.tri = 0;
	b.sqr = 0;
	b.cir = 0;
	b.cro = 0;
	b.ljb = 0;
	b.rjb = 0;
	b.ljx = 0;
	b.ljy = 0;
	b.rjx = 0;
	b.rjy = 0;
	return b;
}

//Update all the buttons of the MINDSENSORS PSP controller
//For all buttons:
//	0:	Unpressed
//	1:	Pressed
//	
//	Left and right joystick: -127 to 127 
struct button upd(struct button b,int port)
{
	//Left and right joystick button press
	b.ljb = ~(BrickPi.SensorI2CIn[port][0][0] >> 1) & 0x01;
	b.rjb = ~(BrickPi.SensorI2CIn[port][0][0] >> 2) & 0x01;
	
	//For buttons a,b,c,d
	b.d = ~(BrickPi.SensorI2CIn[port][0][0] >> 4) & 0x01;
	b.c = ~(BrickPi.SensorI2CIn[port][0][0] >> 5) & 0x01;
	b.b = ~(BrickPi.SensorI2CIn[port][0][0] >> 6) & 0x01;
	b.a = ~(BrickPi.SensorI2CIn[port][0][0] >> 7) & 0x01;
 
	//For buttons l1,l2,r1,r2
	b.l2    = ~(BrickPi.SensorI2CIn[port][0][1] ) & 0x01;
	b.r2    = ~(BrickPi.SensorI2CIn[port][0][1] >> 1) & 0x01;
	b.l1    = ~(BrickPi.SensorI2CIn[port][0][1] >> 2) & 0x01;
	b.r1    = ~(BrickPi.SensorI2CIn[port][0][1] >> 3) & 0x01;
	
	//For buttons square,triangle,cross,circle
	b.tri    = ~(BrickPi.SensorI2CIn[port][0][1] >> 4) & 0x01;
	b.cir    = ~(BrickPi.SensorI2CIn[port][0][1] >> 5) & 0x01;
	b.cro = ~(BrickPi.SensorI2CIn[port][0][1] >> 6) & 0x01;
	b.sqr    = ~(BrickPi.SensorI2CIn[port][0][1] >> 7) & 0x01;
   
   //Left joystick x and y , -127 to 127
	b.ljx = -1*((BrickPi.SensorI2CIn[port][0][5]&0xff) - 128);
	b.ljy = (BrickPi.SensorI2CIn[port][0][4]&0xff) - 128;
	
	//Right joystick x and y , -127 to 127
	b.rjx = -1*((BrickPi.SensorI2CIn[port][0][3]&0xff) - 128);
	b.rjy = (BrickPi.SensorI2CIn[port][0][2]&0xff) - 128;
	return b;
}


//Show button values of the MINDSENSORS PSP controller
void show_val(struct button b)
{
	printf("ljb rjb d c b a l2 r2 l1 r1 tri cir cro sqr\tljx\tljy\trjx\trjy\n");
	printf("%d ",b.ljb);
	printf("  %d ",b.rjb);
	printf("  %d ",b.d);
	printf("%d ",b.c);
	printf("%d ",b.b);
	printf("%d ",b.a);
	         
	printf("%d ",b.l2);
	printf(" %d ",b.r2);
	printf(" %d ",b.l1);
	printf(" %d ",b.r1);
	printf(" %d ",b.tri);
	printf("  %d ",b.cir);
	printf("  %d ",b.cro);
	printf("  %d ",b.sqr);
	printf("\t%d ",b.ljx);
	printf("\t%d ",b.rjx);
	printf("\t%d ",b.ljy);
	printf("\t%d\n\n",b.rjy);
}
unsigned char Array[256];
unsigned char BytesReceived;

int BrickPiChangeAddress(unsigned char OldAddr, unsigned char NewAddr){
  unsigned char i = 0;
  Array[BYTE_MSG_TYPE] = MSG_TYPE_CHANGE_ADDR;
  Array[BYTE_NEW_ADDRESS] = NewAddr;
  BrickPiTx(OldAddr, 2, Array);
    
  if(BrickPiRx(&BytesReceived, Array, 5000))
    return -1;
  if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_CHANGE_ADDR))
    return -1;

  return 0;
}

int BrickPiSetTimeout(){
  unsigned char i = 0;
  while(i < 2){
    Array[BYTE_MSG_TYPE] = MSG_TYPE_TIMEOUT_SETTINGS;
    Array[BYTE_TIMEOUT      ] = ( BrickPi.Timeout             & 0xFF);
    Array[(BYTE_TIMEOUT + 1)] = ((BrickPi.Timeout / 256     ) & 0xFF);
    Array[(BYTE_TIMEOUT + 2)] = ((BrickPi.Timeout / 65536   ) & 0xFF);
    Array[(BYTE_TIMEOUT + 3)] = ((BrickPi.Timeout / 16777216) & 0xFF);
    BrickPiTx(BrickPi.Address[i], 5, Array);
    if(BrickPiRx(&BytesReceived, Array, 2500))
      return -1;
    if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_TIMEOUT_SETTINGS))
      return -1;
    i++;
  }
  return 0;
}

#define MOTOR_PORT_A	0x01
#define MOTOR_PORT_B	0x02
#define MOTOR_PORT_C	0x04
#define MOTOR_PORT_D	0x08

#define MOTOR_CONTROL_SPEED      0x01
#define MOTOR_CONTROL_RAMP       0x02
#define MOTOR_CONTROL_RELATIVE   0x04
#define MOTOR_CONTROL_TACHO      0x08
#define MOTOR_CONTROL_BRK        0x10
#define MOTOR_CONTROL_ON         0x20
#define MOTOR_CONTROL_TIME       0x40
#define MOTOR_CONTROL_GO         0x80

#define MOTOR_STATUS_SPEED       0x01
#define MOTOR_STATUS_RAMP        0x02                   // XXX Not used yet
#define MOTOR_STATUS_MOVING      0x04
#define MOTOR_STATUS_TACHO       0x08
#define MOTOR_STATUS_BRK         0x10
#define MOTOR_STATUS_OVERLOAD    0x20                   // XXX not used yet
#define MOTOR_STATUS_TIME        0x40
#define MOTOR_STATUS_STALL       0x80

#define MOTOR_DIRECTION_FORWARD  0x00
#define MOTOR_DIRECTION_BACKWARD 0x01

#define MOTOR_RUN_ABSOLUTE       0x00
#define MOTOR_RUN_RELATIVE       0x01

#define MOTOR_NEXT_ACTION_FLOAT     0x00
#define MOTOR_NEXT_ACTION_BRAKE     0x01
#define MOTOR_NEXT_ACTION_BRAKEHOLD 0x02

#define MOTOR_CONTINUE              0x00
#define MOTOR_WAIT_TO_COMPLETE      0x01

#ifdef USE_THREADS
boost::mutex motorMtx;

#define motorMtx_lock()		motorMtx.lock()
#define motorMtx_unlock()	motorMtx.unlock()
#else
#define motorMtx_lock()		;
#define motorMtx_unlock()	;
#endif 

// Encoder does 720 ticks/rotation!
int motorSetEncoderTarget(unsigned char which_motors, signed int encoder) {
  int i;
  motorMtx_lock();
  
  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 's';
      Array[BYTE_MSG_TYPE + 2] = 'E';
      Array[BYTE_MSG_TYPE + 3] = wm;
      Array[BYTE_MSG_TYPE + 4] = encoder & 0xff;
      Array[BYTE_MSG_TYPE + 5] = (encoder >> 8) & 0xff;
      Array[BYTE_MSG_TYPE + 6] = (encoder >> 16) & 0xff;
      Array[BYTE_MSG_TYPE + 7] = (encoder >> 24) & 0xff;
      BrickPiTx(BrickPi.Address[i], 8, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived > 1 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_OK) {
        motorMtx_unlock();
        return -1;
      }
    }
  }
  motorMtx_unlock();

  return 0;
}

int motorGetEncoderTarget(unsigned char which_motors) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      int ret = 0;
      int k;

      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 'g';
      Array[BYTE_MSG_TYPE + 2] = 'E';
      Array[BYTE_MSG_TYPE + 3] = wm;
      BrickPiTx(BrickPi.Address[i], 4, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived != 5 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_I32) {
        motorMtx_unlock();
        return -1;
      }

      for (k = 4; k > 0; k--) {
        ret = (ret << 8) | Array[k];
      }

      motorMtx_unlock();
      return ret;
    }
  }

  motorMtx_unlock();
  return 0;
}

// The speed in -100 .. 100 !! That's different from the original range!
int motorSetSpeed(unsigned char which_motors, signed char speed) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 's';
      Array[BYTE_MSG_TYPE + 2] = 'S';
      Array[BYTE_MSG_TYPE + 3] = wm;
      Array[BYTE_MSG_TYPE + 4] = speed;
      BrickPiTx(BrickPi.Address[i], 5, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived > 1 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_OK) {
        motorMtx_unlock();
        return -1;
      }
    }
  }

  motorMtx_unlock();
  return 0;
}

signed char motorGetSpeed(unsigned char which_motors) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 'g';
      Array[BYTE_MSG_TYPE + 2] = 'S';
      Array[BYTE_MSG_TYPE + 3] = wm;
      BrickPiTx(BrickPi.Address[i], 4, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived != 2 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_I8) {
        motorMtx_unlock();
        return -1;
      }

      motorMtx_unlock();
      return (signed char)(Array[1]);
    }
  }

  motorMtx_unlock();
  return 0;
}

// Time in milli seconds but the underlaying resolution might only support > x ms!!
int motorSetTimeToRun(unsigned char which_motors, unsigned int time) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 's';
      Array[BYTE_MSG_TYPE + 2] = 'T';
      Array[BYTE_MSG_TYPE + 3] = wm;
      Array[BYTE_MSG_TYPE + 4] = time & 0xff;
      Array[BYTE_MSG_TYPE + 5] = (time >> 8) & 0xff;
      Array[BYTE_MSG_TYPE + 6] = (time >> 16) & 0xff;
      Array[BYTE_MSG_TYPE + 7] = (time >> 24) & 0xff;
      BrickPiTx(BrickPi.Address[i], 8, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived > 1 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_OK) {
        motorMtx_unlock();
        return -1;
      }
    }
  }

  motorMtx_unlock();
  return 0;
}

unsigned int motorGetTimeToRun(unsigned char which_motors) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      unsigned int ret = 0;
      int k;

      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 'g';
      Array[BYTE_MSG_TYPE + 2] = 'T';
      Array[BYTE_MSG_TYPE + 3] = wm;
      BrickPiTx(BrickPi.Address[i], 4, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived != 5 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_U32) {
        motorMtx_unlock();
        return -1;
      }

      for (k = 4; k > 0; k--) {
        ret = (ret << 8) | Array[k];
      }

      motorMtx_unlock();
      return ret;
    }
  }

  motorMtx_unlock();
  return 0;
}

int motorGetEncoderPosition(unsigned char which_motors) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      int ret = 0;
      int k;

      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 'g';
      Array[BYTE_MSG_TYPE + 2] = 'P';
      Array[BYTE_MSG_TYPE + 3] = wm;
      BrickPiTx(BrickPi.Address[i], 4, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived != 5 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_I32) {
        motorMtx_unlock();
        return -1;
      }

      for (k = 4; k > 0; k--) {
        ret = (ret << 8) | Array[k];
      }

      motorMtx_unlock();
      return ret;
    }
  }

  motorMtx_unlock();
  return 0;
}

unsigned char motorGetStatusByte(unsigned char which_motors) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 'g';
      Array[BYTE_MSG_TYPE + 2] = 'B';
      Array[BYTE_MSG_TYPE + 3] = wm;
      BrickPiTx(BrickPi.Address[i], 4, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived != 2 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_U8) {
        motorMtx_unlock();
        return -1;
      }

      motorMtx_unlock();
      return (unsigned char)(Array[1]);
    }
  }

  motorMtx_unlock();
  return 0;
}

// If a motor in a bank is included, both motors of that bank will be reset!
int motorBankReset(unsigned char which_motors) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 'r';
      BrickPiTx(BrickPi.Address[i], 2, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived > 1 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_OK) {
        motorMtx_unlock();
        return -1;
      }
    }
  }

  motorMtx_unlock();
  return 0;
}

// If a motor in a bank is included, both motors of that bank will be started!
int motorBankStartBothInSync(unsigned char which_motors) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 's';
      Array[BYTE_MSG_TYPE + 2] = 'I';
      BrickPiTx(BrickPi.Address[i], 3, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived > 1 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_OK) {
        motorMtx_unlock();
        return -1;
      }
    }
  }

  motorMtx_unlock();
  return 0;
}

int motorResetEncoder(unsigned char which_motors) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 'r';
      Array[BYTE_MSG_TYPE + 2] = 'E';
      Array[BYTE_MSG_TYPE + 3] = wm;
      BrickPiTx(BrickPi.Address[i], 4, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived > 1 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_OK) {
        motorMtx_unlock();
        return -1;
      }
    }
  }

  motorMtx_unlock();
  return 0;
}

int motorSetSpeedTimeAndControl(unsigned char which_motors, signed char speed,
                                unsigned int duration, unsigned char control) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 's';
      Array[BYTE_MSG_TYPE + 2] = 'Z';
      Array[BYTE_MSG_TYPE + 3] = wm;
      Array[BYTE_MSG_TYPE + 4] = speed;
      Array[BYTE_MSG_TYPE + 5] = duration & 0xff;
      Array[BYTE_MSG_TYPE + 6] = (duration >> 8) & 0xff;
      Array[BYTE_MSG_TYPE + 7] = (duration >> 16) & 0xff;
      Array[BYTE_MSG_TYPE + 8] = (duration >> 24) & 0xff;
      Array[BYTE_MSG_TYPE + 9] = control;
      BrickPiTx(BrickPi.Address[i], 10, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived > 1 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_OK) {
        motorMtx_unlock();
        return -1;
      }
    }
  }

  motorMtx_unlock();
  return 0;
}

int motorSetEncoderSpeedTimeAndControl(unsigned char which_motors,
                                       signed int encoder, signed char speed,
                                       unsigned int duration, unsigned char control) {
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 's';
      Array[BYTE_MSG_TYPE + 2] = 'Y';
      Array[BYTE_MSG_TYPE + 3] = wm;
      Array[BYTE_MSG_TYPE + 4] = encoder & 0xff;
      Array[BYTE_MSG_TYPE + 5] = (encoder >> 8) & 0xff;
      Array[BYTE_MSG_TYPE + 6] = (encoder >> 16) & 0xff;
      Array[BYTE_MSG_TYPE + 7] = (encoder >> 24) & 0xff;
      Array[BYTE_MSG_TYPE + 8] = speed;
      Array[BYTE_MSG_TYPE + 9] = duration & 0xff;
      Array[BYTE_MSG_TYPE + 10] = (duration >> 8) & 0xff;
      Array[BYTE_MSG_TYPE + 11] = (duration >> 16) & 0xff;
      Array[BYTE_MSG_TYPE + 12] = (duration >> 24) & 0xff;
      Array[BYTE_MSG_TYPE + 13] = control;
      BrickPiTx(BrickPi.Address[i], 14, Array);
  
      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }
  
      if (BytesReceived > 1 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_OK) {
        motorMtx_unlock();
        return -1;
      }
    }
  }

  motorMtx_unlock();
  return 0;
}

unsigned char motorIsTimeDone(unsigned char which_motors) {
  unsigned char ret = 0;
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 'i';
      Array[BYTE_MSG_TYPE + 2] = 'T';
      Array[BYTE_MSG_TYPE + 3] = wm;
      BrickPiTx(BrickPi.Address[i], 4, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }
      
      if (BytesReceived != 2 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_U8) {
        motorMtx_unlock();
        return -1;
      }

      ret |= (unsigned char)(Array[1]);
    }
  }

  motorMtx_unlock();
  return ret;
}

unsigned char motorIsTachoDone(unsigned char which_motors) {
  unsigned char ret = 0;
  int i;
  motorMtx_lock();

  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 'i';
      Array[BYTE_MSG_TYPE + 2] = 'P';
      Array[BYTE_MSG_TYPE + 3] = wm;
      BrickPiTx(BrickPi.Address[i], 4, Array);
      
      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      } 
      
      if (BytesReceived != 2 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_U8) {
        motorMtx_unlock();
        return -1;
      } 
      
      ret |= (unsigned char)(Array[1]);
    } 
  } 
  
  motorMtx_unlock();
  return ret;
} 

unsigned char motorWaitUntilTimeDone(unsigned char which_motors) {
  unsigned char ret = 0;

  printf("motorWaitUntilTimeDone(%d)->", which_motors);

  usleep(50 * 1000);					// 50 milliseconds

  ret = motorIsTimeDone(which_motors);

  while (ret & MOTOR_STATUS_TIME) {
    if (ret & MOTOR_STATUS_STALL) {
      printf("stall\n");
      return MOTOR_STATUS_STALL;
    }

    printf(".");

    usleep(50 * 1000);
    ret = motorIsTimeDone(which_motors);
  }

  printf("done\n");
  return 0;
}

unsigned char motorWaitUntilTachoDone(unsigned char which_motors) {
  unsigned char ret = 0;

  usleep(50 * 1000);                                    // 50 milliseconds

  ret = motorIsTachoDone(which_motors);

  while (ret & MOTOR_STATUS_TACHO) {
    if (ret & MOTOR_STATUS_STALL) {
      return MOTOR_STATUS_STALL;
    } 
    
    usleep(50 * 1000);
    ret = motorIsTachoDone(which_motors);
  }

  return 0;
}

static inline signed char motorCalcFinalSpeed(signed char initialSpeed, unsigned char direction) {
  if (direction == MOTOR_DIRECTION_FORWARD) {
    return initialSpeed;
  }

  return -initialSpeed;
}

// XXX: BrakeHold is not yet implemented!
static inline unsigned char motorCalcNextActionBits(unsigned char next_action) {
  if (next_action == MOTOR_NEXT_ACTION_BRAKE) {
    return MOTOR_CONTROL_BRK;
  }
  else if (next_action == MOTOR_NEXT_ACTION_BRAKEHOLD) {
    return MOTOR_CONTROL_BRK | MOTOR_CONTROL_ON;
  }

  return 0;
}

void motorRunUnlimited(unsigned char which_motors, unsigned char direction,
                       signed char speed) {
  unsigned char ctrl = MOTOR_CONTROL_SPEED | MOTOR_CONTROL_GO;
  signed char sp = motorCalcFinalSpeed(speed, direction);
  motorSetSpeedTimeAndControl(which_motors, sp, 0, ctrl);
}

unsigned char motorRunMilliSeconds(unsigned char which_motors, unsigned char direction,
                                   signed char speed, unsigned int duration,
                                   unsigned char wait_for_completion,
                                   unsigned char next_action) {
  signed char sp = motorCalcFinalSpeed(speed, direction);
  unsigned char ctrl = MOTOR_CONTROL_SPEED | MOTOR_CONTROL_TIME | MOTOR_CONTROL_GO;
  ctrl |= motorCalcNextActionBits(next_action);
  motorSetSpeedTimeAndControl(which_motors, sp, duration, ctrl);

  if (wait_for_completion == MOTOR_WAIT_TO_COMPLETE) {
    return motorWaitUntilTimeDone(which_motors);
  }

  return 0;
}

unsigned char motorRunTachometer(unsigned char which_motors, unsigned char direction,
                                 signed char speed, signed int tachometer,
                                 unsigned char relative,
                                 unsigned char wait_for_completion,
                                 unsigned char next_action) {
  signed char sp = motorCalcFinalSpeed(speed, direction);
  unsigned char ctrl = MOTOR_CONTROL_SPEED | MOTOR_CONTROL_TACHO | MOTOR_CONTROL_GO;
  ctrl |= motorCalcNextActionBits(next_action);

  // The tachometer can be absolute or relative.
  // If it is absolute, we ignore the direction parameter.
  signed int final_tach = tachometer;

  if (relative == MOTOR_RUN_RELATIVE) {
    ctrl |= MOTOR_CONTROL_RELATIVE;

    // a (relative) forward command is always a positive tachometer reading
    final_tach = abs(tachometer);
    if (sp < 0) {
      // and a (relative) reverse command is always negative
      final_tach = -final_tach;
    }
  }

  motorSetEncoderSpeedTimeAndControl(which_motors, final_tach, sp, 0, ctrl);

  if (wait_for_completion == MOTOR_WAIT_TO_COMPLETE) {
    return motorWaitUntilTachoDone(which_motors);
  }

  return 0;
}

unsigned char motorRunDegrees(unsigned char which_motors, unsigned char direction,
                              signed char speed, signed int degrees,
                              unsigned char wait_for_completion,
                              unsigned char next_action) {
  return motorRunTachometer(which_motors, direction, speed, degrees * 2,
      MOTOR_RUN_RELATIVE, wait_for_completion, next_action);
}

unsigned char motorRunRotations(unsigned char which_motors, unsigned char direction,
                                signed char speed, signed int rotations,
                                unsigned char wait_for_completion,
                                unsigned char next_action) {
  return motorRunTachometer(which_motors, direction, speed, rotations * 360 * 2,
      MOTOR_RUN_RELATIVE, wait_for_completion, next_action);
}

int motorStop(unsigned char which_motors, unsigned char next_action) {
  int i;
  motorMtx_lock();
  
  for (i = 0; i < 2; i++) {
    unsigned char wm = (which_motors >> (i * 2)) & 0x03;

    if (wm) {
      Array[BYTE_MSG_TYPE] = MSG_TYPE_MOTOR;
      Array[BYTE_MSG_TYPE + 1] = 's';
      Array[BYTE_MSG_TYPE + 2] = 'p';
      Array[BYTE_MSG_TYPE + 3] = wm;
      Array[BYTE_MSG_TYPE + 4] = next_action;
      BrickPiTx(BrickPi.Address[i], 5, Array);

      if (BrickPiRx(&BytesReceived, Array, 20000)) {
        motorMtx_unlock();
        return -2;
      }

      if (BytesReceived > 1 || Array[BYTE_MSG_TYPE] != MSG_RESULT_MOTOR_OK) {
        motorMtx_unlock();
        return -1;
      }
    }
  }

  motorMtx_unlock();
  return 0;
}

unsigned int Bit_Offset = 0;

void AddBits(unsigned char byte_offset, unsigned char bit_offset, unsigned char bits, unsigned long value){
  unsigned char i = 0;
  while(i < bits){
    if(value & 0x01){
      Array[(byte_offset + ((bit_offset + Bit_Offset + i) / 8))] |= (0x01 << ((bit_offset + Bit_Offset + i) % 8));
    }
    value /= 2;
    i++;
  }
  Bit_Offset += bits;
}

unsigned long GetBits(unsigned char byte_offset, unsigned char bit_offset, unsigned char bits){
  unsigned long Result = 0;
  char i = bits;
  while(i){
    Result *= 2;
    Result |= ((Array[(byte_offset + ((bit_offset + Bit_Offset + (i - 1)) / 8))] >> ((bit_offset + Bit_Offset + (i - 1)) % 8)) & 0x01);    
    i--;
  }
  Bit_Offset += bits;
  return Result;
}

unsigned char BitsNeeded(unsigned long value){
  unsigned char i = 0;
  while(i < 32){
    if(!value)
      return i;
    value /= 2;
    i++;
  }
  return 31;
}

int BrickPiSetupSensors(){
  unsigned char i = 0;
  while(i < 2){
    int ii = 0;
    while(ii < 256){
      Array[ii] = 0;
      ii++;
    }
    Bit_Offset = 0;
    Array[BYTE_MSG_TYPE] = MSG_TYPE_SENSOR_TYPE;
    Array[BYTE_SENSOR_1_TYPE] = BrickPi.SensorType[PORT_1 + (i * 2)];
    Array[BYTE_SENSOR_2_TYPE] = BrickPi.SensorType[PORT_2 + (i * 2)];
    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
	  //US Fix starts
	  if(Array[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_ULTRASONIC_CONT)
	  {
		Array[BYTE_SENSOR_1_TYPE + ii] = TYPE_SENSOR_I2C;
		BrickPi.SensorI2CSpeed[port] = US_I2C_SPEED;
		BrickPi.SensorI2CDevices[port] = 1;
		BrickPi.SensorSettings[port][US_I2C_IDX] = BIT_I2C_MID | BIT_I2C_SAME;
		BrickPi.SensorI2CAddr[port][US_I2C_IDX] = LEGO_US_I2C_ADDR;
		BrickPi.SensorI2CWrite [port][US_I2C_IDX]    = 1;
		BrickPi.SensorI2CRead  [port][US_I2C_IDX]    = 1;
		BrickPi.SensorI2COut   [port][US_I2C_IDX][0] = LEGO_US_I2C_DATA_REG;
	  }
	  //US Fix Ends
      if(Array[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_I2C
      || Array[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_I2C_9V){
        AddBits(3, 0, 8, BrickPi.SensorI2CSpeed[port]);
        
        if(BrickPi.SensorI2CDevices[port] > 8)
          BrickPi.SensorI2CDevices[port] = 8;
        
        if(BrickPi.SensorI2CDevices[port] == 0)
          BrickPi.SensorI2CDevices[port] = 1;
        
        AddBits(3, 0, 3, (BrickPi.SensorI2CDevices[port] - 1));
        
        unsigned char device = 0;
        while(device < BrickPi.SensorI2CDevices[port]){
          AddBits(3, 0, 7, (BrickPi.SensorI2CAddr[port][device] >> 1));
          AddBits(3, 0, 2, BrickPi.SensorSettings[port][device]);
          if(BrickPi.SensorSettings[port][device] & BIT_I2C_SAME){          
            AddBits(3, 0, 4, BrickPi.SensorI2CWrite[port][device]);
            AddBits(3, 0, 4, BrickPi.SensorI2CRead [port][device]);
            unsigned char out_byte = 0;
            while(out_byte < BrickPi.SensorI2CWrite[port][device]){
              AddBits(3, 0, 8, BrickPi.SensorI2COut[port][device][out_byte]);
              out_byte++;
            }
          }
          device++;
        }
      }
      ii++;
    }
    unsigned char UART_TX_BYTES = (((Bit_Offset + 7) / 8) + 3);
    BrickPiTx(BrickPi.Address[i], UART_TX_BYTES, Array);
    if(BrickPiRx(&BytesReceived, Array, 5000000)) {
      return -1;
    }
    if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_SENSOR_TYPE))
      return -1;
    i++;
  }
  return 0;
}

unsigned char Retried = 0; // For re-trying a failed update.

int BrickPiUpdateValues(){
  unsigned char i = 0;
  unsigned int ii = 0;
  while(i < 2){
    Retried = 0;    

__RETRY_COMMUNICATION__:
    
    ii = 0;
    while(ii < 256){
      Array[ii] = 0;
      ii++;
    }
    
    Array[BYTE_MSG_TYPE] = MSG_TYPE_VALUES;
    
    Bit_Offset = 0;
    
//    AddBits(1, 0, 2, 0);     use this to disable encoder offset
    
    ii = 0;                 // use this for encoder offset support
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      if(BrickPi.EncoderOffset[port]){
        long Temp_Value = BrickPi.EncoderOffset[port];
        unsigned char Temp_ENC_DIR;
        unsigned char Temp_BitsNeeded;
        
        AddBits(1, 0, 1, 1);
        if(Temp_Value < 0){
          Temp_ENC_DIR = 1;
          Temp_Value *= (-1);
        }
        Temp_BitsNeeded = (BitsNeeded(Temp_Value) + 1);
        AddBits(1, 0, 5, Temp_BitsNeeded);
        Temp_Value *= 2;
        Temp_Value |= Temp_ENC_DIR;
        AddBits(1, 0, Temp_BitsNeeded, Temp_Value);
      }
      else{
        AddBits(1, 0, 1, 0);
      }
      ii++;
    }
    
    int speed;
    unsigned char dir;    
    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      speed = BrickPi.MotorSpeed[port];
      dir = 0;
      if(speed < 0){
        dir = 1;
        speed *= (-1);
      }
      if(speed > 255){
        speed = 255;
      }
      AddBits(1, 0, 10, ((((speed & 0xFF) << 2) | (dir << 1) | (BrickPi.MotorEnable[port] & 0x01)) & 0x3FF));
      ii++;
    }
    
    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      //if(BrickPi.SensorType[port] == TYPE_SENSOR_I2C || BrickPi.SensorType[port] == TYPE_SENSOR_I2C_9V){
	  //US Fix starts
	  if(BrickPi.SensorType[port] == TYPE_SENSOR_I2C || BrickPi.SensorType[port] == TYPE_SENSOR_I2C_9V|| BrickPi.SensorType[port] == TYPE_SENSOR_ULTRASONIC_CONT){
	  //US Fix ends
        unsigned char device = 0;
        while(device < BrickPi.SensorI2CDevices[port]){
          if(!(BrickPi.SensorSettings[port][device] & BIT_I2C_SAME)){
            AddBits(1, 0, 4, BrickPi.SensorI2CWrite[port][device]);
            AddBits(1, 0, 4, BrickPi.SensorI2CRead [port][device]);
            unsigned char out_byte = 0;
            while(out_byte < BrickPi.SensorI2CWrite[port][device]){
              AddBits(1, 0, 8, BrickPi.SensorI2COut[port][device][out_byte]);
              out_byte++;
            }
          }
          device++;
        }
      }
      ii++;
    }
    
    unsigned char UART_TX_BYTES = (((Bit_Offset + 7) / 8) + 1);
    BrickPiTx(BrickPi.Address[i], UART_TX_BYTES, Array);
    
    int result = BrickPiRx(&BytesReceived, Array, 7500);
    
    if(result != -2){                            // -2 is the only error that indicates that the BrickPi uC did not properly receive the message
      BrickPi.EncoderOffset[((i * 2) + PORT_A)] = 0;
      BrickPi.EncoderOffset[((i * 2) + PORT_B)] = 0;
    }
    
    if(result || (Array[BYTE_MSG_TYPE] != MSG_TYPE_VALUES)){
#ifdef DEBUG
      printf("BrickPiRx error: %d\n", result);
#endif
      if(Retried < 2){
        Retried++;
        goto __RETRY_COMMUNICATION__;
      }
      else{
#ifdef DEBUG
        printf("Retry failed.\n");
#endif
        return -1;
      }      
    }
    
    Bit_Offset = 0;
    
    unsigned char Temp_BitsUsed[2] = {0, 0};         // Used for encoder values
    Temp_BitsUsed[0] = GetBits(1, 0, 5);
    Temp_BitsUsed[1] = GetBits(1, 0, 5);
    unsigned long Temp_EncoderVal;
    
    ii = 0;
    while(ii < 2){
      Temp_EncoderVal = GetBits(1, 0, Temp_BitsUsed[ii]);
      if(Temp_EncoderVal & 0x01){
        Temp_EncoderVal /= 2;
        BrickPi.Encoder[ii + (i * 2)] = Temp_EncoderVal * (-1);}
      else{
        BrickPi.Encoder[ii + (i * 2)] = (Temp_EncoderVal / 2);}      
      ii++;
    }

    ii = 0;
    while(ii < 2){
      unsigned char port = ii + (i * 2);
      switch(BrickPi.SensorType[port]){
        case TYPE_SENSOR_TOUCH:
          BrickPi.Sensor[port] = GetBits(1, 0, 1);
        break;
		//US Fix
        //case TYPE_SENSOR_ULTRASONIC_CONT:
        //case TYPE_SENSOR_ULTRASONIC_SS:
		case TYPE_SENSOR_ULTRASONIC_SS:
          BrickPi.Sensor[port] = GetBits(1, 0, 8);
        break;
        case TYPE_SENSOR_COLOR_FULL:
          BrickPi.Sensor[port] = GetBits(1, 0, 3);
          BrickPi.SensorArray[port][INDEX_BLANK] = GetBits(1, 0, 10);
          BrickPi.SensorArray[port][INDEX_RED  ] = GetBits(1, 0, 10);                
          BrickPi.SensorArray[port][INDEX_GREEN] = GetBits(1, 0, 10);
          BrickPi.SensorArray[port][INDEX_BLUE ] = GetBits(1, 0, 10);
        break;          
        case TYPE_SENSOR_I2C:
        case TYPE_SENSOR_I2C_9V:
          {
		//US Fix Starts
		case TYPE_SENSOR_ULTRASONIC_CONT:
		//US Fix Ends
            BrickPi.Sensor[port] = GetBits(1, 0, BrickPi.SensorI2CDevices[port]);
            unsigned char device = 0;
            while(device < BrickPi.SensorI2CDevices[port]){
              if(BrickPi.Sensor[port] & (0x01 << device)){
                unsigned char in_byte = 0;
                while(in_byte < BrickPi.SensorI2CRead[port][device]){
                  BrickPi.SensorI2CIn[port][device][in_byte] = GetBits(1, 0, 8);
                  in_byte++;
                }
              }
              device++;
            }
          }
        break;      
        case TYPE_SENSOR_EV3_US_M0       :
        case TYPE_SENSOR_EV3_US_M1       :
        case TYPE_SENSOR_EV3_US_M2       :
        case TYPE_SENSOR_EV3_US_M3       :
        case TYPE_SENSOR_EV3_US_M4       :
        case TYPE_SENSOR_EV3_US_M5       :
        case TYPE_SENSOR_EV3_US_M6       :
        case TYPE_SENSOR_EV3_COLOR_M0    :
        case TYPE_SENSOR_EV3_COLOR_M1    :
        case TYPE_SENSOR_EV3_COLOR_M2    :
        case TYPE_SENSOR_EV3_COLOR_M4    :
        case TYPE_SENSOR_EV3_COLOR_M5    :
        case TYPE_SENSOR_EV3_GYRO_M0     :
        case TYPE_SENSOR_EV3_GYRO_M1     :
        case TYPE_SENSOR_EV3_GYRO_M2     :
        case TYPE_SENSOR_EV3_GYRO_M4     :
        case TYPE_SENSOR_EV3_INFRARED_M0 :
        case TYPE_SENSOR_EV3_INFRARED_M1 :
        case TYPE_SENSOR_EV3_INFRARED_M3 :
        case TYPE_SENSOR_EV3_INFRARED_M4 :
        case TYPE_SENSOR_EV3_INFRARED_M5 :
        case TYPE_SENSOR_EV3_TOUCH_0:
          BrickPi.Sensor[port] = GetBits(1, 0, 16);
          // gotten_bits = GetBits(1, 0, 16);		  		// Just test code
		  // printf("First Test: %d \n", gotten_bits );		// Just test code
        break;
        case TYPE_SENSOR_EV3_COLOR_M3    :
        case TYPE_SENSOR_EV3_GYRO_M3     :
        case TYPE_SENSOR_EV3_INFRARED_M2 :
          
			// Filter out negative values.
			// Test and shift.  Get it as a 32 bit number.  
			gotten_bits = GetBits(1,0,32);	
			/*if(gotten_bits > 10 && gotten_bits >= 0){
				gotten_bits = gotten_bits>>8;
				printf("First Test: %d \n", gotten_bits );	
			}*/
			// gotten_bits = GetBits(1,0,8);	
			// printf("Results: %d \n", gotten_bits );	
			// if(gotten_bits < 10 && gotten_bits >= 0){
				BrickPi.Sensor[port] = gotten_bits;
			//}
			break;
        case TYPE_SENSOR_LIGHT_OFF:
        case TYPE_SENSOR_LIGHT_ON:
        case TYPE_SENSOR_RCX_LIGHT:
        case TYPE_SENSOR_COLOR_RED:
        case TYPE_SENSOR_COLOR_GREEN:
        case TYPE_SENSOR_COLOR_BLUE:
        case TYPE_SENSOR_COLOR_NONE:
        default:
          BrickPi.Sensor[(ii + (i * 2))] = GetBits(1, 0, 10);
      }
	  if (BrickPi.SensorType[port] == TYPE_SENSOR_ULTRASONIC_CONT)
	  {
	    if(BrickPi.Sensor[port] & ( 0x01 << US_I2C_IDX)) 
			BrickPi.Sensor[port] = BrickPi.SensorI2CIn[port][US_I2C_IDX][0];
		else
			BrickPi.Sensor[port] = -1;
	  }
      ii++;
    }      
    i++;
  }
  return 0;
}

int UART_file_descriptor = 0; 

int BrickPiSetup() {
  UART_file_descriptor = serialOpen("/dev/ttyAMA0", 500000);

  if (UART_file_descriptor < 0) {
    return -1;
  }
  return 0;
}

static int BrickPiTx(unsigned char dest, unsigned char ByteCount, unsigned char OutArray[]) {
  unsigned char tx_buffer[256];
  tx_buffer[0] = dest;
  tx_buffer[1] = dest + ByteCount;
  tx_buffer[2] = ByteCount;
  unsigned char i;

  for (i = 0; i < ByteCount; i++) {
    tx_buffer[1] += OutArray[i];
    tx_buffer[i + 3] = OutArray[i];
  }

  return write(UART_file_descriptor, tx_buffer, ByteCount + 3);
}

static int BrickPiRx(unsigned char *InBytes, unsigned char InArray[], long timeout) {  // timeout in uS, not mS
  unsigned char rx_buffer[256];
  unsigned char RxBytes = 0;
  unsigned char CheckSum = 0;
  unsigned char i = 0;
  int result;
  unsigned long OrigionalTick = CurrentTickUs();

  while (serialDataAvail(UART_file_descriptor) <= 0) {
    if (timeout && ((CurrentTickUs() - OrigionalTick) >= timeout)) {
      return -2;
    }
  }

  RxBytes = 0;
  while (RxBytes < serialDataAvail(UART_file_descriptor)) {    // If it's been 1 ms since the last
                                                               // data was received, assume it's the
                                                               //end of the message.
    RxBytes = serialDataAvail(UART_file_descriptor);
    usleep(500); // MARCO how do 75 us compare to the comment about 1 ms?
  }
  
  for (i = 0; i < RxBytes; i++) {
    result = serialGetchar(UART_file_descriptor);

    if (result >= 0) {
      rx_buffer[i] = result;
    }
    else {      
      return -1;    
    }
  }

  if (RxBytes < 2) {
    return -4;
  }
  
  if (RxBytes < (rx_buffer[1] + 2)) {
    return -6;
  }
  
  CheckSum = rx_buffer[1];
  
  for (i = 2; i < RxBytes; i++) {
    CheckSum += rx_buffer[i];
    InArray[i - 2] = rx_buffer[i];
  }
  
  if (CheckSum != rx_buffer[0]) {
    return -5;
  }
  
  *InBytes = (RxBytes - 2);

  return 0;  
}
__END_NAMESPACE_STD

#endif
