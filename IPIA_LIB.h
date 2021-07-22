/*
 * IPIA_LIB.h
 *
 *  Created on: Nov 29, 2020
 *      Author: Jesus Minjares
 */

#ifndef IPIA_LIB_H_
#define IPIA_LIB_H_

#include "msp.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "GPS.h"

/*MACROS*/
#define SAMPLE  15
#define CLK 3000000
#define ROW 21
#define COL 21
#define ON  1
#define OFF 0

uint16_t* findMax(int row, int col);
void port2Setup(uint8_t pwmBit);
void pwmSetup();
void clear(int row, int col); //clear array/matrix

/*Port Remap */
void portRemap(); //reconfigure UART1 pin P2.2 -> P3.0 for a more practical use

/*Clock*/
void set3Mhz(); //set the DC0 @ 3Mhz

/*GPS*/
void gpsUARTSetup(); //enable P3.0 to read GPS via Uart @ 9600
void gpsToggleSetup(uint8_t bit); //set P3.6 as an output
void turnOnGps(uint8_t bit); //bit set P3.6 to enable GPS
void turnOffGps(uint8_t bit); //bit clear P3.6 to disable GPS

/*adc*/
void adcSetup(); //enable ADC single channel A0
void port5Setup(uint8_t bit); //enable P5.4 to read analog data from the sensor

/*Addition functions*/
char * getTime(); //return the SysTime data structure as a string
char *getCoordinate(); //return the gpsCoordinate as a string
bool isSubstring(char *str, char *substr); //check if the substring exist in the string
uint8_t str_to_uint8(char *str);  //convert a string to an unsigned int of 8 bytes  (0-128)
uint16_t str_to_uint16(char *str); //convert a string to an unsigned int of 16 bytes (0-1024)

/*bluetooth*/
void sendBluetooth(char *message); //send message via bluetooth
void bluetoothSetup(); //initialize UART2/ bluetooth , P3.2

/*Putty*/
void sendPutty(char *message); //send messaga via serial port/USB using UART0
void puttySetup(); //enable PcEcho

/*Timer A2*/
void timerA2Setup(); //set timerA2 @ 10ms intervals


/*Data Structures*/
NMEA_GNRMC gps; //hold GNMRC parsed
COORDINATE gpsCoordinates; //hold latitude and longitude in degrees
UTC_TIME gpsTime; //hold the utc time

/*Data structure to hold current time */
typedef struct SysTime{
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t milli;
}SysTime;
SysTime sysTime; //create an instance of the sysTime data structure to hold current time
SysTime setSysTime(const UTC_TIME  *gpsUtcTime); //set sysTime data structure using UTC_TIME data structure

/*Global variables*/
char *gpsUartStr; //string that is going to search
char carriageReturn; //  '\r'
char newLineFeed;   //   '\n'
int buffer_index;
char buffer[82]; //82 bytes long as the NMEA is the max size of a string
uint16_t sample[SAMPLE];
float ADC14BIT;
float VPos;
float VNeg; //set float variable to get voltage reading
uint16_t *pulse;
uint8_t flag, overSample;
uint32_t harvesting; //software flag and harvesting counter
uint16_t axis[ROW][COL]; //set grid of ROW * COL, adc value at 0
uint8_t RANGE, tic, y, x;
float volt; //voltage variable

/*Initialize Global Variables*/
void initGlobal();

#endif /* IPIA_LIB_H_ */
