/*
 * Author:  Jesus Minjares
 *
 * App:
 *          Software Integration of IPIA System. The system will run for 1 minute to gather data
 *          from the GPS, enough time for hot and cold start. Once the minute has passed, it will parse the
 *          GNRMC string into data structures: NMEA_GNRMC, UTC_TIME and COORDIANTE. THe UTC_TIME will be parse to
 *          initialize the system timer that hold its data on SysTime (Data structure to hold current time). The system will
 *          trigger an ADC sampling every 10ms. During the ADC sampling, it will send the coordinates, time stamp, adc raw reading and voltage.
 *          Initiate Solar Tracker to get max voltage reading from a solar panel, every 5 minute the solar tracker will be trigger for demostration
 *          purposes.
 *          i.e
 *              latitude longitude time_stamp adc_raw_reading voltage
 *
 *
 *               ---------------------
 *            | GPS Module (SIM33EAU) | @ 9600
 *             -----------------------
 *
 *                MSP432P401x
 *             -----------------
 *            |          0V(GND)|--> GND
 *            |                 |
 *            |RST        3.3-5V|--> VCC
 *            |                 |
 *            |         P3.0(RX)|--> TX
 *            |         P3.6    |--> EN
 *            |                 |
 *            |_________________|
 *
 *             --------------
 *            | HC-05 Sensor |  @ 9600
 *             --------------
 *
 *      S: State
 *      RX: Receiver
 *      TX: Transmitter
 *      VCC: 3.3-5v
 *      GND: 0v
 *      EN: Set on high, for AT Mode
 *                                                -----------------
 *               MSP432P401x                     |      HC-05      |
 *            -----------------                   S RX TX VCC GND EN
 *           |          0V(GND)|-->HC-05 GND
 *           |                 |
 *           |RST         VCC  |-->HC-05 VCC
 *           |                 |
 *           |         P3.2(RX)|-->HC-05 TX
 *           |         P3.3(TX)|-- 1K-.<--HC-05 RX
 *HC-05 EN<--|P2.3             |      |
 *           |_________________|      2k
 *                                    |
 *                                    GND
 *
 *             -----------------
 *            | Analog Readings |
 *             -----------------
 *               MSP432P401x
 *             -----------------
 *            |                 |
 *            |                 |
 *            |          (P5.4) |--> Pressure Sensor
 *            |                 |
 *            |          (P5.5) |--> Solar Panel
 *            |                 |
 *            |                 |
 *            |_________________|
 *
*/
#include "msp.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "GPS.h"
#include "IPIA_LIB.h"
void main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    /*Initialize global variables*/
    initGlobal();

    /*SOLAR TRACKER*/
    RANGE = sizeof(pulse)/sizeof(uint16_t);
    clear(RANGE,RANGE);
    port2Setup(0xC0); //BIT6|BIT7 -> 1100 0000, C0
    pwmSetup();

    /*Initialize Clock at 3Mhz*/
    set3Mhz();
    /*initialize putty for debugging*/
    puttySetup();
    /*initialize gpsUart to get data*/
    gpsUARTSetup();
    /*enable ENable pin for the GPS*/
    gpsToggleSetup(0x40); //BIT6 -> 0100 0000, 40

    /*enable bluetooth*/
    bluetoothSetup();

    /*wait for the cold start of the GPS, 15 cold start...wait a minute to get stable data */
    int delay;
    for(delay = 0; delay < 30; delay++) //iterate 30 times
        __delay_cycles(6000000);//2 sec delay


    /*store GNMRC string into the NMEA_GNRMC structure*/
    gps = setGnrmc(buffer);
    /*store the GPS Coordinates in the COORDINATE Structure*/
    gpsCoordinates = setCoordinates(&gps);
    /*store the UTC time to the UTC_TIME Structure*/
    gpsTime = setTime(gps.utc_time);
    /*Store to the UTC-TIME into system timer, to time stamp data*/
    sysTime = setSysTime(&gpsTime);

    /*print GNRMC, Coordinates, and Time for debugging purposes*/

    //print_NMEA_GNRMC(&gps);
    //print_COORDINATE(&gpsCoordinates);
    //print_UTC_TIME(&gpsTime);

    /*enable 5.4 and 5.5 for ADC*/
    port5Setup(0x30); //BIT4|BIT5 -> 0011 0000 , 30

    /*enable TimerA2 at 10ms*/
    timerA2Setup();
    /*enable single channel repeat at 100Hz*/
    adcSetup();

    /*enable global interrupts*/
    __enable_irq();

    /*infinite loop
     * fix: replace with low power mode
     * */
    while(1){}

}
/**
 * GPS UART Interrupt Subroutine
 * note:    The interrupt will capture the data from
 *          the GPS and store it in buffer.
 */
void EUSCIA1_IRQHandler(void){
    static uint8_t gpsColdStart = 0; //create a static variable
    if (EUSCI_A1->IFG & EUSCI_A_IFG_RXIFG){ //check the flag
        buffer[buffer_index++] = EUSCI_A1->RXBUF; //store character, increment index
        /*the gps will end with \r\n\0, so check if the last 3 character match */
        if(buffer[buffer_index-2] == carriageReturn && buffer[buffer_index - 1] == newLineFeed  && buffer[buffer_index] == '\0'){
            if(isSubstring((char*)buffer,gpsUartStr) && gpsColdStart++ < 50){ //check if gpsUartStr is a substring and if gpsColdStart is < 50
                sendBluetooth(buffer); //send via bluetooth
            }
            if(gpsColdStart < 50){ //if gpsColdStart still less than 50
                memset(&buffer,0,sizeof(buffer)); //clear the buffer
                buffer_index = 0; //reset index
            }
            else{
                turnOffGps(BIT6); //turn off GPS via P3.6 (EN) Pin
                gpsColdStart = 0; //reset gpsColdStart
            }
        }
    }
}
/*
 * TIMERA2 Interrupt Subroutine
 * @note    TIMERA2 will be trigger every 1ms and
 *          update the current time for the sysTime.
 *          Will trigger ADC every 10 ms
 *
 */
void TA2_0_IRQHandler(){
    sysTime.milli++; //incrmement every TA2 interval
    if(sysTime.milli >= 999){ sysTime.second++; sysTime.milli = 0;  }
    if(sysTime.second >= 59){ sysTime.minute++; sysTime.second = 0; }
    if(sysTime.minute >= 59){ sysTime.hour++;   sysTime.minute = 0; }
    if(sysTime.hour >= 23){   sysTime.hour = 0;}
    if(sysTime.milli % 10 == 0){ //every 10ms, trigger conversion
        tic++; //increment tic
        if(tic == SAMPLE){ //every 15*10ms -> 150ms
             y++;//increment the index of the array
             if(y == RANGE){ //clear i when exceed to the end
                  y = 0; // set to 0
                  x++; //increment x axis
             }
             if(x == RANGE ){ //once X reaches LIMIT turn on flag
                 flag = ON; //turn flag on
             }
             if(flag == ON){
                 harvesting++; //start harvesting
                 if(harvesting == 1){
                    uint16_t *index =  findMax(ROW, COL); //find the max value of the grid
                    TIMER_A0->CCR[3] = pulse[index[1]] - 1; // change to the current pulse
                    TIMER_A0->CCR[4] = pulse[index[0]] - 1; // set to the max value index

                    volt = VPos/ADC14BIT * axis[index[0]][index[1]]; //convert digital reading into voltage

                 }
                 if(harvesting > 300000){//after 5 minutes
                     harvesting = x = volt = flag = OFF;
                     clear(RANGE,RANGE); //clear the grid
                 }
             }
             if(flag == OFF){
                TIMER_A0->CCR[3] = pulse[y] - 1; // change to the current pulse
                TIMER_A0->CCR[4] = pulse[x] - 1; // ^

                int i = 1;
                for(; i < SAMPLE; i++) //sum the samples to get an average
                    overSample += sample[i];

                uint16_t avg = (uint16_t)(overSample / (SAMPLE - 1)); //get the average reading in the ADC

                volt = VPos/ADC14BIT *avg; //convert average digital to voltage

                axis[x][y] = avg ; //store current avg reading into axis
                flag = OFF; //set flag as OFF
             }
             tic = 0; overSample = 0;
        }
        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC; //enable sample and start conversion
    }
    TIMER_A2->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; //clear the capture and compare flag of the timer
}

/**
 * ADC Interrupt Subroutine
 * @note    Get ADC reading every 10ms, and time stamp stamp the data
 *          send data via bluetooth once it done capturing
 */
void ADC14_IRQHandler(void){
    uint32_t adcRawReading = ADC14->MEM[1]; //A1 save adc reading
    uint32_t solarReading = ADC14->MEM[0];// A0
    if(ADC14->IFGR0 & ADC14_IFGR0_IFG1){
        char tempBuffer[48] = ""; //create an empty buffer
        char *time = getTime(); //store time

        sprintf(tempBuffer,"%s %s ",gpsCoordinates.latitude, gpsCoordinates.longitude);
        sendBluetooth(tempBuffer);

        sendBluetooth(time);
        sprintf(tempBuffer, "%u %lf\r\n",adcRawReading, (3.3/16384 * adcRawReading)); // output the message of adc reading
        sendBluetooth(tempBuffer); //send message

        sample[tic] = solarReading;
    }
}
