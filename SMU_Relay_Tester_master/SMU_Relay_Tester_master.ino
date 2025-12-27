// SMU Relay Tester (Master)
// Rev 1.0 (27/12/2025)
// - Maxtrax

#include <Wire.h>
#include <SPI.h>
#include <PCA9540BD.h>

//#define VERBOSE_REPLY //uncomment this for replying ACK/NACK/Debug logging

//define DEBUG //uncomment this line to print debug data to the serial bus
//#define MASTER_TEST //uncomment to perform relay test loop on master board
//#define SLAVE_TEST //uncomment to perform relay test loop on slave board

#define SPI_TRANSFER_BUFFER //for buffer transfer instead of byte by byte

//----------------------------------------------
// update the below configurations for loop test
//----------------------------------------------
#define SLAVE_SLOT_START 2
#define SLAVE_SLOT_END 2
#define SLAVE_RELAY_START 13
#define SLAVE_RELAY_END 24
#define TEST_DELAY_MSEC 500
//----------------------------------------------

#define SPI_TRANSFER_CLOCK_FREQ_100K 100000
#define SPI_TRANSFER_CLOCK_FREQ_500K 500000
#define SPI_TRANSFER_CLOCK_FREQ_2M 2000000
#define SPI_TRANSFER_CLOCK_FREQ_4M 4000000
#define SPI_TRANSFER_CLOCK_FREQ_8M 8000000
#define SPI_TRANSFER_CLOCK_FREQ_10M 10000000
#define SPI_TRANSFER_CLOCK_FREQ_12M 12000000

#define SPI_TRANSFER_CLOCK_FREQ SPI_TRANSFER_CLOCK_FREQ_100K

const char * app_ver = "v1.0";

const char * ACK_STR = "ACK";
const char * NACK_STR = "NACK";
const char SLOT_CHAR = 'S';
const char END_CHAR = 'E';
const char RELAY_CHAR = 'R';
const char RESET_CHAR = 'X';
const char FRESET_CHAR = 'F';
const char RELAYGRP1_CHAR = 'P';
const char RELAYGRP2_CHAR = 'R';
const char RELAYGRP_CHAR = 'Q';
const char ON_CHAR = '1';
const char OFF_CHAR = '0';
const char DELIM = ',';

const byte FAULT_LED_PIN = 15; //Red
const byte DIAG_LED_PIN = 16; //Green
const byte SLAVE_RESET_PIN = 21;

const byte SPI_CS1_PIN = 0;
const byte SPI_CS2_PIN = 1;
const byte SPI_CS3_PIN = 2;
const byte SPI_CS4_PIN = 3;
const byte SPI_CS5_PIN = 4;
const byte SPI_CS6_PIN = 5;
const byte SPI_CS7_PIN = 6;
const byte SPI_CS8_PIN = 7;
const byte MAX_SLAVE_BOARD = 8;

const int MAX_BUFFERED_CMD = 64;

const int MAX_DELIMS = 16;

//supported on/off durations in ms
const int DURATION_MS[5] = {
    100,
    200,
    300,
    400,
    500
};

PCA9540BD multiplexer_U1; //PCA9540BD Mux (0x70)

SPISettings settings = SPISettings(SPI_TRANSFER_CLOCK_FREQ, MSBFIRST, SPI_MODE0);

int delim_count = 0;
int delim_idx[MAX_DELIMS] = {};
int cmd_idx = 0;
char cmd_str[MAX_BUFFERED_CMD] = {};

byte SPI_CS_PINS[MAX_SLAVE_BOARD] = {
    SPI_CS1_PIN,
    SPI_CS2_PIN,
    SPI_CS3_PIN,
    SPI_CS4_PIN,
    SPI_CS5_PIN,
    SPI_CS6_PIN,
    SPI_CS7_PIN,
    SPI_CS8_PIN
};

#ifdef VERBOSE_REPLY
void printReply(const char * reply)
{
    Serial.println(reply);
}
#else
void printReply(const char * /*reply*/) {};
#endif

void resetBuffer()
{
    delim_count = 0;
    memset(delim_idx, 0, sizeof(delim_idx));
    memset(cmd_str, 0, MAX_BUFFERED_CMD);
}

void setup() {
    // put your setup code here, to run once:
    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    while (!Serial);

    Serial.print("SMU Relay Tester (Master)");
    Serial.println(app_ver);

    Wire.begin(); //need to start the Wire for I2C devices to function

    SPI.begin();
    pinMode(SPI_CS1_PIN, OUTPUT);
    pinMode(SPI_CS2_PIN, OUTPUT);
    pinMode(SPI_CS3_PIN, OUTPUT);
    pinMode(SPI_CS4_PIN, OUTPUT);
    pinMode(SPI_CS5_PIN, OUTPUT);
    pinMode(SPI_CS6_PIN, OUTPUT);
    pinMode(SPI_CS7_PIN, OUTPUT);
    pinMode(SPI_CS8_PIN, OUTPUT);
    
    //begin spi transaction here and not end it, as we're the only SPI user
    SPI.beginTransaction(settings);
    for (byte i = 0; i < MAX_SLAVE_BOARD; i++)
    {
        digitalWrite(SPI_CS_PINS[i], HIGH);
    }

    pinMode(SLAVE_RESET_PIN, OUTPUT);
    digitalWrite(SLAVE_RESET_PIN, HIGH);
    delay(500); //500ms for slaves to reset
    digitalWrite(SLAVE_RESET_PIN, LOW);

    multiplexer_U1.selectChannel(1); // for selecting ioExp1 channel, will only be using this channel

    pinMode(DIAG_LED_PIN, OUTPUT);
}

void loop() {
    // put your main code here, to run repeatedly:
    if (Serial.available())
    {
        char tmp_char = Serial.read();

        //sanitize and remove unwanted characters (ctrl/special char and SPACE)
        if (('!' <= tmp_char) && (tmp_char <= '~'))
        {
            if (tmp_char == SLOT_CHAR)
            {
                //when detected start, reset index and clear buffer
                cmd_idx = 0;
                resetBuffer();
            }

            cmd_str[cmd_idx] = tmp_char;

            if (cmd_str[cmd_idx] == DELIM)
            {
                if (delim_count < MAX_DELIMS)
                {
                #ifdef DEBUG
                    Serial.print("Delim [");
                    Serial.print(delim_count);
                    Serial.print("] at index: ");
                    Serial.println(cmd_idx);
                #endif
                    delim_idx[delim_count] = cmd_idx;
                    delim_count++;
                }
            }

            //check for END, after 5th or 8th delimiter found and 
            //at least 1 byte arrived after the delimiter
            if ( ((delim_count == 5) || (delim_count == 9)) && 
                (cmd_idx > delim_idx[delim_count-1]) &&
                (cmd_str[delim_idx[delim_count-1]+1] == END_CHAR) )
            {
                //parse first data for request type
                if (cmd_str[0] == SLOT_CHAR)
                {
                    if ( (cmd_str[delim_idx[0]+1] == RELAYGRP_CHAR) || //for group commands
                        (cmd_str[delim_idx[0]+1] == RELAYGRP1_CHAR) || (cmd_str[delim_idx[0]+1] == RELAYGRP2_CHAR) )
                    {
                        byte start = (cmd_str[delim_idx[0]+1] == RELAYGRP2_CHAR) ? (MAX_SLAVE_BOARD/2) : 0;
                        byte end = (cmd_str[delim_idx[0]+1] == RELAYGRP1_CHAR) ? (MAX_SLAVE_BOARD/2) : MAX_SLAVE_BOARD;

                        //replace R with Q for RELAYGRP first byte
                        cmd_str[delim_idx[1]+1] = RELAYGRP_CHAR;

                        for (byte slot = start; slot < end; slot++)
                        {
                            digitalWrite(SPI_CS_PINS[slot], LOW);
                            delay(1);

                            //start sending to SPI lines begining of RELAY request type
                        #ifdef SPI_TRANSFER_BUFFER
                            char xfer_cmd[MAX_BUFFERED_CMD] = {};
                            int start = delim_idx[1]+1;
                            int end = cmd_idx-start+1;

                            memcpy(xfer_cmd, &cmd_str[start], end);
                            SPI.transfer(xfer_cmd, end);
                        #else
                            for (int i = delim_idx[1]+1; i <= cmd_idx; i++)
                            {
                                SPI.transfer(cmd_str[i]);
                                delayMicroseconds(1000); // play with this parameter
                            }
                        #endif

                            digitalWrite(SPI_CS_PINS[slot], HIGH);
                            delay(1);

                            printReply(ACK_STR);
                        }
                    }
                    else
                    {
                        //check for SLOT number
                        int SLOT_num = atoi(&cmd_str[delim_idx[0]+1]);

                        if ( (SLOT_num >= 1) && (SLOT_num <= 8) ) //SLOT 1-8 are external relays
                        {
                            digitalWrite(SPI_CS_PINS[SLOT_num-1], LOW);
                            delay(1);

                            //start sending to SPI lines begining of RELAY request type
                        #ifdef SPI_TRANSFER_BUFFER
                            char xfer_cmd[MAX_BUFFERED_CMD] = {};
                            int start = delim_idx[1]+1;
                            int end = cmd_idx-start+1;

                            memcpy(xfer_cmd, &cmd_str[start], end);
                            SPI.transfer(xfer_cmd, end);
                        #else
                            for (int i = delim_idx[1]+1; i <= cmd_idx; i++)
                            {
                                SPI.transfer(cmd_str[i]);
                                delayMicroseconds(1000); // play with this parameter
                            }
                        #endif

                            digitalWrite(SPI_CS_PINS[SLOT_num-1], HIGH);
                            delay(1);

                            printReply(ACK_STR);
                        }
                        else if (SLOT_num == 0) //SLOT 0 is for local relay
                        {
                            //parse next data for RELAY request type
                            if (cmd_str[delim_idx[1]+1] == RELAY_CHAR)
                            {
                                if (cmd_str[delim_idx[2]+1] == RESET_CHAR)
                                {
                                    if (cmd_str[delim_idx[3]+1] == FRESET_CHAR)
                                    {
                                        pinMode(SLAVE_RESET_PIN, OUTPUT);
                                        digitalWrite(SLAVE_RESET_PIN, HIGH);
                                        delay(500); //500ms for slaves to reset
                                        digitalWrite(SLAVE_RESET_PIN, LOW);
                                        printReply(ACK_STR);
                                    }
                                    else
                                    {
                                        //Serial.println("ERROR: unknown RELAY command");
                                        printReply(NACK_STR);
                                    }
                                }
                            }
                            else
                            {
                                //Serial.println("ERROR: unknown RELAY request");
                                printReply(NACK_STR);
                            }
                        }
                        else
                        {
                            //Serial.println("ERROR: unknown SLOT");
                            printReply(NACK_STR);
                        }
                    }
                }
                else
                {
                    //Serial.println("ERROR: unknown SLOT request");
                    printReply(NACK_STR);
                }

                //reset index and clear buffer when done processing
                resetBuffer();
                cmd_idx = -1;
            }

            cmd_idx++;

            if (cmd_idx >= MAX_BUFFERED_CMD) //buffer overflowing!
            {
                //Serial.println("ERROR: command buffer overflowing");
                printReply(NACK_STR);
                resetBuffer();
                cmd_idx = 0;
            }
        }
    }
}
