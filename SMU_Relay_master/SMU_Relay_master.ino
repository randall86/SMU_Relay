// SMU Relay (Master)
// Rev 3.2 (17/4/2025)
// - Maxtrax

#include <Wire.h>
#include <SPI.h>
#include <PCA9540BD.h>
#include <DTIOI2CtoParallelConverter.h>

#define VERBOSE_REPLY //uncomment this for replying ACK/NACK/Debug logging

//define DEBUG //uncomment this line to print debug data to the serial bus
//#define RUN_RELAY_TEST //uncomment to perform relay test loop
//#define MASTER_TEST //uncomment to perform relay test loop on master board
//#define SLAVE_TEST //uncomment to perform relay test loop on slave board

#define SPI_TRANSFER_BUFFER //for buffer transfer instead of byte by byte
//#define SPI_2X_HACK_GRPCMD //for sending 2x commands to slave (same as previous)

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

const char * app_ver = "v3.2";

const char * ACK_STR = "ACK";
const char * NACK_STR = "NACK";
const char SLOT_CHAR = 'S';
const char END_CHAR = 'E';
const char RELAY_CHAR = 'R';
const char RESET_CHAR = 'X';
const char FRESET_CHAR = 'F';
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
DTIOI2CtoParallelConverter ioExp1_U2(0x74);  //PCA9555 I/O Expander (with A1 = 0 and A0 = 0)
DTIOI2CtoParallelConverter ioExp1_U3(0x75);  //PCA9555 I/O Expander (with A1 = 0 and A0 = 1)

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

byte SLAVE_DET_PINS[MAX_SLAVE_BOARD] = {
    PIN1_0,
    PIN1_1,
    PIN1_2,
    PIN1_3,
    PIN1_4,
    PIN1_5,
    PIN1_6,
    PIN1_7
};

void runRelayTestLoop()
{
    digitalWrite(DIAG_LED_PIN, HIGH);
#ifdef MASTER_TEST
    // test slot 0 relays - master board
    for (byte pin = 0; pin < 16; pin++)
    {
        // send relay ON command
        relayWriteWrapper(pin, true);

        delay(TEST_DELAY_MSEC);

        // send relay OFF command
        relayWriteWrapper(pin, false);

        delay(TEST_DELAY_MSEC);
    }
#endif
    delay(1000);
#ifdef SLAVE_TEST
    // construct relay commands
    String on_cmd[128];
    String off_cmd[128];
    for (byte relay = 0; relay < 128; relay++)
    {
        on_cmd[relay] = (String(RELAY_CHAR) + String(DELIM) + String(relay+1) + String(DELIM) + String(ON_CHAR) + String(DELIM) + String(END_CHAR));
        off_cmd[relay] = (String(RELAY_CHAR) + String(DELIM) + String(relay+1) + String(DELIM) + String(OFF_CHAR) + String(DELIM) + String(END_CHAR));
    }

    // test slot 1-8 relays - slave board
    for (byte slot = SLAVE_SLOT_START - 1; slot < SLAVE_SLOT_END; slot++)
    {
        if (ioExp1_U2.digitalRead1(SLAVE_DET_PINS[slot]))
        {
            Serial.print('S');
            Serial.print(slot);
            Serial.print(':');
            Serial.println(ACK_STR);

            digitalWrite(SPI_CS_PINS[slot], LOW);
            delay(100);

            for (byte relay = SLAVE_RELAY_START - 1; relay < SLAVE_RELAY_END; relay++)
            {
                // send relay ON command
                const char * on_cmd_ptr = on_cmd[relay].c_str();
                for (byte i = 0; i <= strlen(on_cmd_ptr); i++)
                {
                #ifdef DEBUG
                    Serial.println(on_cmd_ptr[i]); // Print latest data sent to SPI slave
                #endif
                    SPI.transfer(on_cmd_ptr[i]);
                    delayMicroseconds(1000); // play with this parameter - for SPI delay
                }

                delay(TEST_DELAY_MSEC);

                // send relay OFF command
                const char * off_cmd_ptr = off_cmd[relay].c_str();
                for (byte i = 0; i <= strlen(off_cmd_ptr); i++)
                {
                #ifdef DEBUG
                    Serial.println(off_cmd_ptr[i]); // Print latest data sent to SPI slave
                #endif
                    SPI.transfer(off_cmd_ptr[i]);
                    delayMicroseconds(1000); // play with this parameter - for SPI delay
                }

                delay(TEST_DELAY_MSEC);
            }

            delay(100);
            digitalWrite(SPI_CS_PINS[slot], HIGH);
        }
        else
        {
            Serial.print('S');
            Serial.print(slot);
            Serial.print(':');
            Serial.println(NACK_STR);
        }
    }
#endif

    digitalWrite(DIAG_LED_PIN, LOW);
}

#ifdef VERBOSE_REPLY
void printReply(const char * reply)
{
    Serial.println(reply);
}
#else
void printReply(const char * /*reply*/) {};
#endif

void relayWriteWrapper(byte pinNum, bool state)
{
    if (pinNum <= 7)
    {
        ioExp1_U3.digitalWrite0(pinNum, state);
    }
    else
    {
        ioExp1_U3.digitalWrite1(pinNum-8, state);
    }
}

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

    Serial.print("SMU Relay (Master)");
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

    ioExp1_U2.portMode1(ALLINPUT);

    //Master board relay pins, only for Master 1
    ioExp1_U3.portMode0(ALLOUTPUT);
    ioExp1_U3.portMode1(ALLOUTPUT);
    ioExp1_U3.digitalWritePort0(0);
    ioExp1_U3.digitalWritePort1(0);

    pinMode(DIAG_LED_PIN, OUTPUT);
}

void loop() {
    // put your main code here, to run repeatedly:
#ifdef RUN_RELAY_TEST
    runRelayTestLoop();
#else
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
                    if (cmd_str[delim_idx[0]+1] == RELAYGRP_CHAR)
                    {
                        //replace R with Q for RELAYGRP first byte
                        cmd_str[delim_idx[1]+1] = RELAYGRP_CHAR;

                        for (byte slot = 0; slot < MAX_SLAVE_BOARD; slot++)
                        {
                        #ifdef SPI_2X_HACK_GRPCMD
                            //--------------------------------------------------
                            //HACK HACK HACK - send every command to slave twice
                            //--------------------------------------------------
                            for (byte j = 0; j < 2; j++)
                            {
                        #endif
                                digitalWrite(SPI_CS_PINS[slot], LOW);
                                delay(100);

                                //start sending to SPI lines begining of RELAY request type
                            #ifdef SPI_TRANSFER_BUFFER
                                int start = delim_idx[1]+1;
                                int end = cmd_idx-start+1;
                                SPI.transfer(&cmd_str[start], end);
                            #else
                                for (int i = delim_idx[1]+1; i <= cmd_idx; i++)
                                {
                                    SPI.transfer(cmd_str[i]);
                                    delayMicroseconds(1000); // play with this parameter
                                }
                            #endif

                                delay(100);
                                digitalWrite(SPI_CS_PINS[slot], HIGH);
                        #ifdef SPI_2X_HACK_GRPCMD
                            }
                        #endif
                            printReply(ACK_STR);
                        }
                    }
                    else
                    {
                        //check for SLOT number
                        int SLOT_num = atoi(&cmd_str[delim_idx[0]+1]);

                        if ( (SLOT_num >= 1) && (SLOT_num <= 8) ) //SLOT 1-8 are external relays
                        {
                        #ifdef SPI_2X_HACK_GRPCMD
                            //--------------------------------------------------
                            //HACK HACK HACK - send every command to slave twice
                            //--------------------------------------------------
                            for (byte j = 0; j < 2; j++)
                            {
                        #endif
                                digitalWrite(SPI_CS_PINS[SLOT_num-1], LOW);
                                delay(100);

                                //start sending to SPI lines begining of RELAY request type
                            #ifdef SPI_TRANSFER_BUFFER
                                int start = delim_idx[1]+1;
                                int end = cmd_idx-start+1;
                                SPI.transfer(&cmd_str[start], end);
                            #else
                                for (int i = delim_idx[1]+1; i <= cmd_idx; i++)
                                {
                                    SPI.transfer(cmd_str[i]);
                                    delayMicroseconds(1000); // play with this parameter
                                }
                            #endif

                                delay(100);
                                digitalWrite(SPI_CS_PINS[SLOT_num-1], HIGH);
                        #ifdef SPI_2X_HACK_GRPCMD
                            }
                        #endif
                            printReply(ACK_STR);
                        }
                        else if (SLOT_num == 0) //SLOT 0 is for local relay
                        {
                            //parse next data for RELAY request type
                            if (cmd_str[delim_idx[1]+1] == RELAY_CHAR)
                            {
                                if (cmd_str[delim_idx[2]+1] == RESET_CHAR)
                                {
                                    //check for ON/OFF
                                    if (cmd_str[delim_idx[3]+1] == ON_CHAR)
                                    {
                                        //Serial.println("Received RELAY X ON command. Please wait...");
                                        for (int i = 0; i < 16; i++)
                                        {
                                            relayWriteWrapper(i, true);
                                        }
                                        //Serial.println("RELAY X ON command done.");
                                        printReply(ACK_STR);
                                    }
                                    else if (cmd_str[delim_idx[3]+1] == OFF_CHAR)
                                    {
                                        //Serial.println("Received RELAY X OFF command. Please wait...");
                                        for (int i = 0; i < 16; i++)
                                        {
                                            relayWriteWrapper(i, false);
                                        }
                                        //Serial.println("RELAY X OFF command done.");
                                        printReply(ACK_STR);
                                    }
                                    else if (cmd_str[delim_idx[3]+1] == RESET_CHAR)
                                    {
                                        //Serial.println("Received RELAY X ON <1sec> OFF command. Please wait...");
                                        for (int i = 0; i < 16; i++)
                                        {
                                            relayWriteWrapper(i, true);
                                            delay(1000);
                                            relayWriteWrapper(i, false);
                                        }
                                        //Serial.println("RELAY X ON <1sec> OFF command done.");
                                        printReply(ACK_STR);
                                    }
                                    else if (cmd_str[delim_idx[3]+1] == FRESET_CHAR)
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
                                else
                                {
                                    if (delim_count == 5)
                                    {
                                        //check for RELAY number
                                        int RELAY_num = atoi(&cmd_str[delim_idx[2]+1]);
        
                                        if ( (RELAY_num >= 1) && (RELAY_num <= 16) ) // only supports RELAY 1-16
                                        {
                                            //check for ON/OFF
                                            if (cmd_str[delim_idx[3]+1] == ON_CHAR)
                                            {
                                                relayWriteWrapper(RELAY_num-1, true);
                                                //Serial.println("Received RELAY:ON command");
                                                printReply(ACK_STR);
                                            }
                                            else if (cmd_str[delim_idx[3]+1] == OFF_CHAR)
                                            {
                                                relayWriteWrapper(RELAY_num-1, false);
                                                //Serial.println("Received RELAY:OFF command");
                                                printReply(ACK_STR);
                                            }
                                            else
                                            {
                                                //Serial.println("ERROR: unknown RELAY command");
                                                printReply(NACK_STR);
                                            }
                                        }
                                        else
                                        {
                                            //Serial.println("ERROR: unknown RELAY");
                                            printReply(NACK_STR);
                                        }
                                    }
                                    else if (delim_count == 9)
                                    {
                                        //check for RELAY numbers
                                        bool is_err = false;
                                        int RELAY_num[4] = {};
                                        int total_relays = 0;
                                        for (int i = 0; i < 4; i++)
                                        {
                                            int tmp = atoi(&cmd_str[delim_idx[i+2]+1]);
                                            if ( (tmp >= 1) && (tmp <= 16) ) // only supports RELAY 1-16
                                            {
                                                RELAY_num[total_relays] = tmp;
                                                total_relays++;
                                            }
                                            else if (tmp != 0) //skip if 0
                                            {
                                                //Serial.println("ERROR: unknown RELAY");
                                                printReply(NACK_STR);
                                                is_err = true;
                                            }
                                        }
                                        
                                        //check for ON/OFF
                                        if ( (!is_err) && (cmd_str[delim_idx[6]+1] == ON_CHAR) )
                                        {
                                            //loop through all RELAY numbers to ON
                                            for (int i = 0; i < total_relays; i++)
                                            {
                                                relayWriteWrapper(RELAY_num[i]-1, true);
                                                //Serial.println("Received RELAY:ON command");
                                            }
                                        }
                                        else if ( (!is_err) && (cmd_str[delim_idx[6]+1] == OFF_CHAR) )
                                        {
                                            //loop through all RELAY numbers to OFF
                                            for (int i = 0; i < total_relays; i++)
                                            {
                                                relayWriteWrapper(RELAY_num[i]-1, false);
                                                //Serial.println("Received RELAY:OFF command");
                                            }
                                        }
                                        else
                                        {
                                            //Serial.println("ERROR: unknown RELAY command");
                                            is_err = true;
                                            printReply(NACK_STR);
                                        }
                                        
                                        //check for duration
                                        int duration = atoi(&cmd_str[delim_idx[7]+1]);
                                        
                                        if ( (duration >= 1) && (duration <= 5) ) // only supports duration 1-5
                                        {
                                            //delay following the specified duration
                                            delay(DURATION_MS[duration-1]);
                                            
                                            //check for ON/OFF
                                            if ( (!is_err) && (cmd_str[delim_idx[6]+1] == ON_CHAR) )
                                            {
                                                //loop through all RELAY numbers to OFF - inverse control
                                                for (int i = 0; i < total_relays; i++)
                                                {
                                                    relayWriteWrapper(RELAY_num[i]-1, false);
                                                    //Serial.println("Received RELAY:OFF command");
                                                }
                                            }
                                            else if ( (!is_err) && (cmd_str[delim_idx[6]+1] == OFF_CHAR) )
                                            {
                                                //loop through all RELAY numbers to ON - inverse control
                                                for (int i = 0; i < total_relays; i++)
                                                {
                                                    relayWriteWrapper(RELAY_num[i]-1, true);
                                                    //Serial.println("Received RELAY:ON command");
                                                }
                                            }
                                            else
                                            {
                                                //Serial.println("ERROR: unknown RELAY command");
                                                is_err = true;
                                                printReply(NACK_STR);
                                            }
                                        }
                                        else if (duration != 0) // skip if 0 - no delay/indefinite
                                        {
                                            //Serial.println("ERROR: unknown RELAY command");
                                            is_err = true;
                                            printReply(NACK_STR);
                                        }
                                        
                                        if (!is_err)
                                        {
                                            printReply(ACK_STR);
                                        }
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
#endif
}
