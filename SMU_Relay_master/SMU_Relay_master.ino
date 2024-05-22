// SMU Relay (Master)
// Rev 1.0 (18/04/2024)
// - Maxtrax

#include <Wire.h>
#include <SPI.h>
#include <PCA9540BD.h>
#include <DTIOI2CtoParallelConverter.h>

//#define DEBUG // uncomment this line to print debug data to the serial bus

#define SPI_TRANSFER_CLOCK_FREQ_12 12000000
#define SPI_TRANSFER_CLOCK_FREQ SPI_TRANSFER_CLOCK_FREQ_12

const char * app_ver = "v1.0";

const char * ACK_STR = "ACK";
const char * NACK_STR = "NACK";
const char SLOT_CHAR = 'S';
const char END_CHAR = 'E';
const char RELAY_CHAR = 'R';
const char ON_CHAR = '1';
const char OFF_CHAR = '0';
const char DELIM = ',';

const byte FAULT_LED_PIN = 15; //Red
const byte DIAG_LED_PIN = 16; //Green

const byte SPI_CS1_PIN = 0;
const byte SPI_CS2_PIN = 1;
const byte SPI_CS3_PIN = 2;
const byte SPI_CS4_PIN = 3;
const byte SPI_CS5_PIN = 4;
const byte SPI_CS6_PIN = 5;
const byte SPI_CS7_PIN = 6;
const byte SPI_CS8_PIN = 7;
const byte TOTAL_CS_PINS = 8;

const int MAX_BUFFERED_CMD = 32;

PCA9540BD multiplexer_U1; //PCA9540BD Mux (0x70)
DTIOI2CtoParallelConverter ioExp1_U2(0x74);  //PCA9555 I/O Expander (with A1 = 0 and A0 = 0)
DTIOI2CtoParallelConverter ioExp1_U3(0x75);  //PCA9555 I/O Expander (with A1 = 0 and A0 = 1)

SPISettings settings = SPISettings(SPI_TRANSFER_CLOCK_FREQ, MSBFIRST, SPI_MODE0);

bool first_delim_found = false;
bool second_delim_found = false;
bool third_delim_found = false;
bool fourth_delim_found = false;
bool fifth_delim_found = false;
int delim1_idx = 0;
int delim2_idx = 0;
int delim3_idx = 0;
int delim4_idx = 0;
int delim5_idx = 0;
int cmd_idx = 0;
char cmd_str[MAX_BUFFERED_CMD] = {};

byte SPI_CS_PINS[TOTAL_CS_PINS]= {
    SPI_CS1_PIN,
    SPI_CS2_PIN,
    SPI_CS3_PIN,
    SPI_CS4_PIN,
    SPI_CS5_PIN,
    SPI_CS6_PIN,
    SPI_CS7_PIN,
    SPI_CS8_PIN
};

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
    delim1_idx = 0;
    delim2_idx = 0;
    delim3_idx = 0;
    delim4_idx = 0;
    delim5_idx = 0;
    first_delim_found = false;
    second_delim_found = false;
    third_delim_found = false;
    fourth_delim_found = false;
    fifth_delim_found = false;
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
    
    for (byte i = 0; i < TOTAL_CS_PINS; i++)
    {
        digitalWrite(SPI_CS_PINS[i], HIGH);
    }

    multiplexer_U1.selectChannel(1); // for selecting ioExp1 channel, will only be using this channel

    ioExp1_U2.portMode1(ALLINPUT);

    //Master board relay pins, only for Master 1
    ioExp1_U3.portMode0(ALLOUTPUT);
    ioExp1_U3.portMode1(ALLOUTPUT);
    ioExp1_U3.digitalWritePort0(0);
    ioExp1_U3.digitalWritePort1(0);
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
                //1st delimiter found
                if (!first_delim_found)
                {
                    delim1_idx = cmd_idx;
                    first_delim_found = true;
                }
                //2nd delimiter found
                else if (first_delim_found && !second_delim_found)
                {
                    delim2_idx = cmd_idx;
                    second_delim_found = true;
                }
                //3rd delimiter found
                else if (second_delim_found && !third_delim_found)
                {
                    delim3_idx = cmd_idx;
                    third_delim_found = true;
                }
                //4th delimiter found
                else if (third_delim_found && !fourth_delim_found)
                {
                    delim4_idx = cmd_idx;
                    fourth_delim_found = true;
                }
                //5th delimiter found
                else
                {
                    delim5_idx = cmd_idx;
                    fifth_delim_found = true;
                }
            }

            //5th delimiter found and at least 1 byte arrived after 5th delimiter
            if ( (fifth_delim_found) && (cmd_idx >= delim5_idx + 1) )
            {
                //check for END
                if (cmd_str[delim5_idx+1] == END_CHAR)
                {
                    //parse first data for request type
                    if (cmd_str[0] == SLOT_CHAR)
                    {
                        //check for SLOT number
                        int SLOT_num = atoi(&cmd_str[delim1_idx+1]);

                        if ( (SLOT_num >= 1) && (SLOT_num <= 8) ) //SLOT 1-8 are external relays
                        {
                            SPI.beginTransaction(settings);
                            digitalWrite(SPI_CS_PINS[SLOT_num-1], LOW);

                            //start sending to SPI lines begining of RELAY request type
                            for(int i = delim2_idx+1; i <= cmd_idx; i++)
                            {
                            #ifdef DEBUG
                                Serial.println(cmd_str[i]); // Print latest data sent to SPI slave
                            #endif
                                SPI.transfer(cmd_str[i]);
                                delayMicroseconds(1000); // play with this parameter
                            }

                            digitalWrite(SPI_CS_PINS[SLOT_num-1], HIGH);
                            SPI.endTransaction();

                            Serial.print(ACK_STR);
                        }
                        else if (SLOT_num == 0) //SLOT 0 is for local relay
                        {
                            //parse next data for RELAY request type
                            if (cmd_str[delim2_idx+1] == RELAY_CHAR)
                            {
                                //check for RELAY number
                                int RELAY_num = atoi(&cmd_str[delim3_idx+1]);
                                
                                if ( (RELAY_num >= 1) && (RELAY_num <= 16) ) // only supports RELAY 1-16
                                {
                                    //check for ON/OFF
                                    if (cmd_str[delim4_idx+1] == ON_CHAR)
                                    {
                                        relayWriteWrapper(RELAY_num-1, true);
                                        //Serial.println("Received RELAY:ON command");
                                        Serial.print(ACK_STR);
                                    }
                                    else if (cmd_str[delim4_idx+1] == OFF_CHAR)
                                    {
                                        relayWriteWrapper(RELAY_num-1, false);
                                        //Serial.println("Received RELAY:OFF command");
                                        Serial.print(ACK_STR);
                                    }
                                    else
                                    {
                                        //Serial.println("ERROR: unknown RELAY command");
                                        Serial.print(NACK_STR);
                                    }
                                }
                                else
                                {
                                    //Serial.println("ERROR: unknown RELAY");
                                    Serial.print(NACK_STR);
                                }
                            }
                            else
                            {
                                //Serial.println("ERROR: unknown RELAY request");
                                Serial.print(NACK_STR);
                            }
                        }
                        else
                        {
                            //Serial.println("ERROR: unknown SLOT");
                            Serial.print(NACK_STR);
                        }
                    }
                    else
                    {
                        //Serial.println("ERROR: unknown SLOT request");
                        Serial.print(NACK_STR);
                    }
                    
                    //reset index and clear buffer when done processing
                    resetBuffer();
                    cmd_idx = -1;
                }
            }
    
            cmd_idx++;
            
            if (cmd_idx >= MAX_BUFFERED_CMD) //buffer overflowing!
            {
                //Serial.println("ERROR: command buffer overflowing");
                Serial.print(NACK_STR);
                resetBuffer();
                cmd_idx = 0;
            }
        }
    }
}
