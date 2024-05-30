// SMU Relay (Slave)
// Rev 1.0 (18/04/2024)
// - Maxtrax

#include <Wire.h>
#include <PCA9540BD.h>
#include <SercomSPISlave.h>
#include <DTIOI2CtoParallelConverter.h>

#define NOP __asm__("nop\n\t") //"nop" executes in one machine cycle (at 16 MHz) yielding a 62.5 ns delay

#define DEBUG // uncomment this line to print debug data to the serial bus
#define INTERRUPT2BUFFER // uncomment this line to copy the data received in the Data Received Complete interrupt to a buffer to be used in the main loop
//#define INTERRUPT2SERIAL // uncomment this line to print the data to the serial bus whenever the Data Received Complete interrupt is triggered

const char * app_ver = "v2.0";

const char END_CHAR = 'E';
const char RELAY_CHAR = 'R';
const char ON_CHAR = '1';
const char OFF_CHAR = '0';
const char DELIM = ',';

const byte STATUS_LED_PIN = 3; //Green 
const byte DIAG_LED_PIN = 2; //Red

const byte IOEXP0_RESET_PIN = 4;
const byte IOEXP1_RESET_PIN = 5;

const int MAX_INT_DATA = 3;
const int MAX_BUFFERED_CMD = 32;
const int MAX_READ_BUFFER = 128;

typedef struct _relay_map_t
{
    DTIOI2CtoParallelConverter* ioExpPtr;
    byte muxNum;
    byte pinNum;
}relay_map_t;

PCA9540BD multiplexer_U1; //PCA9540BD Mux (0x70)
Sercom0SPISlave SPISlave; // to use a different SERCOM, change this line and find and replace all SERCOM0 with the SERCOM of your choice
//multiplex channel 0
DTIOI2CtoParallelConverter ioExp0_U3(0x74);  //PCA9555 I/O Expander (with A1 = 0 and A0 = 0)
DTIOI2CtoParallelConverter ioExp0_U4(0x75);  //PCA9555 I/O Expander (with A1 = 0 and A0 = 1)
DTIOI2CtoParallelConverter ioExp0_U5(0x76);  //PCA9555 I/O Expander (with A1 = 1 and A0 = 0)
DTIOI2CtoParallelConverter ioExp0_U6(0x77);  //PCA9555 I/O Expander (with A1 = 1 and A0 = 1)
//multiplex channel 1
DTIOI2CtoParallelConverter ioExp1_U3(0x74);  //PCA9555 I/O Expander (with A1 = 0 and A0 = 0)
DTIOI2CtoParallelConverter ioExp1_U4(0x75);  //PCA9555 I/O Expander (with A1 = 0 and A0 = 1)
DTIOI2CtoParallelConverter ioExp1_U5(0x76);  //PCA9555 I/O Expander (with A1 = 1 and A0 = 0)
DTIOI2CtoParallelConverter ioExp1_U6(0x77);  //PCA9555 I/O Expander (with A1 = 1 and A0 = 1)

bool first_delim_found = false;
bool second_delim_found = false;
bool third_delim_found = false;
int delim1_idx = 0;
int delim2_idx = 0;
int delim3_idx = 0;
int cmd_idx = 0;
int int_idx = 0;
int recv_idx = 0;
int read_idx = 0;
char cmd_str[MAX_BUFFERED_CMD] = {};
char read_buffer[MAX_READ_BUFFER] = {};
char int_buffer[MAX_INT_DATA] = {};
byte LED_status = 0;

relay_map_t relay_map[128] = {
    //multiplex channel 0
    {&ioExp0_U3, 0, 0},
    {&ioExp0_U3, 0, 1},
    {&ioExp0_U3, 0, 2},
    {&ioExp0_U3, 0, 3},
    {&ioExp0_U3, 0, 4},
    {&ioExp0_U3, 0, 5},
    {&ioExp0_U3, 0, 6},
    {&ioExp0_U3, 0, 7},
    {&ioExp0_U3, 0, 8},
    {&ioExp0_U3, 0, 9},
    {&ioExp0_U3, 0, 10},
    {&ioExp0_U3, 0, 11},
    {&ioExp0_U3, 0, 12},
    {&ioExp0_U3, 0, 13},
    {&ioExp0_U3, 0, 14},
    {&ioExp0_U3, 0, 15},
    {&ioExp0_U4, 0, 0},
    {&ioExp0_U4, 0, 1},
    {&ioExp0_U4, 0, 2},
    {&ioExp0_U4, 0, 3},
    {&ioExp0_U4, 0, 4},
    {&ioExp0_U4, 0, 5},
    {&ioExp0_U4, 0, 6},
    {&ioExp0_U4, 0, 7},
    {&ioExp0_U4, 0, 8},
    {&ioExp0_U4, 0, 9},
    {&ioExp0_U4, 0, 10},
    {&ioExp0_U4, 0, 11},
    {&ioExp0_U4, 0, 12},
    {&ioExp0_U4, 0, 13},
    {&ioExp0_U4, 0, 14},
    {&ioExp0_U4, 0, 15},
    {&ioExp0_U5, 0, 0},
    {&ioExp0_U5, 0, 1},
    {&ioExp0_U5, 0, 2},
    {&ioExp0_U5, 0, 3},
    {&ioExp0_U5, 0, 4},
    {&ioExp0_U5, 0, 5},
    {&ioExp0_U5, 0, 6},
    {&ioExp0_U5, 0, 7},
    {&ioExp0_U5, 0, 8},
    {&ioExp0_U5, 0, 9},
    {&ioExp0_U5, 0, 10},
    {&ioExp0_U5, 0, 11},
    {&ioExp0_U5, 0, 12},
    {&ioExp0_U5, 0, 13},
    {&ioExp0_U5, 0, 14},
    {&ioExp0_U5, 0, 15},
    {&ioExp0_U6, 0, 0},
    {&ioExp0_U6, 0, 1},
    {&ioExp0_U6, 0, 2},
    {&ioExp0_U6, 0, 3},
    {&ioExp0_U6, 0, 4},
    {&ioExp0_U6, 0, 5},
    {&ioExp0_U6, 0, 6},
    {&ioExp0_U6, 0, 7},
    {&ioExp0_U6, 0, 8},
    {&ioExp0_U6, 0, 9},
    {&ioExp0_U6, 0, 10},
    {&ioExp0_U6, 0, 11},
    {&ioExp0_U6, 0, 12},
    {&ioExp0_U6, 0, 13},
    {&ioExp0_U6, 0, 14},
    {&ioExp0_U6, 0, 15},
    //multiplex channel 1
    {&ioExp1_U3, 1, 0},
    {&ioExp1_U3, 1, 1},
    {&ioExp1_U3, 1, 2},
    {&ioExp1_U3, 1, 3},
    {&ioExp1_U3, 1, 4},
    {&ioExp1_U3, 1, 5},
    {&ioExp1_U3, 1, 6},
    {&ioExp1_U3, 1, 7},
    {&ioExp1_U3, 1, 8},
    {&ioExp1_U3, 1, 9},
    {&ioExp1_U3, 1, 10},
    {&ioExp1_U3, 1, 11},
    {&ioExp1_U3, 1, 12},
    {&ioExp1_U3, 1, 13},
    {&ioExp1_U3, 1, 14},
    {&ioExp1_U3, 1, 15},
    {&ioExp1_U4, 1, 0},
    {&ioExp1_U4, 1, 1},
    {&ioExp1_U4, 1, 2},
    {&ioExp1_U4, 1, 3},
    {&ioExp1_U4, 1, 4},
    {&ioExp1_U4, 1, 5},
    {&ioExp1_U4, 1, 6},
    {&ioExp1_U4, 1, 7},
    {&ioExp1_U4, 1, 8},
    {&ioExp1_U4, 1, 9},
    {&ioExp1_U4, 1, 10},
    {&ioExp1_U4, 1, 11},
    {&ioExp1_U4, 1, 12},
    {&ioExp1_U4, 1, 13},
    {&ioExp1_U4, 1, 14},
    {&ioExp1_U4, 1, 15},
    {&ioExp1_U5, 1, 0},
    {&ioExp1_U5, 1, 1},
    {&ioExp1_U5, 1, 2},
    {&ioExp1_U5, 1, 3},
    {&ioExp1_U5, 1, 4},
    {&ioExp1_U5, 1, 5},
    {&ioExp1_U5, 1, 6},
    {&ioExp1_U5, 1, 7},
    {&ioExp1_U5, 1, 8},
    {&ioExp1_U5, 1, 9},
    {&ioExp1_U5, 1, 10},
    {&ioExp1_U5, 1, 11},
    {&ioExp1_U5, 1, 12},
    {&ioExp1_U5, 1, 13},
    {&ioExp1_U5, 1, 14},
    {&ioExp1_U5, 1, 15},
    {&ioExp1_U6, 1, 0},
    {&ioExp1_U6, 1, 1},
    {&ioExp1_U6, 1, 2},
    {&ioExp1_U6, 1, 3},
    {&ioExp1_U6, 1, 4},
    {&ioExp1_U6, 1, 5},
    {&ioExp1_U6, 1, 6},
    {&ioExp1_U6, 1, 7},
    {&ioExp1_U6, 1, 8},
    {&ioExp1_U6, 1, 9},
    {&ioExp1_U6, 1, 10},
    {&ioExp1_U6, 1, 11},
    {&ioExp1_U6, 1, 12},
    {&ioExp1_U6, 1, 13},
    {&ioExp1_U6, 1, 14},
    {&ioExp1_U6, 1, 15}
};

void relayWriteWrapper(relay_map_t *p_relay, bool state)
{
    if (NULL != p_relay)
    {
        multiplexer_U1.selectChannel(p_relay->muxNum);
        
        if (p_relay->pinNum <= 7)
        {
            p_relay->ioExpPtr->digitalWrite0(p_relay->pinNum, state);
        }
        else
        {
            p_relay->ioExpPtr->digitalWrite1(p_relay->pinNum-8, state);
        }
    }

    digitalWrite(STATUS_LED_PIN, LOW);
}

void resetBuffer()
{
    delim1_idx = 0;
    delim2_idx = 0;
    delim3_idx = 0;
    first_delim_found = false;
    second_delim_found = false;
    third_delim_found = false;
    memset(cmd_str, 0, MAX_BUFFERED_CMD);
}

void resetIOExpanders()
{
    digitalWrite(IOEXP0_RESET_PIN, LOW);
    NOP; //60ns delay
    digitalWrite(IOEXP0_RESET_PIN, HIGH);
    delayMicroseconds(1); //only need 400ns

    digitalWrite(IOEXP1_RESET_PIN, LOW);
    NOP; //60ns delay
    digitalWrite(IOEXP1_RESET_PIN, HIGH);
    delayMicroseconds(1); //only need 400ns
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    //while (!Serial); // wait for serial port to connect. Needed for Native USB only

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);

    Serial.print("SMU Relay (Slave)");
    Serial.println(app_ver);

    SPISlave.SercomInit(SPISlave.MOSI_Pins::PA04, SPISlave.SCK_Pins::PA05, SPISlave.SS_Pins::PA06, SPISlave.MISO_Pins::PA07);
    Wire.begin(); //need to start the Wire for I2C devices to function

    pinMode(IOEXP0_RESET_PIN, OUTPUT);
    pinMode(IOEXP1_RESET_PIN, OUTPUT);
    resetIOExpanders();

    multiplexer_U1.selectChannel(0); // for selecting ioExp0 channel
    ioExp0_U3.portMode0(ALLOUTPUT);
    ioExp0_U3.portMode1(ALLOUTPUT);
    ioExp0_U3.digitalWritePort0(0);
    ioExp0_U3.digitalWritePort1(0);

    ioExp0_U4.portMode0(ALLOUTPUT);
    ioExp0_U4.portMode1(ALLOUTPUT);
    ioExp0_U4.digitalWritePort0(0);
    ioExp0_U4.digitalWritePort1(0);

    ioExp0_U5.portMode0(ALLOUTPUT);
    ioExp0_U5.portMode1(ALLOUTPUT);
    ioExp0_U5.digitalWritePort0(0);
    ioExp0_U5.digitalWritePort1(0);

    ioExp0_U6.portMode0(ALLOUTPUT);
    ioExp0_U6.portMode1(ALLOUTPUT);
    ioExp0_U6.digitalWritePort0(0);
    ioExp0_U6.digitalWritePort1(0);

    multiplexer_U1.selectChannel(1); // for selecting ioExp1 channel
    ioExp1_U3.portMode0(ALLOUTPUT);
    ioExp1_U3.portMode1(ALLOUTPUT);
    ioExp1_U3.digitalWritePort0(0);
    ioExp1_U3.digitalWritePort1(0);

    ioExp1_U4.portMode0(ALLOUTPUT);
    ioExp1_U4.portMode1(ALLOUTPUT);
    ioExp1_U4.digitalWritePort0(0);
    ioExp1_U4.digitalWritePort1(0);

    ioExp1_U5.portMode0(ALLOUTPUT);
    ioExp1_U5.portMode1(ALLOUTPUT);
    ioExp1_U5.digitalWritePort0(0);
    ioExp1_U5.digitalWritePort1(0);

    ioExp1_U6.portMode0(ALLOUTPUT);
    ioExp1_U6.portMode1(ALLOUTPUT);
    ioExp1_U6.digitalWritePort0(0);
    ioExp1_U6.digitalWritePort1(0);
}

void loop() {
    // put your main code here, to run repeatedly:
    //get request from Master
    //format: R,<1-128>,<1/0>,E
    while ((read_idx < recv_idx) || Serial.available())
    {
        //blink LED when there's activity
        LED_status ^= 1;
        digitalWrite(STATUS_LED_PIN, LED_status);

        char tmp_char = 0;

        if (Serial.available())
        {
            tmp_char = Serial.read();
        }
        else
        {
            tmp_char = read_buffer[read_idx];
            read_idx++;
        }

        #ifdef DEBUG
        Serial.print("Received: ");
        Serial.println(tmp_char); // Print latest data written into the buffer by the interrupt
        #endif

        //wraparound handling when done reading all received buffer or buffer overflow
        if ( (read_idx == recv_idx) || (read_idx >= MAX_READ_BUFFER) )
        {
            recv_idx = 0;
            read_idx = 0;
        }

        //sanitize and remove unwanted characters (ctrl/special char and SPACE)
        if (('!' <= tmp_char) && (tmp_char <= '~'))
        {
            if (tmp_char == RELAY_CHAR)
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
                else
                {
                    delim3_idx = cmd_idx;
                    third_delim_found = true;
                }
            }
            
            //3nd delimiter found and at least 1 byte arrived after 3nd delimiter
            if ( (third_delim_found) && (cmd_idx >= delim3_idx + 1) )
            {
                //check for END
                if (cmd_str[delim3_idx+1] == END_CHAR)
                {
                    //parse first data for request type
                    if (cmd_str[0] == RELAY_CHAR)
                    {
                        //check for RELAY number
                        int RELAY_num = atoi(&cmd_str[delim1_idx+1]);
                        
                        if ( (RELAY_num >= 1) && (RELAY_num <= 128) ) // only supports RELAY 1-128
                        {
                            //check for ON/OFF
                            if (cmd_str[delim2_idx+1] == ON_CHAR)
                            {
                                relayWriteWrapper(&relay_map[RELAY_num-1], true);
                                Serial.println("Received RELAY ON command");
                            }
                            else if (cmd_str[delim2_idx+1] == OFF_CHAR)
                            {
                                relayWriteWrapper(&relay_map[RELAY_num-1], false);
                                Serial.println("Received RELAY OFF command");
                            }
                            else
                            {
                                Serial.println("ERROR: unknown RELAY command");
                            }
                        }
                        else
                        {
                            Serial.println("ERROR: unknown RELAY");
                        }
                    }
                    else
                    {
                        Serial.println("ERROR: unknown RELAY request");
                    }
                    
                    //reset index and clear buffer when done processing
                    resetBuffer();
                    cmd_idx = -1;
                }
            }
    
            cmd_idx++;
            
            if (cmd_idx >= MAX_BUFFERED_CMD) //buffer overflowing!
            {
                Serial.println("ERROR: command buffer overflowing");
                resetBuffer();
                cmd_idx = 0;
            }
        }
    }

    #ifdef INTERRUPT2SERIAL
    delay(1000); // Delay of 1 s to keep the main loop running, while data is written to the serial every time the Data Received Interrupt is triggered
    #endif
}

void SERCOM0_Handler()
/*
Reference: Atmel-42181G-SAM-D21_Datasheet section 26.8.6 on page 503
*/
{
    #ifdef DEBUG
    //Serial.println("In SPI Interrupt");
    #endif
    uint8_t data = 0;
    data = (uint8_t)SERCOM0->SPI.DATA.reg;
    uint8_t interrupts = SERCOM0->SPI.INTFLAG.reg; // Read SPI interrupt register
    #ifdef DEBUG
    //Serial.print("Interrupt: "); Serial.println(interrupts);
    #endif

    // Slave Select Low interrupt
    if (interrupts & (1 << 3)) // 1000 = bit 3 = SSL // page 503
    {
        #ifdef DEBUG
        //Serial.println("SPI Slave Select Low interupt");
        #endif
        SERCOM0->SPI.INTFLAG.bit.SSL = 1; // Clear Slave Select Low interrupt
    }

    // Data Received Complete interrupt: this is where the data is received, which is used in the main loop
    if (interrupts & (1 << 2)) // 0100 = bit 2 = RXC // page 503
    {
        #ifdef DEBUG
        //Serial.println("SPI Data Received Complete interrupt");
        #endif
        data = SERCOM0->SPI.DATA.reg; // Read data register
        SERCOM0->SPI.INTFLAG.bit.RXC = 1; // Clear Receive Complete interrupt
    }

    // Data Transmit Complete interrupt
    if (interrupts & (1 << 1)) // 0010 = bit 1 = TXC // page 503
    {
        #ifdef DEBUG
        //Serial.println("SPI Data Transmit Complete interrupt");
        #endif
        SERCOM0->SPI.INTFLAG.bit.TXC = 1; // Clear Transmit Complete interrupt
    }

    // Data Register Empty interrupt
    if (interrupts & (1 << 0)) // 0001 = bit 0 = DRE // page 503
    {
        #ifdef DEBUG
        //Serial.println("SPI Data Register Empty interrupt");
        #endif
        SERCOM0->SPI.DATA.reg = 0xAA;
    }

    #ifdef INTERRUPT2BUFFER
    // Write data to buffer, to be used in main loop
    read_buffer[recv_idx++] = data;
    //int_buffer[int_idx++] = data;
    //if (int_idx >= MAX_INT_DATA)
    //{
    //    // Extra handling to discard spurious data read from the register
    //    // For each received data on the SPI line is equivalent to 3 reg read
    //    int_idx = 0;
    //    read_buffer[recv_idx++] = int_buffer[2]; // Only index 2 data is valid
    //    //Serial.print("int_buffer[0]: ");
    //    //Serial.println(int_buffer[0]);
    //    //Serial.print("int_buffer[1]: ");
    //    //Serial.println(int_buffer[1]);
    //    //Serial.print("int_buffer[2]: ");
    //    //Serial.println(int_buffer[2]);
    //}
    #endif
    #ifdef INTERRUPT2SERIAL
    // Print data received during the Data Receive Complete interrupt
    char _data = data;
    Serial.print("DATA: ");
    Serial.println(_data); // Print received data
    #endif
}
