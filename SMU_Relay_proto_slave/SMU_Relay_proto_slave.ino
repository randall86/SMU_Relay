// SMU Relay proto board (Slave)
// Rev 1.0 (31/12/2023)
// - Maxtrax

#include <SercomSPISlave.h>
#include <DTIOI2CtoParallelConverter.h>

//#define DEBUG // uncomment this line to print debug data to the serial bus
#define INTERRUPT2BUFFER // uncomment this line to copy the data received in the Data Received Complete interrupt to a buffer to be used in the main loop
//#define INTERRUPT2SERIAL // uncomment this line to print the data to the serial bus whenever the Data Received Complete interrupt is triggered

const char * app_ver = "v1.0";

const char * END_STR = "END";
const char * LED_STR = "LED";
const char * ON_STR = "ON";
const char * OFF_STR = "OFF";
const char DELIM = ',';
const byte MIN_CMD_LEN = 2;

const byte LED1_PIN = PIN0_0;
const byte LED2_PIN = PIN0_1;
const byte LED3_PIN = PIN0_2;
const byte LED4_PIN = PIN0_3;

const int MAX_INT_DATA = 3;
const int MAX_BUFFERED_CMD = 32;
const int MAX_READ_BUFFER = 128;

Sercom0SPISlave SPISlave; // to use a different SERCOM, change this line and find and replace all SERCOM0 with the SERCOM of your choice
DTIOI2CtoParallelConverter ioExp1_U5(0x74);  //PCA9555 I/O Expander (with A1 = 0 and A0 = 0)

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

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    //while (!Serial); // wait for serial port to connect. Needed for Native USB only

    Serial.print("SMU Relay proto board (Slave)");
    Serial.println(app_ver);
    
    SPISlave.SercomInit(SPISlave.MOSI_Pins::PA04, SPISlave.SCK_Pins::PA05, SPISlave.SS_Pins::PA06, SPISlave.MISO_Pins::PA07);
    Wire.begin(); //need to start the Wire for I2C devices to function
    
    ioExp1_U5.portMode0(ALLOUTPUT);
    ioExp1_U5.digitalWrite0(LED1_PIN, LOW);
    ioExp1_U5.digitalWrite0(LED2_PIN, LOW);
    ioExp1_U5.digitalWrite0(LED3_PIN, LOW);
    ioExp1_U5.digitalWrite0(LED4_PIN, LOW);
}

void loop() {
    // put your main code here, to run repeatedly:
    //get request from Master
    while (read_idx < recv_idx)
    {
        #ifdef DEBUG
        Serial.println(read_buffer[read_idx]); // Print latest data written into the buffer by the interrupt
        #endif
        
        char tmp_char = read_buffer[read_idx];
        read_idx++;

        //wraparound handling when done reading all received buffer or buffer overflow
        if ( (read_idx == recv_idx) || (read_idx >= MAX_READ_BUFFER) )
        {
            recv_idx = 0;
            read_idx = 0;
        }

        //sanitize and remove unwanted characters (ctrl/special char and SPACE)
        if (('!' <= tmp_char) && (tmp_char <= '~'))
        {
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
                else
                {
                    delim3_idx = cmd_idx;
                    third_delim_found = true;
                }
            }
            
            //3nd delimiter found and at least 3 bytes arrived after 3nd delimiter
            if ( (third_delim_found) && (cmd_idx >= delim3_idx + 3) )
            {
                //check for END
                if (strncmp(&cmd_str[delim3_idx+1], END_STR, 3) == 0)
                {
                    //parse first 3 bytes of data for request type
                    if (strncmp(&cmd_str[0], LED_STR, 3) == 0)
                    {
                        //check for LED number
                        int LED_num = atoi(&cmd_str[delim1_idx+1]);
                        
                        if ( (LED_num >= 1) && (LED_num <= 4) ) // only supports LED 1,2,3,4
                        {
                            //check for ON/OFF
                            if (strncmp(&cmd_str[delim2_idx+1], ON_STR, 2) == 0)
                            {
                                ioExp1_U5.digitalWrite0((LED_num-1), HIGH);
                                Serial.println("Received LED ON command");
                            }
                            else if (strncmp(&cmd_str[delim2_idx+1], OFF_STR, 3) == 0)
                            {
                                ioExp1_U5.digitalWrite0((LED_num-1), LOW);
                                Serial.println("Received LED OFF command");
                            }
                            else
                            {
                                Serial.println("ERROR: unknown LED command");
                            }
                        }
                        else
                        {
                            Serial.println("ERROR: unknown LED");
                        }
                    }
                    else
                    {
                        Serial.println("ERROR: unknown command");
                    }
                    
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
    Serial.println("In SPI Interrupt");
    #endif
    uint8_t data = 0;
    data = (uint8_t)SERCOM0->SPI.DATA.reg;
    uint8_t interrupts = SERCOM0->SPI.INTFLAG.reg; // Read SPI interrupt register
    #ifdef DEBUG
    Serial.print("Interrupt: "); Serial.println(interrupts);
    #endif

    // Slave Select Low interrupt
    if (interrupts & (1 << 3)) // 1000 = bit 3 = SSL // page 503
    {
        #ifdef DEBUG
        Serial.println("SPI Slave Select Low interupt");
        #endif
        SERCOM0->SPI.INTFLAG.bit.SSL = 1; // Clear Slave Select Low interrupt
    }

    // Data Received Complete interrupt: this is where the data is received, which is used in the main loop
    if (interrupts & (1 << 2)) // 0100 = bit 2 = RXC // page 503
    {
        #ifdef DEBUG
        Serial.println("SPI Data Received Complete interrupt");
        #endif
        data = SERCOM0->SPI.DATA.reg; // Read data register
        SERCOM0->SPI.INTFLAG.bit.RXC = 1; // Clear Receive Complete interrupt
    }

    // Data Transmit Complete interrupt
    if (interrupts & (1 << 1)) // 0010 = bit 1 = TXC // page 503
    {
        #ifdef DEBUG
        Serial.println("SPI Data Transmit Complete interrupt");
        #endif
        SERCOM0->SPI.INTFLAG.bit.TXC = 1; // Clear Transmit Complete interrupt
    }

    // Data Register Empty interrupt
    if (interrupts & (1 << 0)) // 0001 = bit 0 = DRE // page 503
    {
        #ifdef DEBUG
        Serial.println("SPI Data Register Empty interrupt");
        #endif
        SERCOM0->SPI.DATA.reg = 0xAA;
    }

    #ifdef INTERRUPT2BUFFER
    // Write data to buffer, to be used in main loop
    int_buffer[int_idx++] = data;
    if (int_idx >= MAX_INT_DATA)
    {
        // Extra handling to discard spurious data read from the register
        // For each received data on the SPI line is equivalent to 3 reg read
        int_idx = 0;
        read_buffer[recv_idx++] = int_buffer[2]; // Only index 2 data is valid
        //Serial.print("int_buffer[0]: ");
        //Serial.println(int_buffer[0]);
        //Serial.print("int_buffer[1]: ");
        //Serial.println(int_buffer[1]);
        //Serial.print("int_buffer[2]: ");
        //Serial.println(int_buffer[2]);
    }
    #endif
    #ifdef INTERRUPT2SERIAL
    // Print data received during the Data Receive Complete interrupt
    char _data = data;
    Serial.print("DATA: ");
    Serial.println(_data); // Print received data
    #endif
}
