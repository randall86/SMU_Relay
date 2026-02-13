// SMU Relay Tester (Master)
// Rev 2.0 (13/02/2026)
// - Maxtrax

#include <Bounce2.h>
#include <Wire.h>
#include <SPI.h>

//#define VERBOSE_REPLY //uncomment this for replying ACK/NACK/Debug logging
//define DEBUG //uncomment this line to print debug data to the serial bus

#define SPI_TRANSFER_BUFFER //for buffer transfer instead of byte by byte

#define SPI_TRANSFER_CLOCK_FREQ_100K 100000
#define SPI_TRANSFER_CLOCK_FREQ_500K 500000
#define SPI_TRANSFER_CLOCK_FREQ_2M 2000000
#define SPI_TRANSFER_CLOCK_FREQ_4M 4000000
#define SPI_TRANSFER_CLOCK_FREQ_8M 8000000
#define SPI_TRANSFER_CLOCK_FREQ_10M 10000000
#define SPI_TRANSFER_CLOCK_FREQ_12M 12000000

#define SPI_TRANSFER_CLOCK_FREQ SPI_TRANSFER_CLOCK_FREQ_100K

const char * app_ver = "v2.0";

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

const byte SLAVE_RESET_PIN = 6; 

const byte SPI_CS1_PIN = 0;

const byte EXT_RELAY_A = 2;
const byte EXT_RELAY_B = 3;
const byte EXT_RELAY_C = 4;
const byte EXT_RELAY_D = 5;
const byte GATE_A = 1;
const byte GATE_B = 7;
const byte GATE_C = 11;
const byte GATE_D = 12;

const byte SW2_PIN = A2;
const byte SW3_PIN = A3;
const byte SW4_PIN = A4;
const byte SW5_PIN = A5;
const byte SW6_PIN = A6;
const byte SW7_PIN = A1;

const int MAX_BUFFERED_CMD = 64;

const int MAX_DELIMS = 16;

const uint32_t DEBOUNCE_MSEC = 100; //stable time before registering state change

//supported on/off durations in ms
const int DURATION_MS[5] = {
    100,
    200,
    300,
    400,
    500
};

SPISettings settings = SPISettings(SPI_TRANSFER_CLOCK_FREQ, MSBFIRST, SPI_MODE0);

Bounce Sw2Debouncer = Bounce();
Bounce Sw3Debouncer = Bounce();
Bounce Sw4Debouncer = Bounce();
Bounce Sw5Debouncer = Bounce();
Bounce Sw6Debouncer = Bounce();
Bounce Sw7Debouncer = Bounce();

int delim_count = 0;
int delim_idx[MAX_DELIMS] = {};
int cmd_idx = 0;
char cmd_str[MAX_BUFFERED_CMD] = {};

char on_all_relays_cmd[] = {RELAY_CHAR, RESET_CHAR, '1', END_CHAR, '\0'};
char on_grpA_relays_cmd[] = {RELAYGRP_CHAR, 'A', '1', END_CHAR, '\0'};
char on_grpB_relays_cmd[] = {RELAYGRP_CHAR, 'B', '1', END_CHAR, '\0'};
char on_grpC_relays_cmd[] = {RELAYGRP_CHAR, 'C', '1', END_CHAR, '\0'};
char on_grpD_relays_cmd[] = {RELAYGRP_CHAR, 'D', '1', END_CHAR, '\0'};
char off_grpA_relays_cmd[] = {RELAYGRP_CHAR, 'A', '0', END_CHAR, '\0'};
char off_grpB_relays_cmd[] = {RELAYGRP_CHAR, 'B', '0', END_CHAR, '\0'};
char off_grpC_relays_cmd[] = {RELAYGRP_CHAR, 'C', '0', END_CHAR, '\0'};
char off_grpD_relays_cmd[] = {RELAYGRP_CHAR, 'D', '0', END_CHAR, '\0'};
char on_seq_relays_cmd[] = {RELAY_CHAR, RESET_CHAR, RESET_CHAR, END_CHAR, '\0'};

enum _EXT_RELAYS
{
    A_EXT_RELAYS = 0,
    B_EXT_RELAYS,
    C_EXT_RELAYS,
    D_EXT_RELAYS,
    MAX_EXT_RELAYS
};

byte EXT_RELAY_PINS[MAX_EXT_RELAYS+4] = {
    EXT_RELAY_A,
    EXT_RELAY_B,
    EXT_RELAY_C,
    EXT_RELAY_D,
    GATE_A,
    GATE_B,
    GATE_C,
    GATE_D
};

#ifdef VERBOSE_REPLY
void printReply(const char * reply)
{
    Serial.println(reply);
}
#else
void printReply(const char * /*reply*/) {};
#endif

void turnOnAllExtRelays()
{
    for (int i = 0; i < MAX_EXT_RELAYS; i++)
    {
        digitalWrite(EXT_RELAY_PINS[i], HIGH);
        digitalWrite(EXT_RELAY_PINS[i+4], HIGH);
    }
}

void turnOffAllExtRelays()
{
    for (int i = 0; i < MAX_EXT_RELAYS; i++)
    {
        digitalWrite(EXT_RELAY_PINS[i], LOW);
        digitalWrite(EXT_RELAY_PINS[i+4], LOW);
    }
}

void triggerSw2Routine()
{
    //Serial.println("Triggering SW2 button on");
    digitalWrite(EXT_RELAY_PINS[B_EXT_RELAYS], HIGH);
    digitalWrite(EXT_RELAY_PINS[B_EXT_RELAYS+4], HIGH);

    digitalWrite(SPI_CS1_PIN, LOW);
    delay(1);

    //start sending to SPI lines begining of RELAY request type
#ifdef SPI_TRANSFER_BUFFER
    SPI.transfer(on_grpB_relays_cmd, sizeof(on_grpB_relays_cmd));
#else
    for (int i = 0; i < sizeof(on_grpB_relays_cmd); i++)
    {
        SPI.transfer(on_grpB_relays_cmd[i]);
        delayMicroseconds(1000); // play with this parameter
    }
#endif

    digitalWrite(SPI_CS1_PIN, HIGH);
    delay(1);
}

void triggerSw2RoutineOff()
{
    //Serial.println("Triggering SW2 button on");
    digitalWrite(EXT_RELAY_PINS[B_EXT_RELAYS], LOW);
    digitalWrite(EXT_RELAY_PINS[B_EXT_RELAYS+4], LOW);

    digitalWrite(SPI_CS1_PIN, LOW);
    delay(1);

    //start sending to SPI lines begining of RELAY request type
#ifdef SPI_TRANSFER_BUFFER
    SPI.transfer(off_grpB_relays_cmd, sizeof(off_grpB_relays_cmd));
#else
    for (int i = 0; i < sizeof(off_grpB_relays_cmd); i++)
    {
        SPI.transfer(off_grpB_relays_cmd[i]);
        delayMicroseconds(1000); // play with this parameter
    }
#endif

    digitalWrite(SPI_CS1_PIN, HIGH);
    delay(1);
}

void triggerSw3Routine()
{
    //Serial.println("Triggering SW3 button on");
    digitalWrite(EXT_RELAY_PINS[D_EXT_RELAYS], HIGH);
    digitalWrite(EXT_RELAY_PINS[D_EXT_RELAYS+4], HIGH);

    digitalWrite(SPI_CS1_PIN, LOW);
    delay(1);

    //start sending to SPI lines begining of RELAY request type
#ifdef SPI_TRANSFER_BUFFER
    SPI.transfer(on_grpD_relays_cmd, sizeof(on_grpD_relays_cmd));
#else
    for (int i = 0; i < sizeof(on_grpD_relays_cmd); i++)
    {
        SPI.transfer(on_grpD_relays_cmd[i]);
        delayMicroseconds(1000); // play with this parameter
    }
#endif

    digitalWrite(SPI_CS1_PIN, HIGH);
    delay(1);
}

void triggerSw3RoutineOff()
{
    //Serial.println("Triggering SW3 button on");
    digitalWrite(EXT_RELAY_PINS[D_EXT_RELAYS], LOW);
    digitalWrite(EXT_RELAY_PINS[D_EXT_RELAYS+4], LOW);

    digitalWrite(SPI_CS1_PIN, LOW);
    delay(1);

    //start sending to SPI lines begining of RELAY request type
#ifdef SPI_TRANSFER_BUFFER
    SPI.transfer(off_grpD_relays_cmd, sizeof(off_grpD_relays_cmd));
#else
    for (int i = 0; i < sizeof(off_grpD_relays_cmd); i++)
    {
        SPI.transfer(off_grpD_relays_cmd[i]);
        delayMicroseconds(1000); // play with this parameter
    }
#endif

    digitalWrite(SPI_CS1_PIN, HIGH);
    delay(1);
}

void triggerSw4Routine()
{
    //Serial.println("Triggering SW4 button on");
    digitalWrite(EXT_RELAY_PINS[A_EXT_RELAYS], HIGH);
    digitalWrite(EXT_RELAY_PINS[A_EXT_RELAYS+4], HIGH);

    digitalWrite(SPI_CS1_PIN, LOW);
    delay(1);

    //start sending to SPI lines begining of RELAY request type
#ifdef SPI_TRANSFER_BUFFER
    SPI.transfer(on_grpA_relays_cmd, sizeof(on_grpA_relays_cmd));
#else
    for (int i = 0; i < sizeof(on_grpA_relays_cmd); i++)
    {
        SPI.transfer(on_grpA_relays_cmd[i]);
        delayMicroseconds(1000); // play with this parameter
    }
#endif

    digitalWrite(SPI_CS1_PIN, HIGH);
    delay(1);
}

void triggerSw4RoutineOff()
{
    //Serial.println("Triggering SW4 button on");
    digitalWrite(EXT_RELAY_PINS[A_EXT_RELAYS], LOW);
    digitalWrite(EXT_RELAY_PINS[A_EXT_RELAYS+4], LOW);

    digitalWrite(SPI_CS1_PIN, LOW);
    delay(1);

    //start sending to SPI lines begining of RELAY request type
#ifdef SPI_TRANSFER_BUFFER
    SPI.transfer(off_grpA_relays_cmd, sizeof(off_grpA_relays_cmd));
#else
    for (int i = 0; i < sizeof(off_grpA_relays_cmd); i++)
    {
        SPI.transfer(off_grpA_relays_cmd[i]);
        delayMicroseconds(1000); // play with this parameter
    }
#endif

    digitalWrite(SPI_CS1_PIN, HIGH);
    delay(1);
}


void triggerSw5Routine()
{
    //Serial.println("Triggering SW5 button on");

    //trigger group B 1-16, 17-32
    triggerSw2Routine();
    delay(1000);
    triggerSw2RoutineOff();

    //trigger group D 33-48, 49-64
    triggerSw3Routine();
    delay(1000);
    triggerSw3RoutineOff();

    //trigger group A 65-80, 81-96
    triggerSw4Routine();
    delay(1000);
    triggerSw4RoutineOff();

    //trigger group C 97-112, 113-128
    triggerSw7Routine();
    delay(1000);
    triggerSw7RoutineOff();
}

void triggerSw6Routine()
{
    //Serial.println("Triggering SW6 button on");
    turnOffAllExtRelays();
    
    digitalWrite(SLAVE_RESET_PIN, HIGH);
    delay(500); //500ms for slaves to reset
    digitalWrite(SLAVE_RESET_PIN, LOW);
}

void triggerSw7Routine()
{
    //Serial.println("Triggering SW4 button on");
    digitalWrite(EXT_RELAY_PINS[C_EXT_RELAYS], HIGH);
    digitalWrite(EXT_RELAY_PINS[C_EXT_RELAYS+4], HIGH);

    digitalWrite(SPI_CS1_PIN, LOW);
    delay(1);

    //start sending to SPI lines begining of RELAY request type
#ifdef SPI_TRANSFER_BUFFER
    SPI.transfer(on_grpC_relays_cmd, sizeof(on_grpC_relays_cmd));
#else
    for (int i = 0; i < sizeof(on_grpC_relays_cmd); i++)
    {
        SPI.transfer(on_grpC_relays_cmd[i]);
        delayMicroseconds(1000); // play with this parameter
    }
#endif

    digitalWrite(SPI_CS1_PIN, HIGH);
    delay(1);
}

void triggerSw7RoutineOff()
{
    //Serial.println("Triggering SW4 button on");
    digitalWrite(EXT_RELAY_PINS[C_EXT_RELAYS], LOW);
    digitalWrite(EXT_RELAY_PINS[C_EXT_RELAYS+4], LOW);

    digitalWrite(SPI_CS1_PIN, LOW);
    delay(1);

    //start sending to SPI lines begining of RELAY request type
#ifdef SPI_TRANSFER_BUFFER
    SPI.transfer(off_grpC_relays_cmd, sizeof(off_grpC_relays_cmd));
#else
    for (int i = 0; i < sizeof(off_grpC_relays_cmd); i++)
    {
        SPI.transfer(off_grpC_relays_cmd[i]);
        delayMicroseconds(1000); // play with this parameter
    }
#endif

    digitalWrite(SPI_CS1_PIN, HIGH);
    delay(1);
}

void resetBuffer()
{
    delim_count = 0;
    memset(delim_idx, 0, sizeof(delim_idx));
    memset(cmd_str, 0, MAX_BUFFERED_CMD);
}

void debounceTimerTick()
{
    Sw2Debouncer.update();
    Sw3Debouncer.update();
    Sw4Debouncer.update();
    Sw5Debouncer.update();
    Sw6Debouncer.update();
    Sw7Debouncer.update();
}

void checkSwState()
{
    if (Sw2Debouncer.changed())
    {
        if (Sw2Debouncer.read() == HIGH)
        {
            triggerSw2Routine();
        }
    }

    if (Sw3Debouncer.changed())
    {
        if (Sw3Debouncer.read() == HIGH)
        {
            triggerSw3Routine();
        }
    }

    if (Sw4Debouncer.changed())
    {
        if (Sw4Debouncer.read() == HIGH)
        {
            triggerSw4Routine();
        }
    }

    if (Sw5Debouncer.changed())
    {
        if (Sw5Debouncer.read() == HIGH)
        {
            triggerSw5Routine();
        }
    }

    if (Sw6Debouncer.changed())
    {
        if (Sw6Debouncer.read() == HIGH)
        {
            triggerSw6Routine();
        }
    }

    if (Sw7Debouncer.changed())
    {
        if (Sw7Debouncer.read() == HIGH)
        {
            triggerSw7Routine();
        }
    }
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

    for (byte i = 0; i < MAX_EXT_RELAYS; i++)
    {
        // ext relays
        pinMode(EXT_RELAY_PINS[i], OUTPUT);
        digitalWrite(EXT_RELAY_PINS[i], LOW);

        // ext gate
        pinMode(EXT_RELAY_PINS[i+4], OUTPUT);
        digitalWrite(EXT_RELAY_PINS[i+4], LOW);
    }

    Sw2Debouncer.attach(SW2_PIN, INPUT_PULLUP);
    Sw3Debouncer.attach(SW3_PIN, INPUT_PULLUP);
    Sw4Debouncer.attach(SW4_PIN, INPUT_PULLUP);
    Sw5Debouncer.attach(SW5_PIN, INPUT_PULLUP);
    Sw6Debouncer.attach(SW6_PIN, INPUT_PULLUP);
    Sw7Debouncer.attach(SW7_PIN, INPUT_PULLUP);

    Sw2Debouncer.interval(DEBOUNCE_MSEC);
    Sw3Debouncer.interval(DEBOUNCE_MSEC);
    Sw4Debouncer.interval(DEBOUNCE_MSEC);
    Sw5Debouncer.interval(DEBOUNCE_MSEC);
    Sw6Debouncer.interval(DEBOUNCE_MSEC);
    Sw7Debouncer.interval(DEBOUNCE_MSEC);

    //begin spi transaction here and not end it, as we're the only SPI user
    SPI.beginTransaction(settings);
    digitalWrite(SPI_CS1_PIN, HIGH);

    pinMode(SLAVE_RESET_PIN, OUTPUT);
    digitalWrite(SLAVE_RESET_PIN, HIGH);
    delay(500); //500ms for slaves to reset
    digitalWrite(SLAVE_RESET_PIN, LOW);
}

void loop() {
    // Update the Bounce instance (YOU MUST DO THIS EVERY LOOP)
    debounceTimerTick();

    checkSwState();

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
                        //replace R with Q for RELAYGRP first byte
                        cmd_str[delim_idx[1]+1] = RELAYGRP_CHAR;

                        digitalWrite(SPI_CS1_PIN, LOW);
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

                        digitalWrite(SPI_CS1_PIN, HIGH);
                        delay(1);

                        printReply(ACK_STR);
                    }
                    else
                    {
                        //check for SLOT number
                        int SLOT_num = atoi(&cmd_str[delim_idx[0]+1]);

                        //SLOT 1-8 are accepted but only triggering SLOT 1 as the only external relay
                        if ( (SLOT_num >= 1) && (SLOT_num <= 8) )
                        {
                            digitalWrite(SPI_CS1_PIN, LOW);
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

                            digitalWrite(SPI_CS1_PIN, HIGH);
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
                                    if (delim_count == 9)
                                    {
                                        //check for RELAY numbers
                                        bool is_err = false;
                                        int ext_RELAY[MAX_EXT_RELAYS] = {};
                                        int total_relays = 0;
                                        for (int i = 0; i < MAX_EXT_RELAYS; i++)
                                        {
                                            int tmp = cmd_str[delim_idx[i+2]+1];
                                            if ( (tmp >= 0x61) && (tmp <= 0x68) ) // only supports ext RELAY 'a' to 'h'
                                            {
                                                ext_RELAY[total_relays] = tmp - 0x61; //offset to GPIO pin
                                                total_relays++;
                                            }
                                            else if ( (tmp - 0x30) != 0) //skip if 0
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
                                                digitalWrite(EXT_RELAY_PINS[ext_RELAY[i]], HIGH);
                                                //Serial.println("Received RELAY:ON command");
                                            }
                                        }
                                        else if ( (!is_err) && (cmd_str[delim_idx[6]+1] == OFF_CHAR) )
                                        {
                                            //loop through all RELAY numbers to OFF
                                            for (int i = 0; i < total_relays; i++)
                                            {
                                                digitalWrite(EXT_RELAY_PINS[ext_RELAY[i]], LOW);
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
                                                    digitalWrite(EXT_RELAY_PINS[ext_RELAY[i]], LOW);
                                                    //Serial.println("Received RELAY:OFF command");
                                                }
                                            }
                                            else if ( (!is_err) && (cmd_str[delim_idx[6]+1] == OFF_CHAR) )
                                            {
                                                //loop through all RELAY numbers to ON - inverse control
                                                for (int i = 0; i < total_relays; i++)
                                                {
                                                    digitalWrite(EXT_RELAY_PINS[ext_RELAY[i]], HIGH);
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
}
