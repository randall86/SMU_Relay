// SMU Relay Tester (Master)
// Rev 1.0 (27/12/2025)
// - Maxtrax

#include <Scheduler.h>
#include <Countimer.h>
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

const byte SLAVE_RESET_PIN = 6; 

const byte SPI_CS1_PIN = 0;

const byte EXT_RELAY_A = 2;
const byte EXT_RELAY_B = 3;
const byte EXT_RELAY_C = 4;
const byte EXT_RELAY_D = 5;

const byte SW2_PIN = 17;
const byte SW3_PIN = 18;
const byte SW4_PIN = 19;
const byte SW5_PIN = 20;
const byte SW6_PIN = 21;

const int MAX_BUFFERED_CMD = 64;

const int MAX_DELIMS = 16;
const byte MAX_EXT_RELAYS = 4;

const uint32_t DEBOUNCE_MSEC = 300; //stable time before registering state change
const uint32_t CHECK_MSEC = 100; //read switch every 100ms when detected state change

//supported on/off durations in ms
const int DURATION_MS[5] = {
    100,
    200,
    300,
    400,
    500
};

SPISettings settings = SPISettings(SPI_TRANSFER_CLOCK_FREQ, MSBFIRST, SPI_MODE0);

int delim_count = 0;
int delim_idx[MAX_DELIMS] = {};
int cmd_idx = 0;
char cmd_str[MAX_BUFFERED_CMD] = {};

char on_all_relays_cmd[] = {RELAY_CHAR, RESET_CHAR, '1', END_CHAR};
char on_grpA_relays_cmd[] = {RELAYGRP_CHAR, 'A', '1', END_CHAR};
char on_grpB_relays_cmd[] = {RELAYGRP_CHAR, 'B', '1', END_CHAR};
char on_grpC_relays_cmd[] = {RELAYGRP_CHAR, 'C', '1', END_CHAR};
char on_grpD_relays_cmd[] = {RELAYGRP_CHAR, 'D', '1', END_CHAR};
char on_seq_relays_cmd[] = {RELAY_CHAR, RESET_CHAR, RESET_CHAR, END_CHAR};

byte EXT_RELAY_PINS[MAX_EXT_RELAYS] = {
    EXT_RELAY_A,
    EXT_RELAY_B,
    EXT_RELAY_C,
    EXT_RELAY_D
};

//timer for start/stop switch debouncing
Countimer debounceTimerSw2;
Countimer debounceTimerSw3;
Countimer debounceTimerSw4;
Countimer debounceTimerSw5;
Countimer debounceTimerSw6;

bool is_Sw2_read = false;
bool is_Sw3_read = false;
bool is_Sw4_read = false;
bool is_Sw5_read = false;
bool is_Sw6_read = false;

#ifdef VERBOSE_REPLY
void printReply(const char * reply)
{
    Serial.println(reply);
}
#else
void printReply(const char * /*reply*/) {};
#endif

//returns true if state changed
bool debounceSwitch(byte *state, byte pin, uint8_t *count)
{
    bool state_changed = false;

    //read the switch from the HW
    byte raw_state = digitalRead(pin);

    if (raw_state == *state)
    {
        //set the timer which allows a change from current state.
        *count = DEBOUNCE_MSEC/CHECK_MSEC;
    }
    else
    {
        //state has changed - wait for new state to become stable.
        if (--*count == 0)
        {
            // Timer expired - accept the change.
            *state = raw_state;
            state_changed = true;

            // And reset the timer.
            *count = DEBOUNCE_MSEC/CHECK_MSEC;
        }
    }

    return state_changed;
}

void turnOnAllExtRelays()
{
    for (int i = 0; i < MAX_EXT_RELAYS; i++)
    {
        digitalWrite(EXT_RELAY_PINS[i], HIGH);
    }
}

void turnOffAllExtRelays()
{
    for (int i = 0; i < MAX_EXT_RELAYS; i++)
    {
        digitalWrite(EXT_RELAY_PINS[i], LOW);
    }
}

void debounceSw2Routine()
{
    static byte switch_state = 0;
    static uint8_t count = DEBOUNCE_MSEC/CHECK_MSEC;
    
    //if switch state changed, update the state
    if(debounceSwitch(&switch_state, SW2_PIN, &count))
    {
        debounceTimerSw2.stop();
        if(switch_state) // send on all relays 1-128 - R,X,1,E
        {
            turnOnAllExtRelays();

            digitalWrite(SPI_CS1_PIN, LOW);
            delay(1);

            //start sending to SPI lines begining of RELAY request type
        #ifdef SPI_TRANSFER_BUFFER
            SPI.transfer(on_all_relays_cmd, sizeof(on_all_relays_cmd));
        #else
            for (int i = 0; i < sizeof(on_all_relays_cmd); i++)
            {
                SPI.transfer(on_all_relays_cmd[i]);
                delayMicroseconds(1000); // play with this parameter
            }
        #endif

            digitalWrite(SPI_CS1_PIN, HIGH);
            delay(1);

            is_Sw2_read = false;
        }
    }
}

void debounceSw3Routine()
{
    static byte switch_state = 0;
    static uint8_t count = DEBOUNCE_MSEC/CHECK_MSEC;
    
    //if switch state changed, update the state
    if(debounceSwitch(&switch_state, SW3_PIN, &count))
    {
        debounceTimerSw3.stop();
        if(switch_state) // send on all relays 65-128 (group A and C) - Q,A,1,E and Q,C,1,E
        {
            turnOnAllExtRelays();

            digitalWrite(SPI_CS1_PIN, LOW);
            delay(1);

            //start sending to SPI lines begining of RELAY request type
        #ifdef SPI_TRANSFER_BUFFER
            SPI.transfer(on_grpA_relays_cmd, sizeof(on_grpA_relays_cmd));
            SPI.transfer(on_grpC_relays_cmd, sizeof(on_grpC_relays_cmd));
        #else
            for (int i = 0; i < sizeof(on_grpA_relays_cmd); i++)
            {
                SPI.transfer(on_grpA_relays_cmd[i]);
                delayMicroseconds(1000); // play with this parameter
            }
            
            for (int i = 0; i < sizeof(on_grpC_relays_cmd); i++)
            {
                SPI.transfer(on_grpC_relays_cmd[i]);
                delayMicroseconds(1000); // play with this parameter
            }
        #endif

            digitalWrite(SPI_CS1_PIN, HIGH);
            delay(1);

            is_Sw3_read = false;
        } 
    }
}

void debounceSw4Routine()
{
    static byte switch_state = 0;
    static uint8_t count = DEBOUNCE_MSEC/CHECK_MSEC;
    
    //if switch state changed, update the state
    if(debounceSwitch(&switch_state, SW4_PIN, &count))
    {
        debounceTimerSw4.stop();
        if(switch_state) // send on all relays 1-64 (group B and D) - Q,B,1,E and Q,D,1,E
        {
            turnOnAllExtRelays();

            digitalWrite(SPI_CS1_PIN, LOW);
            delay(1);

            //start sending to SPI lines begining of RELAY request type
        #ifdef SPI_TRANSFER_BUFFER
            SPI.transfer(on_grpB_relays_cmd, sizeof(on_grpB_relays_cmd));
            SPI.transfer(on_grpD_relays_cmd, sizeof(on_grpD_relays_cmd));
        #else
            for (int i = 0; i < sizeof(on_grpB_relays_cmd); i++)
            {
                SPI.transfer(on_grpB_relays_cmd[i]);
                delayMicroseconds(1000); // play with this parameter
            }
            
            for (int i = 0; i < sizeof(on_grpD_relays_cmd); i++)
            {
                SPI.transfer(on_grpD_relays_cmd[i]);
                delayMicroseconds(1000); // play with this parameter
            }
        #endif

            digitalWrite(SPI_CS1_PIN, HIGH);
            delay(1);

            is_Sw4_read = false;
        }
    }
}

void debounceSw5Routine()
{
    static byte switch_state = 0;
    static uint8_t count = DEBOUNCE_MSEC/CHECK_MSEC;
    
    //if switch state changed, update the state
    if(debounceSwitch(&switch_state, SW5_PIN, &count))
    {
        debounceTimerSw5.stop();
        if(switch_state) // send on relays 1-128 in sequence - R,X,X,E
        {
            turnOnAllExtRelays();

            digitalWrite(SPI_CS1_PIN, LOW);
            delay(1);

            //start sending to SPI lines begining of RELAY request type
        #ifdef SPI_TRANSFER_BUFFER
            SPI.transfer(on_seq_relays_cmd, sizeof(on_seq_relays_cmd));
        #else
            for (int i = 0; i < sizeof(on_seq_relays_cmd); i++)
            {
                SPI.transfer(on_seq_relays_cmd[i]);
                delayMicroseconds(1000); // play with this parameter
            }
        #endif

            digitalWrite(SPI_CS1_PIN, HIGH);
            delay(1);

            is_Sw5_read = false;
        }
    }
}

void debounceSw6Routine()
{
    static byte switch_state = 0;
    static uint8_t count = DEBOUNCE_MSEC/CHECK_MSEC;
    
    //if switch state changed, update the state
    if(debounceSwitch(&switch_state, SW6_PIN, &count))
    {
        debounceTimerSw6.stop();
        if(switch_state) // hard reset on relay board same as S,0,R,X,F,E
        {
            turnOffAllExtRelays();

            digitalWrite(SLAVE_RESET_PIN, HIGH);
            delay(500); //500ms for slaves to reset
            digitalWrite(SLAVE_RESET_PIN, LOW);

            is_Sw6_read = false;
        }
    }
}

void resetBuffer()
{
    delim_count = 0;
    memset(delim_idx, 0, sizeof(delim_idx));
    memset(cmd_str, 0, MAX_BUFFERED_CMD);
}

void debounceTimerTick()
{
    // tick the debounce timer
    debounceTimerSw2.run();
    debounceTimerSw3.run();
    debounceTimerSw4.run();
    debounceTimerSw5.run();
    debounceTimerSw6.run();

    // read the state of the button value
    if (digitalRead(SW2_PIN) && !is_Sw2_read)
    {
        debounceTimerSw2.start();
        is_Sw2_read = true;
    }
    
    if (digitalRead(SW3_PIN) && !is_Sw3_read)
    {
        debounceTimerSw3.start();
        is_Sw2_read = true;
    }
    
    if (digitalRead(SW4_PIN) && !is_Sw4_read)
    {
        debounceTimerSw4.start();
        is_Sw4_read = true;
    }
    
    if (digitalRead(SW5_PIN) && !is_Sw5_read)
    {
        debounceTimerSw5.start();
        is_Sw5_read = true;
    }
    
    if (digitalRead(SW6_PIN) && !is_Sw6_read)
    {
        debounceTimerSw6.start();
        is_Sw6_read = true;
    }

    yield(); //yield to pass control to other tasks
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
        pinMode(EXT_RELAY_PINS[i], OUTPUT);
        digitalWrite(EXT_RELAY_PINS[i], LOW);
    }

    pinMode(SW2_PIN, INPUT);
    pinMode(SW3_PIN, INPUT);
    pinMode(SW4_PIN, INPUT);
    pinMode(SW5_PIN, INPUT);
    pinMode(SW6_PIN, INPUT);

    //begin spi transaction here and not end it, as we're the only SPI user
    SPI.beginTransaction(settings);
    digitalWrite(SPI_CS1_PIN, HIGH);

    pinMode(SLAVE_RESET_PIN, OUTPUT);
    digitalWrite(SLAVE_RESET_PIN, HIGH);
    delay(500); //500ms for slaves to reset
    digitalWrite(SLAVE_RESET_PIN, LOW);

    debounceTimerSw2.setInterval(debounceSw2Routine, CHECK_MSEC);
    debounceTimerSw3.setInterval(debounceSw3Routine, CHECK_MSEC);
    debounceTimerSw4.setInterval(debounceSw4Routine, CHECK_MSEC);
    debounceTimerSw5.setInterval(debounceSw5Routine, CHECK_MSEC);
    debounceTimerSw6.setInterval(debounceSw6Routine, CHECK_MSEC);
    Scheduler.startLoop(debounceTimerTick); //check switch state from interrupt
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
                                            if ( (tmp >= 0x61) && (tmp <= 0x64) ) // only supports ext RELAY 'a' to 'd'
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
