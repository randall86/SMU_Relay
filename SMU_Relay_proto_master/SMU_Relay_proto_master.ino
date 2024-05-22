// SMU Relay proto board (Master)
// Rev 1.0 (31/12/2023)
// - Maxtrax

#include <SPI.h>

#define SPI_TRANSFER_CLOCK_FREQ_12 12000000
#define SPI_TRANSFER_CLOCK_FREQ SPI_TRANSFER_CLOCK_FREQ_12

const char * app_ver = "v1.0";

const byte CARD1_SPI_CS_PIN = 0;

SPISettings settings = SPISettings(SPI_TRANSFER_CLOCK_FREQ, MSBFIRST, SPI_MODE0);

void setup() {
    // put your setup code here, to run once:
    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    while (!Serial);

    Serial.print("SMU Relay proto board (Master)");
    Serial.println(app_ver);

    SPI.begin();
    pinMode(CARD1_SPI_CS_PIN, OUTPUT);
    digitalWrite(CARD1_SPI_CS_PIN, HIGH);
}

void loop() {
    // put your main code here, to run repeatedly:
    if (Serial.available())
    {
        SPI.beginTransaction(settings);
        digitalWrite(CARD1_SPI_CS_PIN, LOW);

        SPI.transfer(Serial.read());
        delayMicroseconds(1000); // play with this parameter

        digitalWrite(CARD1_SPI_CS_PIN, HIGH);
        SPI.endTransaction();
    }
}
