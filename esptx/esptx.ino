#include <Arduino.h>
#include <SPI.h>

#define MCP_CS   5
#define MCP_SCK  18
#define MCP_MOSI 19
#define MCP_MISO 21

#define LIGHTING_COMMAND_ID 0x7FF

// CAN command values
#define CMD_LEFT_ON          9
#define CMD_LEFT_OFF         10
#define CMD_RIGHT_ON         11
#define CMD_RIGHT_OFF        12
#define CMD_HEADLIGHTS_ON    13
#define CMD_HEADLIGHTS_OFF   14
#define CMD_HAZARDS_ON       15
#define CMD_HAZARDS_OFF      16
#define CMD_BRAKES_ON        17
#define CMD_BRAKES_OFF       18


// ==================================================
// SPI
// ==================================================

void cs_low()
{
    digitalWrite(MCP_CS, LOW);
}

void cs_high()
{
    digitalWrite(MCP_CS, HIGH);
}

uint8_t spi_transfer(uint8_t data)
{
    return SPI.transfer(data);
}


// ==================================================
// MCP2515
// ==================================================

void MCP2515_reset()
{
    cs_low();
    spi_transfer(0xC0);      // RESET command
    cs_high();

    delay(10);
}

void MCP2515_write(uint8_t address, uint8_t data)
{
    cs_low();

    spi_transfer(0x02);      // WRITE command
    spi_transfer(address);
    spi_transfer(data);

    cs_high();
}

uint8_t MCP2515_read(uint8_t address)
{
    cs_low();

    spi_transfer(0x03);      // READ command
    spi_transfer(address);

    uint8_t data = spi_transfer(0x00);

    cs_high();

    return data;
}


// Configure MCP2515

void diagByte(const char *name, uint8_t value) {
    Serial.print(name); Serial.print('='); Serial.print(value, HEX); Serial.print(' ');
}
void diagStatus() {
    Serial.print("STATUS ");
    const uint8_t regs[] = {0x0E,0x0F,0x2A,0x29,0x28,0x2C,0x2D,0x1C,0x1D,0x30,0x60,0x70};
    const char *names[] = {"CANSTAT","CANCTRL","CNF1","CNF2","CNF3","CANINTF","EFLG","TEC","REC","TXB0CTRL","RXB0CTRL","RXB1CTRL"};
    for (uint8_t i=0; i<12; ++i) diagByte(names[i], MCP2515_read(regs[i]));
    Serial.println();
}
void bitModify(uint8_t address, uint8_t mask, uint8_t value) {
    cs_low(); spi_transfer(0x05); spi_transfer(address); spi_transfer(mask); spi_transfer(value); cs_high();
}

void MCP2515_init()
{
    MCP2515_reset();
    diagStatus(); // CANSTAT mode bits must be 0x80 after reset.
    if ((MCP2515_read(0x0E) & 0xE0) != 0x80) {
        Serial.println("INIT FAILED: configuration mode / SPI");
        while (true) delay(1000);
    }

    // 125 kbit/s at 8 MHz. BRP=3 passed the live bus test; BRP=0 failed.
    MCP2515_write(0x2A, 0x03);    // CNF1
    MCP2515_write(0x29, 0x90);    // CNF2
    MCP2515_write(0x28, 0x02);    // CNF3

    // Normal operating mode
    MCP2515_write(0x0F, 0x00);    // CANCTRL

    delay(10);
}


// ==================================================
// CAN TRANSMISSION
// ==================================================

bool CAN_SEND(uint16_t id, uint8_t data)
{
    // Check whether TX buffer 0 is busy
    if (MCP2515_read(0x30) & 0x08)
    {
        Serial.println("TX buffer busy");
        return false;
    }

    // Load standard 11-bit CAN identifier
    MCP2515_write(0x31, id >> 3);             // TXB0SIDH
    MCP2515_write(0x32, (id & 0x07) << 5);   // TXB0SIDL

    // One data byte
    MCP2515_write(0x35, 1);                  // TXB0DLC

    // Command byte
    MCP2515_write(0x36, data);               // TXB0D0

    uint8_t h=MCP2515_read(0x31), l=MCP2515_read(0x32);
    uint8_t dlc=MCP2515_read(0x35), d=MCP2515_read(0x36);
    Serial.print("TX_RAW ");
    diagByte("SIDH",h); diagByte("SIDL",l); diagByte("DLC",dlc); diagByte("D0",d); Serial.println();
    if (h != (id >> 3) || l != ((id & 7) << 5) || dlc != 1 || d != data) {
        Serial.println("TX LOAD MISMATCH: no RTS issued");
        diagStatus(); return false;
    }
    bitModify(0x2C, 0x04, 0); // Clear old TX0IF before this request.
    // Request transmission from TX buffer 0
    cs_low();
    spi_transfer(0x81);                       // RTS TXB0
    cs_high();

    uint32_t started=millis();
    while ((MCP2515_read(0x30) & 8) && (uint32_t)(millis()-started)<250) delay(1);
    uint8_t ctrl=MCP2515_read(0x30), flags=MCP2515_read(0x2C);
    Serial.println((!(ctrl & 0x78) && (flags & 4)) ? "TX_COMPLETE" : "TX_PENDING_OR_ERROR");
    diagStatus();
    return true; // Request was issued; receiver confirmation is separate.
}


void sendCommand(uint8_t command)
{
    Serial.print("Sending CAN ID 0x7FF, command = ");
    Serial.println(command);

    if (CAN_SEND(LIGHTING_COMMAND_ID, command))
    {
        Serial.println("CAN transmit requested");
    }
}


// ==================================================
// SETUP
// ==================================================

void setup()
{
    Serial.begin(9600);

    delay(1000);

    Serial.println();
    Serial.println("==========================");
    Serial.println("ESP32 CAN TRANSMITTER");
    Serial.println("==========================");

    pinMode(MCP_CS, OUTPUT);
    digitalWrite(MCP_CS, HIGH);

    SPI.begin(
        MCP_SCK,
        MCP_MISO,
        MCP_MOSI,
        MCP_CS
    );

    SPI.beginTransaction(
        SPISettings(
            1000000,
            MSBFIRST,
            SPI_MODE0
        )
    );

    MCP2515_init();
    diagStatus();
    if ((MCP2515_read(0x0E) & 0xE0) != 0 ||
        MCP2515_read(0x2A) != 3 || MCP2515_read(0x29) != 0x90 || MCP2515_read(0x28) != 2) {
        Serial.println("INIT FAILED: mode or timing readback");
        while (true) delay(1000);
    }

    Serial.print("CANCTRL = 0x");
    Serial.println(MCP2515_read(0x0F), HEX);

    Serial.println();
    Serial.println("Commands:");
    Serial.println("0 = Hazards OFF");
    Serial.println("1 = Hazards ON");
    Serial.println("2 = Left ON");
    Serial.println("3 = Left OFF");
    Serial.println("4 = Right ON");
    Serial.println("5 = Right OFF");
    Serial.println("6 = Headlights ON");
    Serial.println("7 = Headlights OFF");
    Serial.println("8 = Brakes ON");
    Serial.println("9 = Brakes OFF");
}


// ==================================================
// LOOP
// ==================================================

void loop()
{
    if (Serial.available())
    {
        char command = Serial.read();

        if (command == '?') { diagStatus(); return; }
        if (command >= 'a' && command <= 'c') {
            MCP2515_reset();
            MCP2515_write(0x2A,command == 'a' ? 0 : command == 'b' ? 1 : 3);
            MCP2515_write(0x29,0x90); MCP2515_write(0x28,2);
            MCP2515_write(0x0F,0x08); // One-shot for bounded bus diagnostics.
            delay(10); diagStatus(); return;
        }
        if (command == 'R') { MCP2515_init(); diagStatus(); return; }
        if (command == 'L') {
            MCP2515_reset();
            MCP2515_write(0x2A,0); MCP2515_write(0x29,0x90); MCP2515_write(0x28,2);
            MCP2515_write(0x0F,0x40); delay(10);
            CAN_SEND(0x7FF,15);
            Serial.print("LOOPBACK ");
            diagByte("FLAGS",MCP2515_read(0x2C));
            diagByte("SIDH",MCP2515_read(0x61)); diagByte("SIDL",MCP2515_read(0x62));
            diagByte("DLC",MCP2515_read(0x65)); diagByte("D0",MCP2515_read(0x66)); Serial.println();
            MCP2515_init(); return;
        }
        switch (command)
        {
            case '0':
                sendCommand(CMD_HAZARDS_OFF);
                break;

            case '1':
                sendCommand(CMD_HAZARDS_ON);
                break;

            case '2':
                sendCommand(CMD_LEFT_ON);
                break;

            case '3':
                sendCommand(CMD_LEFT_OFF);
                break;

            case '4':
                sendCommand(CMD_RIGHT_ON);
                break;

            case '5':
                sendCommand(CMD_RIGHT_OFF);
                break;

            case '6':
                sendCommand(CMD_HEADLIGHTS_ON);
                break;

            case '7':
                sendCommand(CMD_HEADLIGHTS_OFF);
                break;

            case '8':
                sendCommand(CMD_BRAKES_ON);
                break;

            case '9':
                sendCommand(CMD_BRAKES_OFF);
                break;
        }
    }
}
