#include <Arduino.h>
#include <avr/io.h>

// ============================================================
// CAN COMMAND DEFINITIONS
// ============================================================

#define LIGHTING_COMMAND_ID 0x7FF

#define CMD_LEFT_ON          9
#define CMD_LEFT_OFF        10
#define CMD_RIGHT_ON        11
#define CMD_RIGHT_OFF       12
#define CMD_HAZARDS_ON      15
#define CMD_HAZARDS_OFF     16

// Lighting outputs
#define LEFT_LED  22
#define RIGHT_LED 23


// ============================================================
// LIGHTING STATE MACHINE
// ============================================================

enum LightState_t
{
    ALL_OFF,
    LEFT_ON,
    RIGHT_ON,
    HAZARD_ON
};

LightState_t turn_mode = ALL_OFF;

bool blink_on = false;
uint32_t last_toggle = 0;


// ============================================================
// LOW-LEVEL SPI DRIVER
// Mega 2560:
//
// Pin 53 = CS
// Pin 52 = SCK
// Pin 51 = MOSI
// Pin 50 = MISO
// ============================================================

void cs_low()
{
    PORTB &= ~(1 << PB0);
}

void cs_high()
{
    PORTB |= (1 << PB0);
}


uint8_t spi_transfer(uint8_t data)
{
    SPDR = data;

    while (!(SPSR & (1 << SPIF)))
    {
        // Wait for SPI transfer to complete
    }

    return SPDR;
}


void spi_init()
{
    // PB0 = CS   output
    // PB1 = SCK  output
    // PB2 = MOSI output
    // PB3 = MISO input

    DDRB |= (1 << PB0)
          | (1 << PB1)
          | (1 << PB2);

    DDRB &= ~(1 << PB3);

    cs_high();

    // SPI enabled
    // Master mode
    // Mode 0
    // Clock divider = 16
    SPCR = (1 << SPE)
         | (1 << MSTR)
         | (1 << SPR0);

    SPSR &= ~(1 << SPI2X);
}


// ============================================================
// MCP2515 LOW-LEVEL DRIVER
// ============================================================

void MCP2515_reset()
{
    cs_low();

    spi_transfer(0xC0);     // RESET instruction

    cs_high();

    delay(10);
}


void MCP2515_write(uint8_t address, uint8_t data)
{
    cs_low();

    spi_transfer(0x02);     // WRITE instruction
    spi_transfer(address);
    spi_transfer(data);

    cs_high();
}


uint8_t MCP2515_read(uint8_t address)
{
    cs_low();

    spi_transfer(0x03);     // READ instruction
    spi_transfer(address);

    uint8_t data = spi_transfer(0x00);

    cs_high();

    return data;
}


// ============================================================
// MCP2515 INITIALIZATION
// ============================================================


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

    // CAN bit timing
    // Intended configuration:
    // 8 MHz MCP2515 oscillator
    // 125 kbit/s CAN; original 500 kbit/s failed live bus tests.

    MCP2515_write(0x2A, 0x03);   // CNF1
    MCP2515_write(0x29, 0x90);   // CNF2
    MCP2515_write(0x28, 0x02);   // CNF3

    // RXB0CTRL
    // Accept valid frames using the normal filters. RXM=11 would place
    // incomplete/error frames in RXB0 and can look like corrupt CAN data.
    MCP2515_write(0x60, 0x00);

    // Put MCP2515 into normal CAN mode
    MCP2515_write(0x0F, 0x00);

    delay(10);
}


// ============================================================
// CAN RECEIVE
// ============================================================

bool CAN_receive(
    uint16_t *id,
    uint8_t *length,
    uint8_t *data
)
{
    uint8_t flags=MCP2515_read(0x2C);
    if (!(flags & 3)) return false;
    uint8_t mask=(flags & 1) ? 1 : 2;
    uint8_t base=(mask == 1) ? 0x60 : 0x70;
    uint8_t ctrl=MCP2515_read(base);
    uint8_t high=MCP2515_read(base+1), low=MCP2515_read(base+2);
    uint8_t dlc=MCP2515_read(base+5), d0=MCP2515_read(base+6);
    *length=dlc & 15;
    *id=((uint16_t)high << 3) | (low >> 5);
    for (uint8_t i=0; i<*length && i<8; ++i) data[i]=MCP2515_read(base+6+i);
    // Release even malformed frames. Snapshot flags before serial printing.
    bitModify(0x2C,mask,0);
    uint8_t after=MCP2515_read(0x2C);
    static uint32_t lastPrint=0;
    static bool printed=false;
    bool report=!printed || (uint32_t)(millis()-lastPrint)>=1000;
    if (report) {
        printed=true; lastPrint=millis();
        Serial.print("RX_RAW "); diagByte("BUFFER",mask-1);
        diagByte("CTRL",ctrl); diagByte("SIDH",high); diagByte("SIDL",low);
        diagByte("DLC",dlc); diagByte("D0",d0);
        diagByte("FLAGS_BEFORE",flags); diagByte("FLAGS_AFTER",after); Serial.println();
        diagStatus();
    }
    // Extended and remote frames must never act as lighting commands.
    if (*length>8 || (low & 8) || (ctrl & 8)) return false;
    // Suppress repeated irrelevant frames while retaining one diagnostic/second.
    if (*id != LIGHTING_COMMAND_ID || *length != 1) return false;
    return true;
}


// ============================================================
// LIGHT OUTPUTS
// ============================================================

void lights_off()
{
    digitalWrite(LEFT_LED, LOW);
    digitalWrite(RIGHT_LED, LOW);
}


void lights_set(LightState_t state)
{
    if (state == LEFT_ON)
    {
        digitalWrite(LEFT_LED, HIGH);
        digitalWrite(RIGHT_LED, LOW);
    }

    else if (state == RIGHT_ON)
    {
        digitalWrite(LEFT_LED, LOW);
        digitalWrite(RIGHT_LED, HIGH);
    }

    else if (state == HAZARD_ON)
    {
        digitalWrite(LEFT_LED, HIGH);
        digitalWrite(RIGHT_LED, HIGH);
    }

    else
    {
        lights_off();
    }
}


// ============================================================
// LIGHTING COMMAND / FSM
// ============================================================

void processCommand(uint8_t command)
{
    LightState_t next_mode = turn_mode;

    if (command == CMD_HAZARDS_ON)
    {
        next_mode = HAZARD_ON;
    }

    else if (command == CMD_HAZARDS_OFF)
    {
        if (turn_mode == HAZARD_ON)
        {
            next_mode = ALL_OFF;
        }
    }

    else if (command == CMD_LEFT_ON)
    {
        next_mode = LEFT_ON;
    }

    else if (command == CMD_LEFT_OFF)
    {
        if (turn_mode == LEFT_ON)
        {
            next_mode = ALL_OFF;
        }
    }

    else if (command == CMD_RIGHT_ON)
    {
        next_mode = RIGHT_ON;
    }

    else if (command == CMD_RIGHT_OFF)
    {
        if (turn_mode == RIGHT_ON)
        {
            next_mode = ALL_OFF;
        }
    }

    // Only update outputs when state changes
    if (next_mode != turn_mode)
    {
        turn_mode = next_mode;

        blink_on = (turn_mode != ALL_OFF);

        last_toggle = millis();

        lights_set(turn_mode);
    }
    const char *states[] = {"ALL_OFF", "LEFT_ON", "RIGHT_ON", "HAZARD_ON"};
    Serial.print("STATE: "); Serial.println(states[turn_mode]);
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(9600);

    pinMode(LEFT_LED, OUTPUT);
    pinMode(RIGHT_LED, OUTPUT);

    lights_off();

    spi_init();
    MCP2515_init();
    diagStatus();
    if ((MCP2515_read(0x0E) & 0xE0) != 0 ||
        MCP2515_read(0x2A) != 3 || MCP2515_read(0x29) != 0x90 || MCP2515_read(0x28) != 2) {
        Serial.println("INIT FAILED: mode or timing readback");
        while (true) delay(1000);
    }

    Serial.println();
    Serial.println("==========================");
    Serial.println("MEGA CAN RECEIVER");
    Serial.println("==========================");

    Serial.print("CANCTRL = 0x");
    Serial.println(MCP2515_read(0x0F), HEX);

    Serial.println("Waiting for CAN frames...");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    if (Serial.available()) {
        char command=Serial.read();
        if (command == '?') diagStatus();
        if (command >= 'a' && command <= 'c') {
            MCP2515_reset();
            MCP2515_write(0x2A,command == 'a' ? 0 : command == 'b' ? 1 : 3);
            MCP2515_write(0x29,0x90); MCP2515_write(0x28,2);
            MCP2515_write(0x0F,0); delay(10); diagStatus();
        }
        if (command == 'R') { MCP2515_init(); diagStatus(); }
        if (command == 'L') {
            MCP2515_reset();
            MCP2515_write(0x2A,0); MCP2515_write(0x29,0x90); MCP2515_write(0x28,2);
            MCP2515_write(0x0F,0x40); delay(10);
            MCP2515_write(0x31,0xFF); MCP2515_write(0x32,0xE0);
            MCP2515_write(0x35,1); MCP2515_write(0x36,15);
            cs_low(); spi_transfer(0x81); cs_high(); delay(10);
            Serial.print("LOOPBACK ");
            diagByte("FLAGS",MCP2515_read(0x2C));
            diagByte("SIDH",MCP2515_read(0x61)); diagByte("SIDL",MCP2515_read(0x62));
            diagByte("DLC",MCP2515_read(0x65)); diagByte("D0",MCP2515_read(0x66)); Serial.println();
            MCP2515_init();
        }
    }
    uint16_t id;
    uint8_t length;
    uint8_t data[8];

    // Check MCP2515 for received CAN frame
    if (CAN_receive(&id, &length, data))
    {
        Serial.print("RX CAN ID: 0x");
        Serial.print(id, HEX);

        Serial.print(" DLC: ");
        Serial.print(length);

        Serial.print(" DATA: ");

        for (uint8_t i = 0; i < length; i++)
        {
            Serial.print(data[i]);
            Serial.print(" ");
        }

        Serial.println();

        // Only process our lighting command
        if (id == LIGHTING_COMMAND_ID && length == 1)
        {
            Serial.print("Lighting command received: ");
            Serial.println(data[0]);

            processCommand(data[0]);
        }
    }


    // ========================================================
    // NON-BLOCKING TURN SIGNAL BLINKING
    // ========================================================

    if (turn_mode != ALL_OFF)
    {
        uint32_t now = millis();

        if ((uint32_t)(now - last_toggle) >= 500)
        {
            last_toggle = now;

            blink_on = !blink_on;

            if (blink_on)
            {
                lights_set(turn_mode);
            }
            else
            {
                lights_off();
            }
        }
    }
}
