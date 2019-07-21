#include <stdio.h>
#include <stdint.h>

#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <SSD1306.h>
#include "soc/efuse_reg.h"
#include "HardwareSerial.h"
#include "EEPROM.h"

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] =
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned TX_INTERVAL = 10;

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

#define PIN_BUTTON 0

#define OTAA_MAGIC 0xCAFEBABE

typedef struct {
    u4_t netid = 0;
    devaddr_t devaddr = 0;
    u1_t nwkKey[16];
    u1_t artKey[16];
    uint32_t magic;
} otaa_data_t;

typedef struct {
    bool update;
    // 1st line: LoRa address
    char loraDevEui[32];
    // 2nd line: LoRa status
    char loraStatus[32];
} screen_t;

// stored in "little endian" format
static uint8_t deveui[8];
static otaa_data_t otaa_data;
static SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);
static screen_t screen;

// This should also be in little endian format, see above.
void os_getDevEui(u1_t * buf)
{
    memcpy_P(buf, deveui, 8);
}

void os_getArtEui(u1_t * buf)
{
    memcpy_P(buf, APPEUI, 8);
}

void os_getDevKey(u1_t * buf)
{
    memcpy_P(buf, APPKEY, 16);
}

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = { 26, 33, 32 }       // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

static void print_keys(void)
{
    Serial.print("netid: ");
    Serial.println(otaa_data.netid, DEC);
    Serial.print("devaddr: ");
    Serial.println(otaa_data.devaddr, HEX);
    Serial.print("artKey: ");
    for (int i = 0; i < sizeof(otaa_data.artKey); ++i) {
        Serial.printf("%02X", otaa_data.artKey[i]);
    }
    Serial.println("");
    Serial.print("nwkKey: ");
    for (int i = 0; i < sizeof(otaa_data.nwkKey); ++i) {
        Serial.printf("%02X", otaa_data.nwkKey[i]);
    }
    Serial.println("");
}

static void setLoraStatus(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(screen.loraStatus, sizeof(screen.loraStatus), fmt, args);
    va_end(args);

    screen.update = true;
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        setLoraStatus("OTAA JOIN...");
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        LMIC_getSessionKeys(&otaa_data.netid, &otaa_data.devaddr, otaa_data.nwkKey,
                            otaa_data.artKey);
        otaa_data.magic = OTAA_MAGIC;
        EEPROM.put(0, otaa_data);
        EEPROM.commit();

        print_keys();

        setLoraStatus("JOIN OK!");
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        setLoraStatus("JOIN failed!");
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        setLoraStatus("REJOIN failed!");
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        setLoraStatus("%08X", LMIC.devaddr);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        setLoraStatus("Transmitting");
        break;
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        Serial.println(F("EV_RXSTART"));
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE"));
        setLoraStatus("JOIN sent");
        break;
    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned) ev);
        break;
    }
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println(F("Starting..."));

    // button config
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    // init the OLED
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    // setup of unique ids
    uint64_t chipid = ESP.getEfuseMac();
    deveui[0] = (chipid >> 0) & 0xFF;
    deveui[1] = (chipid >> 8) & 0xFF;
    deveui[2] = (chipid >> 16) & 0xFF;
    deveui[3] = (chipid >> 24) & 0xFF;
    deveui[4] = (chipid >> 32) & 0xFF;
    deveui[5] = (chipid >> 40) & 0xFF;
    deveui[6] = (chipid >> 48) & 0xFF;
    deveui[7] = (chipid >> 56) & 0xFF;
    snprintf(screen.loraDevEui, sizeof(screen.loraDevEui),
             "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", deveui[7], deveui[6], deveui[5], deveui[4],
             deveui[3], deveui[2], deveui[1], deveui[0]);

    // LMIC init
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    EEPROM.begin(512);
    EEPROM.get(0, otaa_data);
    if (otaa_data.magic == OTAA_MAGIC) {
        setLoraStatus("Resume OTAA");
        LMIC_setSession(otaa_data.netid, otaa_data.devaddr, otaa_data.nwkKey, otaa_data.artKey);
        print_keys();
    } else {
        LMIC_startJoining();
    }

}

static void screen_update(void)
{
    char value[16];

    if (screen.update) {
        display.clear();

        // 1st line
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, screen.loraDevEui);

        // 2nd line
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 12, screen.loraStatus);

        // 3rd line
        // ...

        // 4th line
        // ..

        display.display();
        screen.update = false;
    }
}


void loop(void)
{
    static unsigned long last_sent = 0;
    static unsigned long button_ts = 0;

    // check for long button press to restart OTAA
    unsigned long ms = millis();
    if (digitalRead(PIN_BUTTON) == 0) {
        if ((ms - button_ts) > 2000) {
            Serial.println("Resetting OTAA");
            LMIC_reset();
            LMIC_startJoining();
            button_ts = ms;
        }
    } else {
        button_ts = ms;
    }

    // time to send a dust measurement?
    unsigned long now = millis() / 1000;
    if ((now - last_sent) > TX_INTERVAL) {
        last_sent = now;
        // try to send a packet
        // ...
    }
    // update screen
    screen_update();

    // ?
    os_runloop_once();
}
