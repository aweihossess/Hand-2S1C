#include <Arduino.h>
#include <driver/spi_master.h>

namespace {
constexpr int kMiso = 47;
constexpr int kMosi = 38;
constexpr int kSclk = 48;
constexpr int kCs = 7;
constexpr int kMuxA = 2;
constexpr int kMuxB = 4;
constexpr int kMuxC = 5;
constexpr uint8_t kWords = 4;
constexpr uint16_t kAngleCom = 0x3FFF;

spi_device_handle_t gSpi = nullptr;

uint8_t parity(uint16_t value) {
    uint8_t result = 0;
    while (value) {
        result ^= 1;
        value &= value - 1;
    }
    return result;
}

uint16_t readCommand(uint16_t address) {
    uint16_t frame = address | 0x4000;
    if (parity(frame & 0x7FFF)) {
        frame |= 0x8000;
    }
    return frame;
}

uint16_t swap16(uint16_t value) {
    return static_cast<uint16_t>((value >> 8) | (value << 8));
}

void selectMux(uint8_t channel) {
    digitalWrite(kMuxA, channel & 0x01);
    digitalWrite(kMuxB, (channel >> 1) & 0x01);
    digitalWrite(kMuxC, (channel >> 2) & 0x01);
    delayMicroseconds(100);
}

bool transferMisoMux(uint8_t channel, uint16_t *rx) {
    uint16_t tx[kWords];
    const uint16_t command = swap16(readCommand(kAngleCom));
    for (uint8_t i = 0; i < kWords; ++i) {
        tx[i] = command;
        rx[i] = 0;
    }

    selectMux(channel);
    spi_transaction_t transaction = {};
    transaction.length = kWords * 16;
    transaction.tx_buffer = tx;
    transaction.rx_buffer = rx;

    digitalWrite(kCs, LOW);
    delayMicroseconds(2);
    const esp_err_t result = spi_device_polling_transmit(gSpi, &transaction);
    delayMicroseconds(2);
    digitalWrite(kCs, HIGH);
    delayMicroseconds(10);
    return result == ESP_OK;
}

bool transferCsDemux(uint8_t channel, uint16_t *rx) {
    uint16_t tx[kWords];
    const uint16_t command = swap16(readCommand(kAngleCom));
    for (uint8_t i = 0; i < kWords; ++i) {
        tx[i] = command;
        rx[i] = 0;
    }

    // Y0..Y4 are real finger groups. Y7 is unused and acts as the idle
    // selection, so changing Y7 -> target -> Y7 generates both CS edges.
    selectMux(7);
    selectMux(channel);
    spi_transaction_t transaction = {};
    transaction.length = kWords * 16;
    transaction.tx_buffer = tx;
    transaction.rx_buffer = rx;
    const esp_err_t result = spi_device_polling_transmit(gSpi, &transaction);
    selectMux(7);
    delayMicroseconds(10);
    return result == ESP_OK;
}

void printFrames(const char *label, uint8_t channel, const uint16_t *received, bool ok) {
    Serial.printf("%s%u %s |", label, channel, ok ? "SPI_OK" : "SPI_FAIL");
    for (uint8_t i = 0; i < kWords; ++i) {
        const uint16_t raw = swap16(received[i]);
        const bool parityOk = parity(raw & 0x7FFF) == ((raw >> 15) & 0x01);
        Serial.printf(" %04X(angle=%5u,p=%c,e=%u)",
                      raw,
                      raw & 0x3FFF,
                      parityOk ? 'Y' : 'N',
                      (raw >> 14) & 0x01);
    }
    Serial.println();
}

void scanMisoMux(uint8_t channel) {
    uint16_t discarded[kWords] = {};
    uint16_t received[kWords] = {};
    const bool primeOk = transferMisoMux(channel, discarded);
    const bool readOk = transferMisoMux(channel, received);
    printFrames("MISO", channel, received, primeOk && readOk);
}

void scanCsDemux(uint8_t channel) {
    uint16_t discarded[kWords] = {};
    uint16_t received[kWords] = {};
    const bool primeOk = transferCsDemux(channel, discarded);
    const bool readOk = transferCsDemux(channel, received);
    printFrames("CS  ", channel, received, primeOk && readOk);
}
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n[EncoderProbe] Nano ESP32 / GPIO numbering");
    Serial.printf("[EncoderProbe] SCLK=%d MISO=%d MOSI=%d CS=%d MUX=%d,%d,%d\n",
                  kSclk, kMiso, kMosi, kCs, kMuxA, kMuxB, kMuxC);

    pinMode(kCs, OUTPUT);
    digitalWrite(kCs, HIGH);
    pinMode(kMuxA, OUTPUT);
    pinMode(kMuxB, OUTPUT);
    pinMode(kMuxC, OUTPUT);

    spi_bus_config_t bus = {};
    bus.mosi_io_num = kMosi;
    bus.miso_io_num = kMiso;
    bus.sclk_io_num = kSclk;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 32;

    const esp_err_t busResult = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    Serial.printf("[EncoderProbe] spi_bus_initialize=%d\n", static_cast<int>(busResult));

    spi_device_interface_config_t device = {};
    device.mode = 1;
    device.clock_speed_hz = 1000000;
    device.spics_io_num = -1;
    device.queue_size = 1;
    const esp_err_t deviceResult = spi_bus_add_device(SPI2_HOST, &device, &gSpi);
    Serial.printf("[EncoderProbe] spi_bus_add_device=%d\n", static_cast<int>(deviceResult));
}

void loop() {
    if (Serial.available() && Serial.read() == 'R') {
        Serial.println("[EncoderProbe] software reset requested");
        Serial.flush();
        delay(100);
        esp_restart();
    }

    Serial.printf("\n[EncoderProbe] scan t=%lu ms\n", millis());
    Serial.println("[EncoderProbe] shared-CS / MISO-mux scan");
    for (uint8_t channel = 0; channel < 8; ++channel) {
        scanMisoMux(channel);
    }
    Serial.println("[EncoderProbe] 74HC138 CS-demux scan (Y7 idle)");
    for (uint8_t channel = 0; channel < 5; ++channel) {
        scanCsDemux(channel);
    }
    delay(1000);
}
