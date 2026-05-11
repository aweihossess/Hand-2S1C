#include "HandCalibrationNvs.h"

#include <Arduino.h>
#include <Preferences.h>

namespace {

constexpr uint32_t kCalibNvsMagic = 0x43414C42u; // 'CALB'
constexpr const char* kNs = "handcal";
constexpr const char* kKeyMagic = "magic";
constexpr const char* kKeyMask = "mask";
constexpr const char* kKeyEnc = "enc";
constexpr const char* kKeyMot = "mot";

} // namespace

void handCalibrationNvsLoad(TaskSharedData_t* sd)
{
    if (!sd) {
        return;
    }
    Preferences prefs;
    if (!prefs.begin(kNs, true)) {
        return;
    }
    if (prefs.getUInt(kKeyMagic, 0) != kCalibNvsMagic) {
        prefs.end();
        return;
    }
    const uint8_t mask = prefs.getUChar(kKeyMask, 0);
    if (mask & 1u) {
        const size_t encLen = prefs.getBytesLength(kKeyEnc);
        if (encLen == sizeof(sd->calib_zero_raw_cache)) {
            prefs.getBytes(kKeyEnc, sd->calib_zero_raw_cache, sizeof(sd->calib_zero_raw_cache));
            sd->calib_zero_raw_valid = 1;
        }
    }
    if (mask & 2u) {
        const size_t motLen = prefs.getBytesLength(kKeyMot);
        if (motLen == sizeof(sd->mechanism_zero_motor_abs)) {
            prefs.getBytes(kKeyMot, sd->mechanism_zero_motor_abs, sizeof(sd->mechanism_zero_motor_abs));
            sd->mechanism_zero_motor_valid = 1;
        }
    }
    prefs.end();
}

void handCalibrationNvsSave(TaskSharedData_t* sd)
{
    if (!sd) {
        return;
    }
    Preferences prefs;
    if (!prefs.begin(kNs, false)) {
        Serial.println("[HandCalibrationNvs] NVS begin( RW ) failed");
        return;
    }

    uint8_t mask = 0;
    if (sd->calib_zero_raw_valid) {
        prefs.putBytes(kKeyEnc, sd->calib_zero_raw_cache, sizeof(sd->calib_zero_raw_cache));
        mask |= 1u;
    }
    if (sd->mechanism_zero_motor_valid) {
        prefs.putBytes(kKeyMot, sd->mechanism_zero_motor_abs, sizeof(sd->mechanism_zero_motor_abs));
        mask |= 2u;
    } else {
        prefs.remove(kKeyMot);
    }

    if (mask == 0) {
        prefs.end();
        return;
    }

    prefs.putUChar(kKeyMask, mask);
    prefs.putUInt(kKeyMagic, kCalibNvsMagic);
    prefs.end();
    Serial.printf("[HandCalibrationNvs] saved mask=%u (1=enc 2=mot)\n", (unsigned)mask);
}
