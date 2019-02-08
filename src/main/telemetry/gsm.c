/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/streambuf.h"
#include "common/utils.h"
#include "common/printf.h"
#include "common/string_light.h"

#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/fc_init.h"

#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/diagnostics.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/pitotmeter.h"

#include "telemetry/gsm.h"
#include "telemetry/telemetry.h"

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_GSM)

static serialPortConfig_t *portConfig;
static serialPort_t *gsmPort;
static bool gsmEnabled = false;

static uint8_t atCommand[GSM_AT_COMMAND_MAX_SIZE];
static uint8_t gsmResponse[GSM_RESPONSE_BUFFER_SIZE + 1];
static uint32_t gsmResponseIndex = 0;
static uint8_t* gsmResponseValue = NULL;
static int gsmTelemetryState = GSM_NONE;
static timeUs_t gsmNextTime = 0;
static timeUs_t failsafeTime = 0;
static bool gsmWaitAfterResponse = false;
static bool readingSMS = false;
static bool sendingSMS = false;
static int gsmRssi = 0;

bool checkGroundStationNumber(uint8_t* rv) {
    if (strlen(telemetryConfig()->gsmGroundStationNumber) == 0) return false;
    for (int i = 0; i < MAX_GSM_LENGTH && rv[i] != '\"'; i++)
        if (telemetryConfig()->gsmGroundStationNumber[i] != rv[i]) return false;
    return true;
}

void sendATCommand(const char* command) {
    int len = strlen((char*)command);
    if (len > GSM_AT_COMMAND_MAX_SIZE) len = GSM_AT_COMMAND_MAX_SIZE;
    serialWriteBuf(gsmPort, (const uint8_t*) command, len);
}

void handleGsmTelemetry(void) {
    if (!gsmEnabled) return;
    if (!gsmPort) return;
    uint32_t now = millis();
    while (serialRxBytesWaiting(gsmPort) > 0) {
        uint8_t c = serialRead(gsmPort);
        if (gsmResponseIndex < GSM_RESPONSE_BUFFER_SIZE) {
            if (c == '\n') {
                gsmResponse[gsmResponseIndex] = '\0';
                if (gsmResponseIndex > 0) gsmResponseIndex--;
                if (gsmResponse[gsmResponseIndex] == '\r') gsmResponse[gsmResponseIndex] = '\0';
                gsmResponseIndex = 0; //data ok
                break;
            } else {
                gsmResponse[gsmResponseIndex] = c;
                gsmResponseIndex++;
            }
        } else {
            gsmResponse[gsmResponseIndex] = '\0';
            gsmResponseIndex = 0; //data ok
            break;
        }
    }
    if (gsmResponse[0] != '\0' && gsmResponseIndex == 0) {
        if (readingSMS) {
            readingSMS = false;
            if (sl_strcasecmp((char*)gsmResponse,GSM_SMS_COMMAND_TELEMETRY) == 0) sendingSMS = true;
            else if (sl_strcasecmp((char*)gsmResponse,GSM_SMS_COMMAND_RTH) == 0) activateForcedRTH();
            else if (sl_strcasecmp((char*)gsmResponse,GSM_SMS_COMMAND_ABORT_RTH) == 0) abortForcedRTH();
        }
        if (gsmResponse[0] == 'O' && gsmResponse[1] == 'K') {
            if (!gsmWaitAfterResponse) gsmNextTime = now + 500;
        } else if (gsmResponse[0] == 'E' && gsmResponse[1] == 'R') {
            if (!gsmWaitAfterResponse) gsmNextTime = now + 500;
        } else if (gsmResponse[0] == '+') {
            int i;
            for (i = 0; i < GSM_RESPONSE_BUFFER_SIZE && gsmResponse[i] != ':'; i++);
            if (gsmResponse[i] == ':') {
                gsmResponseValue = &gsmResponse[i+2];
                if (gsmResponse[1] == 'C' && gsmResponse[2] == 'S') { //+CSQ: 26,0
                    int j;
                    for (j = i+2; j < GSM_RESPONSE_BUFFER_SIZE && gsmResponse[j] != ','; j++);
                    gsmResponse[j] = '\0';
                    gsmRssi = atoi((char*)gsmResponseValue);
                } else if (gsmResponse[1] == 'C' && gsmResponse[2] == 'L') { //+CLIP: "ddxxxxxxxxx",145,,,,1
                    if (checkGroundStationNumber(&gsmResponse[8])) {
                        gsmNextTime = now + 5000;
                        sendingSMS = true;
                    }
                } else if (gsmResponse[1] == 'C' && gsmResponse[2] == 'M') { //+CMT: "+ddxxxxxxxxx",,"yyyy/MM/dd,hh:mm:ss+xx"<CR><LF>T
                    if (checkGroundStationNumber(&gsmResponse[8])) readingSMS = true; // next gsmResponse line will be SMS content
                }
            }
        }
        gsmResponse[0] = '\0';
    }
    if (!(FLIGHT_MODE(FAILSAFE_MODE) && ARMING_FLAG(WAS_EVER_ARMED))) failsafeTime = now + 10000;
    if (sendingSMS || failsafeTime < now) {
        sendingSMS = false;
        failsafeTime = now + 60000;
        gsmTelemetryState = GSM_SEND_SMS; 
    }
    if (now < gsmNextTime) return;
    gsmNextTime = now + 5000;      //by default, if OK or ERROR not received, wait this long
    gsmWaitAfterResponse = false;  //by default, if OK or ERROR received, go to next state immediately.
    switch (gsmTelemetryState) {
    case GSM_NONE:
        gsmTelemetryState = GSM_INIT_AT;
        gsmNextTime = now + 10000;
        break;
    case GSM_INIT_AT: sendATCommand("AT\r\n");
        gsmTelemetryState = GSM_INIT_ECHO;
        break;
    case GSM_INIT_ECHO: sendATCommand("ATE0\r\n");
        gsmTelemetryState = GSM_INIT_SIM;
        break;
    case GSM_INIT_SIM: sendATCommand("AT+CPBS=\"SM\"\r\n");
        gsmTelemetryState = GSM_INIT_ENTER_PIN;
        break;
    case GSM_INIT_ENTER_PIN: sendATCommand("AT+CPIN=0000\r\n");
        gsmTelemetryState = GSM_INIT_SET_SMS;
        break;
    case GSM_INIT_SET_SMS: sendATCommand("AT+CMGF=1\r\n");
        gsmTelemetryState = GSM_INIT_SET_SMS_RECEIVE;
        break;
    case GSM_INIT_SET_SMS_RECEIVE: sendATCommand("AT+CNMI=3,2\r\n");
        gsmTelemetryState = GSM_INIT_SET_CLIP;
        break;
    case GSM_INIT_SET_CLIP: sendATCommand("AT+CLIP=1\r\n");
        gsmTelemetryState = GSM_READ_SMS;
        break;
    case GSM_READ_SMS: sendATCommand("AT+CMGL=\"ALL\"\r\n");
        gsmTelemetryState = GSM_DELETE_SMS;
        gsmNextTime = now + 5000;
        break;
    case GSM_DELETE_SMS: sendATCommand("AT+CMGD=1,1\r\n");
        gsmTelemetryState = GSM_CHECK_SIGNAL;
        gsmNextTime = now + 5000;
        break;
    case GSM_CHECK_SIGNAL: sendATCommand("AT+CSQ\r\n");
        gsmTelemetryState = GSM_INIT_AT;
        gsmNextTime = now + 10000;
        gsmWaitAfterResponse = true;
        break;
    case GSM_SEND_SMS: sendATCommand("AT+CMGS=\"");
        sendATCommand((char*)telemetryConfig()->gsmGroundStationNumber);
        sendATCommand("\"\r");
        gsmTelemetryState = GSM_SEND_SMS_ENTER;
        gsmNextTime = now + 100;
        break;
    case GSM_SEND_SMS_ENTER: {
        uint16_t vbat = getBatteryVoltage();
        int32_t E7 = 10000000;
        int32_t lat = 0, lon = 0, alt = 0, gs = 0;
        if (sensors(SENSOR_GPS)) {
            lat = gpsSol.llh.lat;
            lon = gpsSol.llh.lon;
            alt = gpsSol.llh.alt / 100; //cm -> m
            gs = gpsSol.groundSpeed / 100; //cm/s -> m/s
        }
#if defined(USE_NAV)
        alt = getEstimatedActualPosition(Z) / 100; //cm -> m
#endif
        int len = tfp_sprintf((char*)atCommand,  //\x1a sends msg, \x1b cancels
          "VBAT:%d.%d ALT:%ld DIST:%d SPEED:%ld TDIST:%ld AVGSPD:%ld SATS:%d GSM:%d google.com/maps/dir/%ld.%07ld,%ld.%07ld/@%ld.%07ld,%ld.%07ld,500m\x1a",
          vbat / 100, vbat % 100, alt, GPS_distanceToHome, gs, getTotalTravelDistance() / 100, lrintf(calculateAverageSpeed()), gpsSol.numSat, gsmRssi, 
          lat / E7, lat % E7, lon / E7, lon % E7, lat / E7, lat % E7, lon / E7, lon % E7);
        serialWriteBuf(gsmPort, atCommand, len);
        gsmTelemetryState = GSM_CHECK_SIGNAL; 
        break; }
    }
}

void freeGsmTelemetryPort(void) {
    closeSerialPort(gsmPort);
    gsmPort = NULL;
    gsmEnabled = false;
}

void initGsmTelemetry(void) {
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_GSM);
}

void checkGsmTelemetryState(void) {
    if (gsmEnabled) return;
    if (!portConfig) return;
    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    gsmPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_GSM, NULL, NULL, baudRates[baudRateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!gsmPort) return;
    gsmEnabled = true;
}

#endif
