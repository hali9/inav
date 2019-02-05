#include "build/build_config.h"

#include "common/axis.h"
#include "common/streambuf.h"
#include "common/utils.h"
#include "common/printf.h"

#include "drivers/serial.h"
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
#include "drivers/serial_uart.h"

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

#include "drivers/light_led.h"

#include "common/string_light.h"

#include <stdlib.h>
#include <string.h>

#include "build/debug.h"

static serialPort_t *gsmPort;
static serialPortConfig_t *portConfig;
static bool gsmEnabled = false;

static uint8_t atCommand[GSM_AT_COMMAND_MAX_SIZE];
static int gsmTelemetryState = GSM_STATE_INIT;
static timeUs_t gsmNextTime = 0;
static uint8_t gsmResponse[GSM_RESPONSE_BUFFER_SIZE + 1];
static int atCommandStatus = GSM_AT_OK;
static uint8_t* gsmResponseValue = NULL;
static bool gsmWaitAfterResponse = false;
static bool readingSMS = false;
static int gsmRssi = 0;

bool isGroundStationNumberDefined() {
    return strlen(telemetryConfig()->gsmGroundStationNumber) == 0;
}

void readGsmResponse()
{
    DEBUG_SET(DEBUG_GSM, 0, 0);
    if (readingSMS) {
        readingSMS = false;
        readSMS();
        DEBUG_SET(DEBUG_GSM, 0, 1);
    }
    if (gsmResponse[0] == 'O' && gsmResponse[1] == 'K') {
        atCommandStatus = GSM_AT_OK;
        if (!gsmWaitAfterResponse) {
            gsmNextTime = millis() + GSM_AT_COMMAND_DELAY_MIN_MS;
        }
        DEBUG_SET(DEBUG_GSM, 0, 2);
        return;
    } else if (gsmResponse[0] == 'E' && gsmResponse[1] == 'R') {
        atCommandStatus = GSM_AT_ERROR;
        if (!gsmWaitAfterResponse) {
            gsmNextTime = millis() + GSM_AT_COMMAND_DELAY_MIN_MS;
        }
        DEBUG_SET(DEBUG_GSM, 0, 3);
        return;
    } else if (gsmResponse[0] == 'R' && gsmResponse[1] == 'I') {        
        if (isGroundStationNumberDefined()) {
            gsmTelemetryState = GSM_STATE_SEND_SMS;
        }
    } else if (gsmResponse[0] == '+') {
        int i;
        for (i = 0; i < GSM_RESPONSE_BUFFER_SIZE && gsmResponse[i] != ':'; i++);
        if (gsmResponse[i] == ':') {
            gsmResponseValue = &gsmResponse[i+2];
            readGsmResponseData();
        }
        DEBUG_SET(DEBUG_GSM, 0, 4);
        return;
    }
}

void readGsmResponseData()
{
    if (gsmResponse[1] == 'C' && gsmResponse[2] == 'S') { // +CSQ: 26,0
        gsmRssi = atoi((char*)gsmResponseValue);
        DEBUG_SET(DEBUG_GSM, 0, 5);
    } else if (gsmResponse[1] == 'C' && gsmResponse[2] == 'L') { // +CLIP: "3581234567"
        DEBUG_SET(DEBUG_GSM, 0, 6);
        readOriginatingNumber(&gsmResponse[8]);        
    } else if (gsmResponse[1] == 'C' && gsmResponse[2] == 'M') { // +CMT: <oa>,[<alpha>],<scts>[,<tooa>,<fo>,<pid>,<dcs>,<sca>,<tosca>,<length>]<CR><LF><data>
        DEBUG_SET(DEBUG_GSM, 0, 7);
        readOriginatingNumber(&gsmResponse[7]);
        readingSMS = true; // next gsmResponse line will be SMS content
    }
}

bool readOriginatingNumber(uint8_t* rv)
{
    int i;
    if (!isGroundStationNumberDefined()) {
        DEBUG_SET(DEBUG_GSM, 0, 8);
        return false;
    }
    for (i = 0; i < MAX_GSM_LENGTH && rv[i] != '\"'; i++) {
         if (telemetryConfig()->gsmGroundStationNumber[i] != rv[i]) {
             DEBUG_SET(DEBUG_GSM, 0, 9);
             return false;
         }
    }
    DEBUG_SET(DEBUG_GSM, 0, 10);
    return true;
}

void readSMS()
{
    if (sl_strcasecmp((char*)gsmResponse,GSM_SMS_COMMAND_TELEMETRY) == 0) {
        gsmTelemetryState = GSM_STATE_SEND_SMS;
        DEBUG_SET(DEBUG_GSM, 0, 11);
    } else if (sl_strcasecmp((char*)gsmResponse,GSM_SMS_COMMAND_RTH) == 0) {
        DEBUG_SET(DEBUG_GSM, 0, 12);
        activateForcedRTH();
    } else if (sl_strcasecmp((char*)gsmResponse,GSM_SMS_COMMAND_ABORT_RTH) == 0) {
        DEBUG_SET(DEBUG_GSM, 0, 13);
        abortForcedRTH();
    }
}

void handleGsmTelemetry()
{
    DEBUG_SET(DEBUG_GSM, 1, 0);
    static int ri = 0;
    uint32_t now = millis();
    if (!gsmEnabled) return;
    if (!gsmPort) return;
    uint8_t c;
    for (; serialRxBytesWaiting(gsmPort); ri++) {
        DEBUG_SET(DEBUG_GSM, 1, 1);
        c = serialRead(gsmPort);
        gsmResponse[ri] = c;
        if (c == '\n' || ri == GSM_RESPONSE_BUFFER_SIZE) {
            gsmResponse[ri] = '\0'; //response line expected to end in \r\n, remove them
            if (ri > 0) gsmResponse[--ri] = '\0';
            readGsmResponse();
            ri = 0;
            return;
        }
    }
    if (now < gsmNextTime) return;
    gsmNextTime = now + GSM_AT_COMMAND_DELAY_MS; //by default, if OK or ERROR not received, wait this long
    gsmWaitAfterResponse = false;                //by default, if OK or ERROR received, go to next state immediately.
    DEBUG_SET(DEBUG_GSM, 2, gsmTelemetryState);
    switch (gsmTelemetryState) {
    case GSM_STATE_INIT:
        DEBUG_TRACE_SYNC("GSM INIT");
        sendATCommand("AT\n");
        gsmTelemetryState = GSM_STATE_INIT2;
        break;
    case GSM_STATE_INIT2:
        sendATCommand("AT+CPBS=\"SM\"\n");
        gsmTelemetryState = GSM_STATE_INIT_ENTER_PIN;
        break;
    case GSM_STATE_INIT_ENTER_PIN:
        sendATCommand("AT+CPIN=0000\n");
        gsmTelemetryState = GSM_STATE_INIT_SET_SMS_MODE;
        break;
    case GSM_STATE_INIT_SET_SMS_MODE:
        sendATCommand("AT+CMGF=1\n");
        gsmTelemetryState = GSM_STATE_INIT_SET_SMS_RECEIVE_MODE;
        break;
    case GSM_STATE_INIT_SET_SMS_RECEIVE_MODE:
        sendATCommand("AT+CNMI=3,2\n");
        gsmTelemetryState = GSM_STATE_INIT_SET_CLIP;
        break;
    case GSM_STATE_INIT_SET_CLIP:
        sendATCommand("AT+CLIP=1\n");
        gsmTelemetryState = GSM_STATE_CHECK_SIGNAL;
        break;
    case GSM_STATE_READ_SMS:
        sendATCommand("AT+CMGL=\"ALL\"\n");
        gsmNextTime = now + 5000;
        gsmTelemetryState = GSM_STATE_DELETE_SMS;
        break;
    case GSM_STATE_DELETE_SMS:
        sendATCommand("AT+CMGD=1,4\n"); //delete all messages, max response time 5 s
        gsmNextTime = now + 5000;
        gsmTelemetryState = GSM_STATE_CHECK_SIGNAL;
        break;
    case GSM_STATE_CHECK_SIGNAL:
        sendATCommand("AT+CSQ\n");
        gsmNextTime = now + GSM_CYCLE_MS;
        gsmWaitAfterResponse = true;
        gsmTelemetryState = GSM_STATE_INIT;
        break;
    case GSM_STATE_SEND_SMS:
        sendATCommand("AT+CMGS=\"");
        sendATCommand((char*)telemetryConfigMutable()->gsmGroundStationNumber);
        sendATCommand("\"\r");
        gsmTelemetryState = GSM_STATE_SEND_SMS_ENTER_MESSAGE;
        gsmNextTime = now + 100;
        break;
    case GSM_STATE_SEND_SMS_ENTER_MESSAGE:
        sendSMS();
        gsmTelemetryState = GSM_STATE_CHECK_SIGNAL;
        break;
    }
    DEBUG_SET(DEBUG_GSM, 3, gsmTelemetryState);
}

void sendATCommand(const char* command)
{
    atCommandStatus = GSM_AT_WAITING_FOR_RESPONSE;
    int len = strlen((char*)command);
    if (len > GSM_AT_COMMAND_MAX_SIZE) len = GSM_AT_COMMAND_MAX_SIZE;
    serialWriteBuf(gsmPort, (const uint8_t*) command, len);
}

void sendSMS()
{
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
    uint16_t vbat = getBatteryVoltage();
    int32_t avgSpeed = (int)round(10 * calculateAverageSpeed());
    int32_t E7 = 10000000;
    // \x1a sends msg, \x1b cancels
    int len = tfp_sprintf((char*)atCommand, 
      "VBAT:%d.%d ALT:%ld DIST:%d SPEED:%ld TDIST:%ld AVGSPD:%d.%d SATS:%d GSM:%d google.com/maps/@%ld.%07ld,%ld.%07ld,500m\x1a",
      vbat / 100, vbat % 100, alt, GPS_distanceToHome, gs, getTotalTravelDistance() / 100, avgSpeed / 10, avgSpeed % 10, 
      gpsSol.numSat, gsmRssi, lat / E7, lat % E7, lon / E7, lon % E7);
    serialWriteBuf(gsmPort, atCommand, len);
    atCommandStatus = GSM_AT_WAITING_FOR_RESPONSE;
}

void freeGsmTelemetryPort(void)
{
    closeSerialPort(gsmPort);
    gsmPort = NULL;
    gsmEnabled = false;
}

void initGsmTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_GSM);
}

void checkGsmTelemetryState(void)
{
    if (gsmEnabled) {
        return;
    }
    configureGsmTelemetryPort();
}

void configureGsmTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }
    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    gsmPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_GSM, NULL, NULL, baudRates[baudRateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!gsmPort) {
        return;
    }
    gsmEnabled = true;
}
