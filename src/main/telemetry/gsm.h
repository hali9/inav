#include "io/gps.h"
#include "io/serial.h"

#define GSM_AT_COMMAND_MAX_SIZE 256
#define GSM_RESPONSE_BUFFER_SIZE 256

#define GSM_SMS_COMMAND_TELEMETRY 	"T"
#define GSM_SMS_COMMAND_RTH 		"RTH"
#define GSM_SMS_COMMAND_ABORT_RTH 	"-RTH"

typedef enum  {
    GSM_NONE = 0,
    GSM_INIT_AT,
    GSM_INIT_ECHO,
    GSM_INIT_SIM,
    GSM_INIT_SET_SMS,
    GSM_INIT_SET_SMS_RECEIVE,
    GSM_INIT_SET_CLIP,
    GSM_INIT_ENTER_PIN,
    GSM_DIAL,
    GSM_DIAL_HANGUP,
    GSM_READ_SMS,
    GSM_DELETE_SMS,
    GSM_SEND_SMS,
    GSM_SEND_SMS_ENTER,
    GSM_CHECK_SIGNAL,
    GSM_CHECK_SIGNAL2
} gsmTelemetryState_e;

bool checkGroundStationNumber(uint8_t*);
void sendATCommand(const char* command);
void handleGsmTelemetry();
void freeGsmTelemetryPort(void);
void initGsmTelemetry(void);
void checkGsmTelemetryState(void);
