# Note: Sensor status is a bitmask
uint16 SENSOR_STATUS_HMB_CTRL = 1
uint16 SENSOR_STATUS_OUTSIDE = 2
uint16 SENSOR_STATUS_COLLISION = 4
uint16 SENSOR_STATUS_LIFTED = 8
uint16 SENSOR_STATUS_TOO_STEEP = 16
uint16 SENSOR_STATUS_PARKED = 32
uint16 SENSOR_STATUS_IN_CS = 64
uint16 SENSOR_STATUS_USER_STOP = 128
uint16 SENSOR_STATUS_CFG_NEEDED = 256
uint16 SENSOR_STATUS_DISC_ON = 512
uint16 SENSOR_STATUS_LOOP_ON = 1024
uint16 SENSOR_STATUS_CHARGING = 2048

uint16 OPERATIONAL_MODE_OFFLINE = 0
uint16 OPERATIONAL_MODE_MANUAL = 1
uint16 OPERATIONAL_MODE_RANDOM = 2

uint16 MOWER_INTERNAL_STATUS_OFF = 0
uint16 MOWER_INTERNAL_STATUS_WAIT_SAFETY_PIN = 1
uint16 MOWER_INTERNAL_STATUS_STOPPED = 2
uint16 MOWER_INTERNAL_STATUS_FATAL_ERROR = 3
uint16 MOWER_INTERNAL_STATUS_PENDING_START = 4
uint16 MOWER_INTERNAL_STATUS_PAUSED = 5
uint16 MOWER_INTERNAL_STATUS_IN_OPERATION = 6
uint16 MOWER_INTERNAL_STATUS_RESTRICTED = 7
uint16 MOWER_INTERNAL_STATUS_ERROR = 8

uint16 CONTROL_STATE_UNDEFINED = 0
uint16 CONTROL_STATE_IDLE = 1
uint16 CONTROL_STATE_INIT = 2
uint16 CONTROL_STATE_MANUAL = 3
uint16 CONTROL_STATE_RANDOM = 4
uint16 CONTROL_STATE_PARK = 5

Header header
uint16 sensorStatus
uint16 operationalMode
uint16 mowerInternalState
uint16 controlState

