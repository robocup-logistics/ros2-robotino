
# MotorError.msg

std_msgs/Header header

uint8 NO_ERROR = 0
uint8 ENCODER_FAILURE = 1
uint8 DRIVE_FAILURE = 2
uint8 LOW_VOLTAGE = 3
uint8 UNKNOWN_ERROR = 255

# Arrays to represent each wheel's state
string[] name # Wheel identifier
bool[] error_status      # True if the corresponding wheel has an error
uint8[] error_code       # Error code for each wheel
string[] error_msg   # Error message for each wheel
