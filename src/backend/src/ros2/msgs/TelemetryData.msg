# Purpose: Real-time telemetry data message definition for agricultural robots
# Dependencies: std_msgs 4.2.3, geometry_msgs 4.2.3

# Standard ROS 2 header for timestamp and frame information
std_msgs/Header header

# Device identification and type constants
string device_id

# Telemetry type constants
uint8 TELEMETRY_TYPE_LOCATION = 0
uint8 TELEMETRY_TYPE_BATTERY = 1
uint8 TELEMETRY_TYPE_SPEED = 2
uint8 TELEMETRY_TYPE_ALTITUDE = 3
uint8 TELEMETRY_TYPE_HEADING = 4
uint8 TELEMETRY_TYPE_SENSOR = 5
uint8 TELEMETRY_TYPE_STATUS = 6

# Status constants
uint8 STATUS_OPERATIONAL = 0
uint8 STATUS_WARNING = 1
uint8 STATUS_ERROR = 2
uint8 STATUS_CRITICAL = 3

# Message fields
uint8 type                      # Type of telemetry data using above constants
time timestamp                  # High-precision timestamp for the telemetry data
float64 value_numeric          # Numeric value for the telemetry reading
string value_string            # String value for status messages or additional data
geometry_msgs/Point location   # 3D position data (x, y, z coordinates)
float32 battery_level         # Battery percentage (0.0 to 100.0)
uint8 status                  # Current status using status constants
string metadata              # JSON-formatted metadata for extensibility