# ROS 2 message definition for real-time device status and telemetry data
# Version: 1.0.0
# Purpose: Comprehensive monitoring and fleet coordination for agricultural robots

# Standard ROS 2 message header with timestamp and frame information
std_msgs/Header header

# Device identification
string device_id                      # Unique identifier for the device
string device_name                    # Human-readable device name

# Robot classification
uint8 robot_type                      # Primary robot type (aerial/ground)
uint8 robot_subtype                   # Specific robot configuration

# Robot type constants
uint8 ROBOT_TYPE_AERIAL_DRONE=0       # Aerial drone classification
uint8 ROBOT_TYPE_GROUND_ROBOT=1       # Ground robot classification

# Operational status
uint8 status                          # Current operational state
uint8 ROBOT_STATUS_ACTIVE=0           # Currently executing a mission
uint8 ROBOT_STATUS_IDLE=1             # Available for task assignment
uint8 ROBOT_STATUS_CHARGING=2         # Recharging batteries
uint8 ROBOT_STATUS_MAINTENANCE=3      # Under maintenance or repair
uint8 ROBOT_STATUS_ERROR=4            # Experiencing operational issues

# Power and resource metrics
float32 battery_level                 # Battery level percentage (0.0-100.0)

# Spatial positioning
geometry_msgs/Point position          # 3D position coordinates
geometry_msgs/Quaternion orientation  # 3D orientation quaternion

# Operational capabilities
uint8[] capabilities                  # Bitmap of device capabilities
uint8 ROBOT_CAPABILITY_SPRAY=1        # Capability for precision spraying
uint8 ROBOT_CAPABILITY_MONITOR=2      # Capability for monitoring
uint8 ROBOT_CAPABILITY_SURVEY=4       # Capability for aerial survey
uint8 ROBOT_CAPABILITY_HARVEST=8      # Capability for harvesting operations

# Temporal tracking
time last_active                      # Timestamp of last activity

# Diagnostic information
uint16 error_code                     # Specific error code if status is ERROR
string diagnostic_info                # Additional diagnostic information