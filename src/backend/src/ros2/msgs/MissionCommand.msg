# Version: 1.0.0
# Purpose: Coordinates autonomous drone and ground robot operations for agricultural missions

# External message dependencies
# std_msgs/Header v4.2.0 - Standard ROS message header
# geometry_msgs/Point v4.2.0 - 3D point coordinates

# Standard header for temporal synchronization and coordinate frame reference
std_msgs/Header header

# Unique identifier for the mission
string mission_id

# Command type constants for mission control
uint8 COMMAND_TYPE_START=1      # Initiate new mission
uint8 COMMAND_TYPE_PAUSE=2      # Temporarily halt mission
uint8 COMMAND_TYPE_RESUME=3     # Continue paused mission
uint8 COMMAND_TYPE_ABORT=4      # Terminate mission
uint8 COMMAND_TYPE_COMPLETE=5   # Mark mission as completed
uint8 COMMAND_TYPE_EMERGENCY=6  # Emergency protocol activation
uint8 COMMAND_TYPE_RECALIBRATE=7 # Sensor recalibration
uint8 COMMAND_TYPE_RETURN_HOME=8 # Return to base station

# Current command type
uint8 command_type

# System constants
uint8 VERSION=1                 # Message format version
uint32 MAX_DEVICES=24          # Maximum number of devices in fleet
uint32 MAX_POINTS=1000         # Maximum coverage area points

# Array of device identifiers for fleet management
string[] device_ids

# Array of 3D points defining mission coverage area
geometry_msgs/Point[] coverage_area

# Scheduled mission start time
time start_time

# JSON-encoded mission parameters
string parameters

# Message format version for compatibility
uint8 version

# Message integrity verification
uint32 checksum