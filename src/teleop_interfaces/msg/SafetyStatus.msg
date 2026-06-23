# Aggregated safety state published by safety_monitor (L1).

std_msgs/Header header

# Overall verdict
bool ok                 # true only if every monitor passes and no e-stop
bool estop_active       # latched emergency-stop state

# Severity level
uint8 LEVEL_OK=0
uint8 LEVEL_WARN=1
uint8 LEVEL_ERROR=2
uint8 LEVEL_ESTOP=3
uint8 level

# Per-monitor pass flags
bool joint_limit_ok
bool workspace_ok
bool velocity_ok
bool comm_ok

# Human-readable active fault tags, e.g. "joint_limit:panda_joint4", "watchdog:teleop"
string[] active_faults
