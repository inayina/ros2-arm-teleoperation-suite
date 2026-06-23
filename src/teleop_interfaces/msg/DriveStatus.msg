# State of a single CANopen DS402 virtual servo drive (L4).

std_msgs/Header header

uint8 node_id           # CANopen node id (1..7)
uint16 statusword       # DS402 statusword (0x6041)
uint16 controlword      # last DS402 controlword (0x6040)

# DS402 finite state machine
uint8 STATE_NOT_READY_TO_SWITCH_ON=0
uint8 STATE_SWITCH_ON_DISABLED=1
uint8 STATE_READY_TO_SWITCH_ON=2
uint8 STATE_SWITCHED_ON=3
uint8 STATE_OPERATION_ENABLED=4
uint8 STATE_QUICK_STOP_ACTIVE=5
uint8 STATE_FAULT_REACTION_ACTIVE=6
uint8 STATE_FAULT=7
uint8 ds402_state

uint16 fault_code       # EMCY error code, 0 = no fault
uint8 mode_of_operation # DS402 0x6060 (e.g. 10 = cyclic sync torque)

# Feedback (engineering units)
float64 actual_position # rad
float64 actual_velocity # rad/s
float64 actual_torque   # N*m
float64 target_torque   # N*m commanded via RPDO
