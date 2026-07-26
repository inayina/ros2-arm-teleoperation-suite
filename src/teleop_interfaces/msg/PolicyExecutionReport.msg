# Execution decision audit record for panda_policy_runtime_v1.
# M1 defines the interface; authoritative publication begins with the M2 adapter.

std_msgs/Header header
builtin_interfaces/Time received_stamp

string contract_version
string event_id
string parent_event_id
string trace_run_id
string episode_id
string validity
string reason_code

uint64 command_sequence
bool accepted
string decision
string source_action_schema_version
string execution_action_schema_version
float64[] source_action
bool has_bounded_action
float64[] bounded_action
bool clipped
string[] clip_axes
bool hold_active
bool estop_active
string adapter_name
string adapter_version

# Contract invariant: publishers must leave this false.
bool claims_task_success
