# M1 shadow command for panda_policy_runtime_v1.
# This message is telemetry until M4B explicitly transfers execution authority.

std_msgs/Header header
builtin_interfaces/Time received_stamp
builtin_interfaces/Time valid_until

string contract_version
string event_id
string parent_event_id
string trace_run_id
string episode_id
string validity
string reason_code

uint64 observation_sequence
uint64 command_sequence
string action_schema_version
float64[] action
uint32 chunk_index
uint32 chunk_size
bool from_prefetched_chunk
float64 inference_latency_ms

# Contract invariant: publishers must leave this false.
bool claims_task_success
