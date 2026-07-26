# Structured mirror of upstream task ground truth for panda_policy_runtime_v1.
# Safety/risk must never overwrite or aggregate this status.

std_msgs/Header header
builtin_interfaces/Time received_stamp

string contract_version
string event_id
string parent_event_id
string trace_run_id
string episode_id
string validity
string reason_code
string phase
string task_status

# ROS 2 messages have no nullable scalar. Map JSON null/false/true to these
# tri-state values at the boundary.
int8 OUTCOME_UNKNOWN=-1
int8 OUTCOME_FALSE=0
int8 OUTCOME_TRUE=1
int8 reach
int8 grasp
int8 lift
int8 place

bool has_object_delta
float64 object_delta_x_m
float64 object_delta_y_m
float64 object_delta_z_m
string gt_source

# Both fields are contract invariants and must remain false.
bool risk_may_override
bool claims_task_success
