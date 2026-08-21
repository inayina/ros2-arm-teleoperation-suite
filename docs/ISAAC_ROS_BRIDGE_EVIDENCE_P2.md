# Isaac–ROS Bridge Evidence Gate (P2)

**Status:** `PASS — interface evidence only`  
**Executed:** 2026-08-21  
**Scope:** no checkpoint, no policy node, no learned-policy rollout, no Task GT claim.

## Gate result

The bounded full-stack probe received valid data for all required topics within
15 seconds:

- raw Isaac `/isaac/joint_states` (9 positions);
- canonical `/sim/encoder_state` (7 positions), `/ee_pose` in `panda_link0`,
  `/gripper/state`, scene RGB and wrist RGB;
- control-state `/joint_states` (7 positions).

The separate E1 verifier also exercised the non-policy
`/sim/joint_effort_cmd` command boundary for five deterministic repeats. Its
backend watchdog later observed a stale command and applied zero effort.

## Evidence

The versioned compact evidence summary is
[ISAAC_ROS_BRIDGE_EVIDENCE_P2.json](ISAAC_ROS_BRIDGE_EVIDENCE_P2.json).
The raw JSON files are retained locally under the ignored `evidence/` runtime
directory; their SHA256 values below bind this summary to that run output.

| Artifact | SHA256 | What it proves |
| --- | --- | --- |
| `fullstack_control_state_topics.json` | `26b4d0e7c9776bef4554d23390c441f0320599bd346b38326cfa782327e0bb1c` | raw → canonical policy inputs plus `/joint_states` control-state freshness |
| `e1_policy_input_topics.json` | `113b97744bab866e0c9453b158a7d3e51d2a65e901de49ef972b79f328d9ff4e` | isolated raw → canonical policy-input topic contract |
| `e1_nonpolicy_command_path.json` | `ffbbd2de8d1f949e1dceaaacfb435a45dcce9f4de4a5c82deabef72707df8364` | deterministic non-policy effort command path and watchdog behavior |

## Interpretation and stop rule

This closes `ISAAC_ROS_BRIDGE_EVIDENCE_REQUIRED` as an interface closure. It
does **not** retroactively turn the earlier B attempt into a valid learned-policy
sample, prove remote checkpoint identity, execute a learned command, establish
task success, or authorize a new B rollout. No learned-policy rerun, seed
expansion, retraining, collection, geometry tuning, or controller change follows
from this gate.
