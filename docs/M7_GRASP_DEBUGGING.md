# M7 Grasp Debugging: Physics-Only Gripper Tuning

本文档记录 M7 抓取调试方法。目标是先把夹爪调到能够随接触、抬升、滑移改变夹持力度，再录制作品集 GIF。调试阶段不要反复刷新 `media/m7/*.gif`。

## 当前结论

上一轮 GIF 暴露的问题是正确的：夹爪不是稳定真实夹住，而是位置命令直接追到全闭。旧模型里 `gripper_motor` 是 position actuator，收到 `/teleop/gripper_cmd = 0.0` 后会继续追完全闭合；如果物体被挤出，夹爪仍然会空闭到接近 `0.0`。这不符合物理抓取。

当前修正方向：

- `grasp_assist_enabled:=false` 作为物理抓取调试默认值。任何 `grasp_assist_attached=true` 都不能算真实抓取。
- MuJoCo 夹爪 actuator 增加有限力限制，避免无限刚度位置伺服硬压。
- MuJoCo 节点增加 contact hold：检测到指尖-物体接触后，`gripper_cmd` 可以继续是 0，但 `gripper_effective_cmd` 会保持在接触开度附近，不再直接压到全闭。
- MuJoCo 节点增加 adaptive force debug 字段，用于判断夹爪力度是否随抓取阶段变化。
- 关键根因已经确认：`forcerange` 只是力上限，不等于实际夹持力。MuJoCo position actuator 的实际输出约等于 `kp * position_error`，再被 `forcerange` 截断。旧参数 `kp=350` 且 contact hold 只压入约 `0.004 m`，实际夹持力只有约 `1.4 N`，所以看起来有 force limit，但物体仍然会滑掉。
- 当前有效参数是 `kp=4000`、`gripper_contact_hold_margin=0.006`、`gripper_force_squeeze_margin_max=0.006`、`gripper_force_max_n=30.0`，并把 M7 pick 高度降到 `pick_height_offset=0.015`。也就是接触后不闭死，但允许约 `6 mm` 有限压入产生真实法向夹持力，再由 adaptive force 把上限抬高。

## 手柄控制边界

手柄不直接控制摩擦。摩擦是物体和指尖材料的物理属性，应该通过 MuJoCo geom 参数体现；手柄只表达夹爪开合意图。

真实抓取里更合理的控制边界是：

1. 操作者用手柄发出开合目标，例如 `/teleop/gripper_cmd`。
2. 程序根据接触状态把目标转换成 `gripper_effective_cmd`，接触后停在物体尺寸对应的开度附近。
3. 程序根据抓取阶段调整 `gripper_force_limit_n`，自由闭合低力，接触后加力，抬升或下滑时补力。
4. 摩擦参数只决定同样夹持力下是否容易打滑，不能替代夹持力闭环。

所以“直接闭死”是不合格的，即使画面看起来短暂夹到了也不能算稳定真实抓取。

## 为什么需要本机 UDP socket

ROS2 默认通信层是 DDS。即使所有节点都在同一台机器上，DDS 也会创建本机 UDP socket，用来做节点发现、topic 匹配和数据传输。这不是联网访问外部服务器，也不是为了模拟吸附；它只是 ROS2 节点之间通信的底层机制。

因此在受限沙箱里跑会出现 `TRANSPORT_UDP Error: Operation not permitted`。这种日志说明 ROS2 通信没有正常起来，不能用来判断抓取物理是否成功。有效验证必须在允许本机 UDP socket 的环境里跑。

## 最新有效调试与 GIF 结论

`2026-07-04` 的最终无 GIF 通过日志是 `.m7_run_logs/m7_force_debug_pick15_squeeze6_hold8.log`。同一组稳定参数随后录制到 `.m7_run_logs/m7_grasp_final_gif.log`，并刷新：

- `media/m7/grasp_demo.gif`：`2026-07-04 16:00:39 +0800`，`640x480`，`338` 帧。
- `media/m7/gripper_closeup.gif`：`2026-07-04 16:00:32 +0800`，`320x240`，`382` 帧。

batch 结束前的关键结果：

- 没有使用模拟吸附：`grasp_assist_attached=False`。
- 没有直接闭死：录制版 batch 结束前 `gripper_effective_cmd ~= 0.496`，`gripper_opening ~= 0.642`，没有追到 `0.0`。
- 真实压入量受限：`gripper_squeeze_m=0.006`，不是靠无限位置伺服硬夹。
- 力度随抓取状态变化：`gripper_force_mode=contact_drop_boost`，`gripper_force_limit_n=30.0`。
- 物体没有在 episode 内掉回桌面：无 GIF 验证中物体最高到 `object_pos.z ~= 0.049`，batch 结束前保持在 `object_pos.z ~= 0.042`；桌面稳定高度约 `0.024`。
- 指尖接触没有在 episode 内丢失：batch 结束前 `finger_object_contacts=21`，`lost_contact` 行数为 `0`。
- 轨迹参数已固化到 `scripts/capture_m7_demo.sh` 默认值：`pick_height_offset=0.015`、`close_duration=3.0`、`grasp_pause=3.0`、`lift_duration=10.0`、`lift_target_z=0.075`、`post_lift_hold=8.0`。

注意：日志里 `Killing ROS 2 launch...` 之后出现的下落是关机阶段的 transient，不能作为本轮抓取失败证据。判断成功与否只看 batch 结束前、kill 之前的数据。

最终录制版在 batch 结束前同样保持 `finger_object_contacts=21`、`gripper_force_limit_n=30.0`、`gripper_squeeze_m=0.006`，可作为当前 M7 物理抓取演示基线。

## 旧失败日志记录

`2026-07-04 14:57` 的无 GIF 日志 `.m7_run_logs/m7_force_debug.log` 说明：

- 已跳过 GIF 录制：日志包含 `Skipping GIF recorder (M7_RECORD_GIF=false)`，`media/m7/*.gif` 没有被刷新。
- 没有使用模拟吸附：关键行里 `grasp_assist_attached=False`。
- 直接闭死问题已被压住：闭合命令到 `gripper_cmd=0.000` 后，`gripper_effective_cmd=0.687`，`gripper_contact_hold_target=0.687`，夹爪没有继续追到 0。
- 力度已经开始随阶段变化：接触时 `gripper_force_mode=hold`，`gripper_force_limit_n` 约 `7.90-8.17`；轻微抬升时出现 `lift_hold`。
- 仍未达到录制门槛：抬升后 `finger_object_contacts` 从 `14` 降到 `7`，随后掉到 `0`，物体回到桌面高度 `object_pos.z ~= 0.024`。当前失败主因是抬升阶段接触丢失后没有及时重新收紧并补力，不是 GIF 录制问题。

`2026-07-04 15:01` 的 `.m7_run_logs/m7_force_debug_regrip.log` 是沙箱限制导致的无效验证：ROS2 DDS 报 `TRANSPORT_UDP Error: Operation not permitted`，控制器没有 active，因此不能用于判断抓取是否成功。

后续几轮也暴露出两个要点：

- 只把 `gripper_force_max_n` 提高到 `30 N` 不够，因为旧 `kp` 和 hold margin 太小，position actuator 没有产生足够实际夹持力。
- `gripper_contact_drop` 只能用于补力，不能直接触发持续 regrip 闭合；否则画面会再次接近“夹爪闭死”，即使物体短时没掉也不合格。
- `gripper_squeeze_m` 太小会慢滑，太大会把物体侧向挤出。验证边界是：约 `4 mm` 会在长 hold 中慢滑，`7-8 mm` 会挤飞，当前稳定值约 `6 mm`。
- `pick_height_offset=0.020` 抓取偏上，8 秒 hold 末尾容易掉到 `object_pos.z ~= 0.026-0.028`。`pick_height_offset=0.015` 会把接触位置降到更稳定的区域。

## 只跑调试，不录 GIF

调试阶段使用：

```bash
M7_RECORD_GIF=false \
M7_GRASP_ASSIST=false \
M7_CONTACT_DEBUG=true \
bash scripts/capture_m7_demo.sh > .m7_run_logs/m7_force_debug.log 2>&1
```

这会运行同一套 M7 episode，但跳过 `media/m7/grasp_demo.gif` 和 `media/m7/gripper_closeup.gif` 录制。

只有满足本文档的录制门槛后，才使用：

```bash
M7_RECORD_GIF=true \
M7_GRASP_ASSIST=false \
M7_CONTACT_DEBUG=true \
bash scripts/capture_m7_demo.sh
```

## 关键日志字段

核心日志行来自 `mujoco_sim`：

```text
M7 grasp debug ...
```

必须关注这些字段：

| 字段 | 含义 | 期望 |
|---|---|---|
| `grasp_assist_attached` | 是否启用/触发模拟吸附 | 真实抓取必须始终为 `False` |
| `finger_object_contacts` | 指尖与目标物体的 MuJoCo contact 数量 | 闭合后应大于 0，抬升保持阶段不能掉到 0 |
| `gripper_cmd` | 上层命令开度，0=闭合，1=张开 | 调试中通常会到 0 |
| `gripper_effective_cmd` | MuJoCo 实际使用的夹爪目标 | 接触后不应继续跟着 `gripper_cmd` 到 0 |
| `gripper_contact_hold_target` | 接触保持开度 | 接触后应变为非空 |
| `gripper_force_limit_n` | 当前夹爪 actuator force limit | 应从 approach 低力升到 hold/lift/slip 更高力 |
| `gripper_force_mode` | 当前夹爪力度模式 | 应出现 `approach`、`hold`、`lift_hold`、`contact_drop_boost` 或 `slip_boost` |
| `object_z_velocity` | 目标物体 z 方向速度 | 下滑时为负，触发 `slip_boost` |
| `object_pos` | 目标物体位置 | 抬升后 z 应高于桌面高度，保持段不应掉回桌面 |
| `gripper_contact_drop` | 当前接触数相对峰值下降量 | 可用于补力，但不应直接导致闭死 |
| `gripper_squeeze_m` | 当前开度到有效命令的有限压入量 | 当前稳定值约 `0.006 m`，过大容易挤飞 |

快速检查：

```bash
rg "gripper_effective_cmd|gripper_force_limit_n|gripper_squeeze_m|finger_object_contacts|grasp_assist_attached" \
  .m7_run_logs/m7_force_debug.log
```

## 录制门槛

不要因为某一帧看起来夹到了就录 GIF。必须先满足下面条件：

1. `grasp_assist_attached=False` 全程成立。
2. 闭合后 `finger_object_contacts > 0`。
3. 接触后 `gripper_cmd=0.000` 时，`gripper_effective_cmd` 保持在非零接触开度附近。
4. `gripper_force_limit_n` 随阶段变化：approach 低，hold/lift 更高；若接触数下降或发生下滑，应进入 `contact_drop_boost` 或 `slip_boost`。
5. 抬升后物体高度不回到桌面高度。当前桌面稳定高度大约是 `object_pos.z ~= 0.024`。
6. `/grasp/status` 不能出现 `ASSISTED_GRASP`；最终应进入物理 `SUCCESS`，否则继续调参。

当前最终日志和 GIF 已满足 MuJoCo episode 内的物理接触与保持证据。

## 推荐调参顺序

先不要调摩擦参数。优先调夹爪控制逻辑：

1. `gripper_force_min_n`：自由闭合阶段的低力。太大容易把物体挤飞；太小接触后夹不住。
2. `gripper_force_contact_gain_n`：接触数增加时的加力幅度。
3. `gripper_force_lift_gain_n_per_m`：物体被抬起后补力幅度。
4. `gripper_force_slip_boost_n`：检测到下滑时的临时补力。
5. `gripper_contact_hold_margin`：接触后允许继续压入的开度余量。
6. `gripper_force_squeeze_margin_max`：把期望 force 映射成 position actuator 的有限压入量。太小会滑，太大会挤飞。
7. `pick_height_offset`：抓取位置比单纯加力更关键。当前稳定值是 `0.015`。
8. `lift_target_z` 和 `lift_duration`：先低高度慢抬，等稳定后再提高演示高度。
9. `gripper_motor` 的 `kp`：它决定 position error 能转换成多少实际夹持力；只调 `forcerange` 不一定有用。

接触几何和摩擦参数只作为第二层调优。它们可以帮助稳定，但不能替代闭环夹持力。

## 当前实现位置

- 夹爪 MuJoCo actuator：`config/models/franka_panda.xml`
- 夹爪 contact hold 与 adaptive force：`src/mujoco_sim/mujoco_sim/mujoco_sim_node.py`
- M7 调试/录制入口：`scripts/capture_m7_demo.sh`
- 抓取状态判定：`src/grasp_monitor/grasp_monitor/grasp_monitor_node.py`
- 物理抓取验收脚本：`scripts/validate_m7_grasp_monitor.sh`

## 不再接受的演示证据

- 只看 GIF 说抓住了。
- `grasp_assist_attached=true` 的抓取。
- 夹爪最终空闭到 0，但物体已经掉回桌面的画面。
- 只靠增大摩擦或减轻物体让单次 GIF 看起来成功。

可接受证据必须同时包含日志字段和最终 GIF。日志先过，GIF 后录。
