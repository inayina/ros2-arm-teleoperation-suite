#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger, SetBool
from moveit_msgs.srv import ServoCommandType
from teleop_interfaces.srv import EndEpisode
import tf2_ros
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

from synth_data_gen.motion_primitives import (
    compute_twist_linear,
    position_error,
    update_max_tracking_error,
)

class BatchGenerator(Node):
    def __init__(self):
        super().__init__('batch_generator')
        self.declare_parameter('episodes', 10)
        self.declare_parameter('seed', 42)
        self.declare_parameter('hover_duration', 3.0)
        self.declare_parameter('descend_duration', 2.5)
        self.declare_parameter('close_duration', 1.0)
        self.declare_parameter('grasp_pause', 1.0)
        self.declare_parameter('lift_duration', 2.0)
        self.declare_parameter('lift_target_z', 0.0)
        self.declare_parameter('post_lift_hold', 0.0)
        self.declare_parameter('hover_height', 0.45)
        self.declare_parameter('pick_height_offset', 0.05)
        self.declare_parameter('gripper_close_target', 0.06)
        self.declare_parameter('reach_confirm_frames', 12)
        self.declare_parameter('pose_step_m', 0.006)
        self.declare_parameter('pose_cmd_rate_hz', 100.0)
        self.declare_parameter('reset_timeout', 5.0)
        self.declare_parameter('target_object_name', '')
        self.declare_parameter('language_instruction', '')
        self.declare_parameter('target_bin_y', 999.0)
        self.declare_parameter('validation_mode', 'place')
        self.declare_parameter('max_attempts_per_episode', 5)
        self.declare_parameter('fail_on_max_attempts', True)
        self.declare_parameter('lift_success_delta', 0.02)
        self.declare_parameter('bin_xy_tolerance', 0.14)
        self.declare_parameter('validation_settle_s', 1.0)
        self.declare_parameter('recorder_settle_s', 5.0)
        self.declare_parameter('record_warmup_s', 2.0)
        self.declare_parameter('use_ready_pose', True)
        self.declare_parameter('ee_xy_tolerance', 0.10)
        self.declare_parameter('ee_z_tolerance', 0.04)
        self.declare_parameter('ee_arrival_timeout_s', 12.0)
        self.declare_parameter('motion_mode', 'pose')
        self.declare_parameter('twist_max_linear_mps', 0.05)
        self.declare_parameter('twist_descend_linear_mps', 0.04)
        self.declare_parameter('ready_pose', [0.45, 0.0, 0.55])
        self.declare_parameter('ee_tracking_tolerance_m', 0.08)
        
        self.episodes = self.get_parameter('episodes').value
        self.seed = self.get_parameter('seed').value
        self.hover_duration = float(self.get_parameter('hover_duration').value)
        self.descend_duration = float(self.get_parameter('descend_duration').value)
        self.close_duration = float(self.get_parameter('close_duration').value)
        self.grasp_pause = float(self.get_parameter('grasp_pause').value)
        self.lift_duration = float(self.get_parameter('lift_duration').value)
        self.lift_target_z = float(self.get_parameter('lift_target_z').value)
        self.post_lift_hold = float(self.get_parameter('post_lift_hold').value)
        self.hover_height = float(self.get_parameter('hover_height').value)
        self.pick_height_offset = float(self.get_parameter('pick_height_offset').value)
        self.gripper_close_target = float(self.get_parameter('gripper_close_target').value)
        self.pose_step_m = float(self.get_parameter('pose_step_m').value)
        self.reach_confirm_frames = max(
            1, int(self.get_parameter('reach_confirm_frames').value)
        )
        self.pose_cmd_rate_hz = float(self.get_parameter('pose_cmd_rate_hz').value)
        self.reset_timeout = float(self.get_parameter('reset_timeout').value)
        self.target_object_name = str(self.get_parameter('target_object_name').value).strip()
        self.language_instruction = str(self.get_parameter('language_instruction').value).strip()
        self.target_bin_y = float(self.get_parameter('target_bin_y').value)
        self.validation_mode = str(self.get_parameter('validation_mode').value).strip().lower()
        self.max_attempts_per_episode = max(
            1, int(self.get_parameter('max_attempts_per_episode').value)
        )
        self.fail_on_max_attempts = bool(self.get_parameter('fail_on_max_attempts').value)
        self.lift_success_delta = float(self.get_parameter('lift_success_delta').value)
        self.bin_xy_tolerance = float(self.get_parameter('bin_xy_tolerance').value)
        self.validation_settle_s = float(self.get_parameter('validation_settle_s').value)
        self.recorder_settle_s = float(self.get_parameter('recorder_settle_s').value)
        self.record_warmup_s = float(self.get_parameter('record_warmup_s').value)
        self.use_ready_pose = bool(self.get_parameter('use_ready_pose').value)
        self.ee_xy_tolerance = float(self.get_parameter('ee_xy_tolerance').value)
        self.ee_z_tolerance = float(self.get_parameter('ee_z_tolerance').value)
        self.ee_arrival_timeout_s = float(self.get_parameter('ee_arrival_timeout_s').value)
        self.motion_mode = str(self.get_parameter('motion_mode').value).strip().lower()
        self.twist_max_linear_mps = float(self.get_parameter('twist_max_linear_mps').value)
        self.twist_descend_linear_mps = float(self.get_parameter('twist_descend_linear_mps').value)
        self.ready_pose = [float(v) for v in self.get_parameter('ready_pose').value]
        self.ee_tracking_tolerance_m = float(self.get_parameter('ee_tracking_tolerance_m').value)
        
        self.initial_pose = None
        self.latest_object_pose = None
        self.latest_ee_pose = None
        self._trial_initial_object_z = None
        self._trial_initial_object_xyz = None
        self._trial_max_object_z = None
        self._trial_max_ee_tracking_error = 0.0
        self._gripper_hold_value = None
        self._pick_height_offset_default = 0.05
        self._last_cmd_ori = [1.0, 0.0, 0.0, 0.0]
        self._cmd_pos = None
        self._pick_offset_by_shape = {
            "sphere": 0.006,
            "cylinder": 0.010,
            "box": 0.012,
        }
        # Workspace for teaching picks (Panda table-top). Out-of-range poses usually mean
        # the object was ejected / tracking drifted and should trigger a scene re-reset.
        self._object_workspace_xy = (0.20, 0.60, -0.25, 0.25)
        self._object_workspace_z = (0.00, 0.20)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.cli_reset = self.create_client(Trigger, '/sim/reset_scene')
        self.cli_safety_reset = self.create_client(Trigger, '/safety/reset')
        self.pub_rec = self.create_publisher(String, '/teleop/record_trigger', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/teleop/cmd_pose', 10)
        self.pub_twist = self.create_publisher(TwistStamped, '/teleop/cmd_twist', 10)
        self.pub_grip = self.create_publisher(Float64, '/teleop/gripper_cmd', 10)
        self.sub_object = self.create_subscription(
            PoseStamped, '/sim/object_pose', self._on_object_pose, 10)
        self.sub_ee = self.create_subscription(
            PoseStamped, '/ee_pose', self._on_ee_pose, 10)
        self.cli_pause_servo = self.create_client(SetBool, '/servo_node/pause_servo')
        self.cli_switch_servo = self.create_client(
            ServoCommandType, '/servo_node/switch_command_type')
        self.cli_end_episode = self.create_client(EndEpisode, '/lerobot_recorder/end_episode')
        
        # Start run_batch thread
        self.get_logger().info(f'Batch generator ready. Running {self.episodes} episodes.')
        import threading
        threading.Thread(target=self.run_batch).start()

    def run_batch(self):
        task_list = self._task_list()
        BIN_X = 0.4
        BIN_Z = 0.12  # 放置高度（筐上方）

        # Wait for tf to become available
        while True:
            try:
                trans = self.tf_buffer.lookup_transform('panda_link0', 'panda_ee', rclpy.time.Time())
                p = Pose()
                p.position.x = trans.transform.translation.x
                p.position.y = trans.transform.translation.y
                p.position.z = trans.transform.translation.z
                p.orientation = trans.transform.rotation
                
                # Validate the TF lookup. If it deviates significantly from nominal home, fallback.
                dist = math.hypot(p.position.x - 0.307, p.position.y - 0.0)
                if dist > 0.05 or abs(p.position.z - 0.490) > 0.05:
                    self.get_logger().warn(
                        f"TF lookup initial pose pos=[{p.position.x:.3f}, {p.position.y:.3f}, {p.position.z:.3f}] "
                        f"deviates from nominal home pose. Falling back to nominal home."
                    )
                    p.position.x = 0.307
                    p.position.y = 0.0
                    p.position.z = 0.490
                    p.orientation.w = 0.0
                    p.orientation.x = 1.0
                    p.orientation.y = 0.0
                    p.orientation.z = 0.0
                
                self.initial_pose = p
                self.get_logger().info(
                    f"TF LOOKUP initial_pose: pos=[{p.position.x:.3f}, {p.position.y:.3f}, {p.position.z:.3f}], "
                    f"ori=[{p.orientation.w:.3f}, {p.orientation.x:.3f}, {p.orientation.y:.3f}, {p.orientation.z:.3f}]"
                )
                break
            except Exception as e:
                self.get_logger().info(f'Wait for tf... {e}')
                time.sleep(0.5)

        success_count = 0
        total_attempts = 0
        attempts_for_current = 0
        while success_count < self.episodes:
            # 循环选取当前任务
            task_cfg = task_list[success_count % len(task_list)]
            target_obj, instruction, bin_y = task_cfg
            attempts_for_current += 1
            total_attempts += 1

            self.get_logger().info(
                f'--- Starting Episode {success_count+1}/{self.episodes} '
                f'| Attempt {attempts_for_current}/{self.max_attempts_per_episode} '
                f'| Task: {target_obj} ---'
            )

            # 0. 更新仿真器目标物体参数（在 Reset 之前，使仿真器切换追踪目标）
            target_set = self._set_node_parameter('/mujoco_sim', 'target_object_name', target_obj)
            # 更新录制器语言指令与 upstream gate（供中游识别物理评测边界）
            language_set = self._set_node_parameter(
                '/lerobot_recorder', 'language_instruction', instruction, timeout=8.0
            )
            if not language_set:
                # One retry: service often stalls briefly after the previous episode commit.
                time.sleep(0.5)
                language_set = self._set_node_parameter(
                    '/lerobot_recorder', 'language_instruction', instruction, timeout=8.0
                )
            gate_set = self._set_node_parameter('/lerobot_recorder', 'upstream_gate', 'batch_generator')

            # 1. Reset Scene（触发多物体随机化）
            reset_ok = self._reset_scene(timeout=self.reset_timeout)

            # Let the scene settle and recover arm if reset left it far from home.
            time.sleep(1.0)
            self._hold_nominal_home(duration=4.0)
            object_pose = self._wait_for_reachable_object_pose(timeout=4.0)
            if object_pose is None:
                self.get_logger().warn(
                    'Object pose missing/out of workspace after reset; re-resetting scene once'
                )
                reset_ok = self._reset_scene(timeout=self.reset_timeout) and reset_ok
                time.sleep(1.0)
                self._hold_nominal_home(duration=3.0)
                object_pose = self._wait_for_reachable_object_pose(timeout=4.0)
            self._start_trial_tracking(object_pose)

            try:
                trans = self.tf_buffer.lookup_transform('panda_link0', 'panda_ee', rclpy.time.Time())
                p_x = trans.transform.translation.x
                p_y = trans.transform.translation.y
                p_z = trans.transform.translation.z
                
                # Validate the TF lookup.
                dist = math.hypot(p_x - 0.307, p_y - 0.0)
                if dist <= 0.05 and abs(p_z - 0.490) <= 0.05:
                    self.initial_pose.position.x = p_x
                    self.initial_pose.position.y = p_y
                    self.initial_pose.position.z = p_z
                    self.initial_pose.orientation = trans.transform.rotation
                    self.get_logger().info(
                        f"TF LOOKUP attempt initial_pose updated: pos=[{self.initial_pose.position.x:.3f}, {self.initial_pose.position.y:.3f}, {self.initial_pose.position.z:.3f}], "
                        f"ori=[{self.initial_pose.orientation.w:.3f}, {self.initial_pose.orientation.x:.3f}, {self.initial_pose.orientation.y:.3f}, {self.initial_pose.orientation.z:.3f}]"
                    )
                else:
                    self.get_logger().warn(
                        f"TF lookup after reset pos=[{p_x:.3f}, {p_y:.3f}, {p_z:.3f}] deviates from nominal home pose. "
                        f"Keeping previous home reference: pos=[{self.initial_pose.position.x:.3f}, {self.initial_pose.position.y:.3f}, {self.initial_pose.position.z:.3f}]"
                    )
            except Exception as e:
                self.get_logger().warn(f"Failed to update initial pose: {e}")

            # 2. Start Recording (open gripper first so contact hold starts from a known state)
            self._release_gripper_lock()
            self.pub_grip.publish(Float64(data=1.0))
            self.pub_rec.publish(String(data='start'))
            time.sleep(max(0.5, self.record_warmup_s))

            # 3. Execute Pick-Place Motion
            self._request_safety_reset()
            self._ensure_servo_ready()
            start_p, start_q = self._motion_start_from_ee()
            object_xyz = self._object_xyz(object_pose)
            motion_ok = True
            if object_xyz is None or not self._object_pose_in_workspace(object_xyz):
                self.get_logger().warn(
                    f'Skipping motion: object pose unreachable ({object_xyz})'
                )
                motion_ok = False
                object_x, object_y, object_z = 0.4, 0.0, 0.05
            else:
                object_x, object_y, object_z = object_xyz

            down_q = start_q
            ready_p = list(self.ready_pose)
            hover_p = [object_x, object_y, object_z + self.hover_height]
            offset = self._pick_z_offset(target_obj)
            pick_p = [object_x, object_y, max(0.02, object_z + offset)]
            xy_start = ready_p if self.use_ready_pose else start_p
            approach_xy_p = [object_x, object_y, xy_start[2]]

            if motion_ok and self.use_ready_pose:
                self.get_logger().info(
                    f'FSM Phase 0: Ready. Moving from start_p={start_p} to ready_p={ready_p}'
                )
                motion_ok = self._move_toward(
                    start_p, ready_p, duration=self.hover_duration, label='ready'
                )

            self.get_logger().info(
                f'FSM Phase 1a: Approach XY. Moving to approach_xy_p={approach_xy_p}'
            )
            if motion_ok:
                motion_ok = self._move_toward(
                    xy_start,
                    approach_xy_p,
                    duration=self.hover_duration,
                    label='approach_xy',
                    axes='xy',
                )

            self.get_logger().info(
                f'FSM Phase 1b: Hover. Moving to hover_p={hover_p}'
            )
            if motion_ok:
                motion_ok = self._move_toward(
                    approach_xy_p,
                    hover_p,
                    duration=self.hover_duration,
                    label='hover',
                    axes='z',
                )

            # 阶段2：下降至物体抓取高度
            if motion_ok:
                self.get_logger().info(
                    f"FSM Phase 2: Descend. Moving from hover_p={hover_p} to pick_p={pick_p} "
                    f"(object_z={object_z:.3f}, pick_offset={offset:.3f})"
                )
                motion_ok = self._move_toward(
                    hover_p, pick_p, duration=self.descend_duration, label="pick", descend=True, axes='z'
                )

            if motion_ok:
                time.sleep(0.5)
                ee = self._ee_xyz()
                if ee is not None:
                    xy_err = math.hypot(ee[0] - pick_p[0], ee[1] - pick_p[1])
                    if xy_err > self.ee_xy_tolerance:
                        align_p = [pick_p[0], pick_p[1], ee[2]]
                        self.get_logger().info(
                            f'FSM Phase 2b: Pick XY align. err_xy={xy_err:.3f} -> {align_p}'
                        )
                        motion_ok = self._move_toward(
                            list(ee),
                            align_p,
                            duration=max(3.0, self.descend_duration * 0.5),
                            label='pick_xy',
                            axes='xy',
                        )
                        time.sleep(0.3)
                        ee = self._ee_xyz()
                    if motion_ok and ee is not None:
                        z_err = abs(ee[2] - pick_p[2])
                        if z_err > self.ee_z_tolerance:
                            self.get_logger().info(
                                f'FSM Phase 2c: Pick Z trim. err_z={z_err:.3f}'
                            )
                            motion_ok = self._move_toward(
                                list(ee),
                                pick_p,
                                duration=max(2.0, self.descend_duration * 0.4),
                                label='pick_z',
                                axes='z',
                                descend=True,
                            )
                    if motion_ok and not self._confirm_ee_at_target(
                        pick_p, axes='all', timeout=2.0
                    ):
                        self.get_logger().warn(
                            'Pick pose not steady at target after descend/align'
                        )
                        motion_ok = False

            if motion_ok:
                # 阶段3：合拢夹爪抓取
                self.get_logger().info("FSM Phase 3: Close Gripper.")
                self._move_gripper_smooth(1.0, self.gripper_close_target, duration=self.close_duration)
                self._lock_gripper(self.gripper_close_target)
                self._publish_motion_hold(pick_p, down_q, duration=self.grasp_pause)

                # 阶段4：抬升至安全高度
                lift_p = hover_p
                if self.lift_target_z > 0.0:
                    lift_p = [object_x, object_y, self.lift_target_z]
                self.get_logger().info(
                    f"FSM Phase 4: Lift. Moving from pick_p={pick_p} to lift_p={lift_p}"
                )
                motion_ok = self._move_toward(
                    pick_p, lift_p, duration=self.lift_duration, label="lift", axes='z'
                )
            else:
                lift_p = hover_p
                self.get_logger().warn(
                    "FSM motion did not converge before grasp; skipping grasp/lift phases"
                )

            if self.validation_mode == "lift":
                if motion_ok and self.post_lift_hold > 0.0:
                    time.sleep(self.post_lift_hold)
            elif motion_ok:
                # 阶段5：平移至目标筐正上方
                bin_hover_p = [BIN_X, bin_y, lift_p[2]]
                self.get_logger().info(
                    f"FSM Phase 5: Transport. Moving from lift_p={lift_p} to bin_hover_p={bin_hover_p}"
                )
                self._move_toward(lift_p, bin_hover_p, duration=self.hover_duration, label="transport")

                # 阶段6：下降至放置高度
                bin_place_p = [BIN_X, bin_y, BIN_Z]
                self.get_logger().info(
                    f"FSM Phase 6: Place. Moving from bin_hover_p={bin_hover_p} to bin_place_p={bin_place_p}"
                )
                self._move_toward(
                    bin_hover_p, bin_place_p, duration=self.descend_duration, label="place", descend=True
                )

                # 阶段7：松开夹爪释放物体
                self.get_logger().info("FSM Phase 7: Release Gripper.")
                self._release_gripper_lock()
                self._move_gripper_smooth(0.0, 1.0, duration=self.close_duration)
                time.sleep(0.5)

                if self.post_lift_hold > 0.0:
                    time.sleep(self.post_lift_hold)

            # 4. Validate before committing the buffered episode.
            time.sleep(max(0.0, self.validation_settle_s))
            validation = self._validate_episode(
                target_obj=target_obj,
                bin_x=BIN_X,
                bin_y=bin_y,
                target_set=target_set,
                language_set=language_set,
                gate_set=gate_set,
                reset_ok=reset_ok,
                motion_ok=motion_ok,
            )
            if validation["success"]:
                if self._commit_recording():
                    success_count += 1
                    attempts_for_current = 0
                    self.get_logger().info(
                        f'--- Accepted Episode {success_count}/{self.episodes}: {validation["reason"]} ---'
                    )
                else:
                    # Do NOT discard here: a slow image flush can finish the
                    # /end_episode write after our wait deadline. Discarding then
                    # races a successful commit and falsely burns an attempt.
                    self.get_logger().warn(
                        f'--- Rejected Episode {success_count+1}/{self.episodes}: '
                        f'validation passed but recorder commit failed/ambiguous; '
                        f'retrying same task (no discard) ---'
                    )
            else:
                self._discard_recording()
                self.get_logger().warn(
                    f'--- Rejected Episode {success_count+1}/{self.episodes}: '
                    f'{validation["reason"]}; retrying same task ---'
                )
                self._hold_nominal_home(duration=6.0)
                if attempts_for_current >= self.max_attempts_per_episode:
                    msg = (
                        f'Exceeded max_attempts_per_episode={self.max_attempts_per_episode} '
                        f'for task {target_obj}. Last validation: {validation}'
                    )
                    if self.fail_on_max_attempts:
                        self.get_logger().error(msg)
                        self._discard_recording()
                        self._shutdown_batch(exit_code=1)
                        return
                    self.get_logger().warn(msg + ' Skipping this task.')
                    success_count += 1
                    attempts_for_current = 0
            time.sleep(1.0)

            self.get_logger().info(
                f'--- Batch progress: {success_count}/{self.episodes} accepted '
                f'after {total_attempts} attempts ---'
            )

        self.get_logger().info('Batch generation completed successfully.')
        self._shutdown_batch(exit_code=0)

    def _commit_recording(self) -> bool:
        if self._end_episode_via_service(discard=False):
            return True
        self._flush_record_trigger('stop_success')
        return False

    def _discard_recording(self):
        if not self._end_episode_via_service(discard=True):
            self._flush_record_trigger('discard')

    def _end_episode_via_service(self, discard: bool) -> bool:
        if not self.cli_end_episode.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/lerobot_recorder/end_episode unavailable; using topic fallback')
            return False
        req = EndEpisode.Request()
        req.discard = bool(discard)
        future = self.cli_end_episode.call_async(req)
        # Commit blocks inside the recorder on image flush; allow a longer
        # deadline than discard (discard only clears an in-memory buffer).
        settle = max(1.0, float(self.recorder_settle_s))
        if not discard:
            settle = max(settle, 45.0)
        deadline = time.monotonic() + settle
        while True:
            if future.done():
                try:
                    result = future.result()
                    if not result or not result.success:
                        msg = getattr(result, 'message', 'unknown error') if result else 'no response'
                        self.get_logger().warn(f'end_episode rejected: {msg}')
                        return False
                    if discard:
                        self.get_logger().info(
                            f'Recorder discarded episode ({result.frame_count} buffered frames)'
                        )
                        return True
                    if result.dataset_path:
                        self.get_logger().info(
                            f'Recorder committed: {result.dataset_path} '
                            f'({result.frame_count} frames)'
                        )
                        return True
                    self.get_logger().warn(
                        f'end_episode commit missing dataset_path ({result.frame_count} frames)'
                    )
                    return False
                except Exception as exc:
                    self.get_logger().warn(f'end_episode service failed: {exc}')
                    return False
            if time.monotonic() >= deadline:
                if not discard and settle < 90.0:
                    # One extension: large RGBD episodes can flush past 45s.
                    self.get_logger().warn(
                        'end_episode commit still pending; extending wait for image flush...'
                    )
                    settle = 90.0
                    deadline = time.monotonic() + 45.0
                    continue
                self.get_logger().warn('end_episode service timed out; using topic fallback')
                return False
            time.sleep(0.02)

    def _flush_record_trigger(self, command: str):
        deadline = time.monotonic() + max(0.5, float(self.recorder_settle_s))
        while time.monotonic() < deadline:
            self.pub_rec.publish(String(data=command))
            time.sleep(0.15)

    def _shutdown_batch(self, exit_code: int = 0):
        time.sleep(0.5)
        import os
        os._exit(exit_code)

    def _task_list(self):
        defaults = {
            "object_red_box": (
                "pick up the red box and place it in the left bin",
                -0.2,
            ),
            "object_blue_cylinder": (
                "pick up the blue cylinder and place it in the right bin",
                0.2,
            ),
            "object_green_sphere": (
                "pick up the green sphere and place it in the left bin",
                -0.2,
            ),
        }
        if self.target_object_name:
            instruction, bin_y = defaults.get(
                self.target_object_name,
                (f"pick up {self.target_object_name}", -0.2),
            )
            if self.language_instruction:
                instruction = self.language_instruction
            if self.target_bin_y < 900.0:
                bin_y = self.target_bin_y
            return [(self.target_object_name, instruction, bin_y)]
        return [
            ("object_red_box", defaults["object_red_box"][0], defaults["object_red_box"][1]),
            (
                "object_blue_cylinder",
                defaults["object_blue_cylinder"][0],
                defaults["object_blue_cylinder"][1],
            ),
            (
                "object_green_sphere",
                defaults["object_green_sphere"][0],
                defaults["object_green_sphere"][1],
            ),
        ]

    def _request_safety_reset(self, timeout=5.0) -> bool:
        if not self.cli_safety_reset.wait_for_service(timeout_sec=min(1.0, timeout)):
            self.get_logger().warn('/safety/reset unavailable')
            return False
        future = self.cli_safety_reset.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                try:
                    result = future.result()
                    if result and result.success:
                        return True
                    if result:
                        self.get_logger().warn(f'/safety/reset: {result.message}')
                    return False
                except Exception as exc:
                    self.get_logger().warn(f'/safety/reset failed: {exc}')
                    return False
            time.sleep(0.02)
        self.get_logger().warn('/safety/reset timed out')
        return False

    def _reset_scene(self, timeout=5.0):
        if not self.cli_reset.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn('/sim/reset_scene unavailable; continuing without scene reset.')
            return False

        future = self.cli_reset.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                try:
                    result = future.result()
                    if result and result.success:
                        self.get_logger().info(f'/sim/reset_scene: {result.message}')
                        if self.initial_pose:
                            pos = [
                                self.initial_pose.position.x,
                                self.initial_pose.position.y,
                                self.initial_pose.position.z
                            ]
                            ori = [
                                self.initial_pose.orientation.w,
                                self.initial_pose.orientation.x,
                                self.initial_pose.orientation.y,
                                self.initial_pose.orientation.z
                            ]
                            self._move_arm(pos, ori)
                    elif result:
                        self.get_logger().warn(f'/sim/reset_scene failed: {result.message}')
                    return bool(result and result.success)
                except Exception as exc:
                    self.get_logger().warn(f'/sim/reset_scene call failed: {exc}')
                    return False
            time.sleep(0.02)

        self.get_logger().warn('/sim/reset_scene timed out; continuing with current scene.')
        return False

    def _on_object_pose(self, msg):
        self.latest_object_pose = msg
        if self._trial_max_object_z is not None:
            z = float(msg.pose.position.z)
            if math.isfinite(z):
                self._trial_max_object_z = max(self._trial_max_object_z, z)

    def _on_ee_pose(self, msg):
        self.latest_ee_pose = msg

    def _ee_xyz(self):
        if self.latest_ee_pose is None:
            return None
        return (
            float(self.latest_ee_pose.pose.position.x),
            float(self.latest_ee_pose.pose.position.y),
            float(self.latest_ee_pose.pose.position.z),
        )

    def _position_reached(self, ee, target_pos, axes='all'):
        xy_err = math.hypot(ee[0] - target_pos[0], ee[1] - target_pos[1])
        z_err = abs(ee[2] - target_pos[2])
        if axes == 'xy':
            return xy_err <= self.ee_xy_tolerance
        if axes == 'z':
            return z_err <= self.ee_z_tolerance and xy_err <= self.ee_xy_tolerance
        return xy_err <= self.ee_xy_tolerance and z_err <= self.ee_z_tolerance

    def _confirm_ee_at_target(self, target_pos, axes='all', timeout=2.0):
        """Require consecutive /ee_pose samples at target to reject transient false reaches."""
        deadline = time.monotonic() + max(0.2, float(timeout))
        confirm = 0
        needed = self.reach_confirm_frames
        while time.monotonic() < deadline:
            ee = self._ee_xyz()
            if ee is not None and self._position_reached(ee, target_pos, axes=axes):
                confirm += 1
                if confirm >= needed:
                    return True
            else:
                confirm = 0
            time.sleep(0.02)
        return False

    def _wait_for_ee_near(self, target_pos, label="target", timeout=None, hold_ori=None, axes='all'):
        timeout = self.ee_arrival_timeout_s if timeout is None else timeout
        deadline = time.monotonic() + max(0.5, float(timeout))
        last_log = 0.0
        next_pub = 0.0
        hold_ori = hold_ori or self._last_cmd_ori
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_pub:
                self._move_arm(target_pos, hold_ori)
                next_pub = now + 0.02
            ee = self._ee_xyz()
            if ee is not None:
                xy_err = math.hypot(ee[0] - target_pos[0], ee[1] - target_pos[1])
                z_err = abs(ee[2] - target_pos[2])
                if self._position_reached(ee, target_pos, axes=axes):
                    if self._confirm_ee_at_target(
                        target_pos, axes=axes, timeout=0.5
                    ):
                        self.get_logger().info(
                            f"EE reached {label}: err_xy={xy_err:.3f} err_z={z_err:.3f}"
                        )
                        return True
                now = time.monotonic()
                if now - last_log > 2.0:
                    self.get_logger().info(
                        f"Waiting EE {label}: pos={[round(v, 3) for v in ee]} "
                        f"target={[round(v, 3) for v in target_pos]} "
                        f"err_xy={xy_err:.3f} err_z={z_err:.3f}"
                    )
                    last_log = now
            time.sleep(0.05)
        ee = self._ee_xyz()
        if ee is None:
            self.get_logger().warn(f"EE arrival timeout for {label}: no /ee_pose received")
        else:
            xy_err = math.hypot(ee[0] - target_pos[0], ee[1] - target_pos[1])
            z_err = abs(ee[2] - target_pos[2])
            self.get_logger().warn(
                f"EE arrival timeout for {label}: err_xy={xy_err:.3f} err_z={z_err:.3f}"
            )
        return False

    def _wait_for_object_pose(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        last_pose = self.latest_object_pose
        while time.monotonic() < deadline:
            if self.latest_object_pose is not None and self.latest_object_pose is not last_pose:
                return self.latest_object_pose
            if self.latest_object_pose is not None and last_pose is None:
                return self.latest_object_pose
            time.sleep(0.05)
        if self.latest_object_pose is None:
            self.get_logger().warn('No /sim/object_pose received; falling back to nominal pick pose.')
        return self.latest_object_pose

    def _object_pose_in_workspace(self, xyz) -> bool:
        if xyz is None:
            return False
        x_min, x_max, y_min, y_max = self._object_workspace_xy
        z_min, z_max = self._object_workspace_z
        return (
            x_min <= float(xyz[0]) <= x_max
            and y_min <= float(xyz[1]) <= y_max
            and z_min <= float(xyz[2]) <= z_max
        )

    def _wait_for_reachable_object_pose(self, timeout=4.0):
        deadline = time.monotonic() + max(0.5, float(timeout))
        best = None
        while time.monotonic() < deadline:
            pose = self._wait_for_object_pose(timeout=0.4)
            xyz = self._object_xyz(pose)
            if xyz is not None and self._object_pose_in_workspace(xyz):
                return pose
            if xyz is not None:
                best = pose
                self.get_logger().warn(
                    f'Object pose outside workspace: {[round(v, 3) for v in xyz]}'
                )
            time.sleep(0.05)
        if best is not None and self._object_pose_in_workspace(self._object_xyz(best)):
            return best
        return None

    def _start_trial_tracking(self, object_pose):
        xyz = self._object_xyz(object_pose)
        self._trial_max_ee_tracking_error = 0.0
        if xyz is None:
            self._trial_initial_object_z = None
            self._trial_initial_object_xyz = None
            self._trial_max_object_z = None
            return
        self._trial_initial_object_xyz = xyz
        self._trial_initial_object_z = xyz[2]
        self._trial_max_object_z = xyz[2]

    def _current_command_pos(self):
        ee = self._ee_xyz()
        if ee is not None:
            return list(ee)
        if self.latest_ee_pose is not None:
            p = self.latest_ee_pose.pose.position
            return [float(p.x), float(p.y), float(p.z)]
        return [
            float(self.initial_pose.position.x),
            float(self.initial_pose.position.y),
            float(self.initial_pose.position.z),
        ]

    def _sync_cmd_setpoint(self, pos, ori):
        self._cmd_pos = [float(pos[0]), float(pos[1]), float(pos[2])]
        if ori is not None:
            self._last_cmd_ori = [float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3])]

    def _ensure_servo_ready(self, timeout=4.0):
        hold_pos, hold_ori = self._motion_start_from_ee()
        self._sync_cmd_setpoint(hold_pos, hold_ori)
        use_twist = self.motion_mode == 'twist'
        if use_twist:
            self._publish_twist_hold(0.8)
        else:
            self._publish_pose_hold(hold_pos, hold_ori, duration=0.8, rate_hz=self.pose_cmd_rate_hz)

        pause_req = SetBool.Request()
        pause_req.data = False
        self._call_service(self.cli_pause_servo, pause_req, timeout)
        switch_req = ServoCommandType.Request()
        if use_twist:
            switch_req.command_type = getattr(ServoCommandType.Request, 'TWIST', 1)
            mode_label = 'TWIST'
        else:
            switch_req.command_type = getattr(ServoCommandType.Request, 'POSE', 2)
            mode_label = 'POSE'
        if self._call_service(self.cli_switch_servo, switch_req, timeout):
            self.get_logger().info(f'Servo unpaused and set to {mode_label} mode.')
        else:
            self.get_logger().warn('Servo unpause/switch may have failed; motion may stall.')

        if use_twist:
            self._publish_twist_hold(0.5)
        else:
            self._publish_pose_hold(hold_pos, hold_ori, duration=0.5, rate_hz=self.pose_cmd_rate_hz)

    def _call_service(self, client, request, timeout=4.0):
        if not client.wait_for_service(timeout_sec=min(1.0, timeout)):
            return False
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                try:
                    result = future.result()
                    return bool(result and getattr(result, "success", True))
                except Exception:
                    return False
            time.sleep(0.02)
        return False

    def _motion_start_from_ee(self):
        if self.latest_ee_pose is not None:
            p = self.latest_ee_pose.pose.position
            o = self.latest_ee_pose.pose.orientation
            if all(math.isfinite(float(v)) for v in (p.x, p.y, p.z, o.w, o.x, o.y, o.z)):
                start_p = [float(p.x), float(p.y), float(p.z)]
                start_q = [float(o.w), float(o.x), float(o.y), float(o.z)]
                self.get_logger().info(
                    f"Motion start synced from /ee_pose: pos={[round(v, 3) for v in start_p]}"
                )
                return start_p, start_q
        start_p = [
            self.initial_pose.position.x,
            self.initial_pose.position.y,
            self.initial_pose.position.z,
        ]
        start_q = [
            self.initial_pose.orientation.w,
            self.initial_pose.orientation.x,
            self.initial_pose.orientation.y,
            self.initial_pose.orientation.z,
        ]
        return start_p, start_q

    def _pick_z_offset(self, target_obj: str) -> float:
        if abs(self.pick_height_offset - self._pick_height_offset_default) > 1e-6:
            return self.pick_height_offset
        for shape, offset in self._pick_offset_by_shape.items():
            if shape in target_obj:
                return offset
        return self.pick_height_offset

    def _lock_gripper(self, normalized_value: float):
        self._gripper_hold_value = float(normalized_value)
        self.pub_grip.publish(Float64(data=self._gripper_hold_value))

    def _release_gripper_lock(self):
        self._gripper_hold_value = None

    def _publish_pose_hold(self, pos, ori, duration=1.0, rate_hz=50.0):
        duration = max(0.0, float(duration))
        if duration <= 0.0:
            return
        dt = 1.0 / max(1.0, float(rate_hz))
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._move_arm(pos, ori)
            if self._gripper_hold_value is not None:
                self.pub_grip.publish(Float64(data=self._gripper_hold_value))
            time.sleep(dt)

    @staticmethod
    def _object_xyz(object_pose):
        if object_pose is None:
            return None
        return (
            float(object_pose.pose.position.x),
            float(object_pose.pose.position.y),
            float(object_pose.pose.position.z),
        )

    def _validate_episode(
        self,
        target_obj: str,
        bin_x: float,
        bin_y: float,
        target_set: bool,
        language_set: bool,
        gate_set: bool,
        reset_ok: bool,
        motion_ok: bool = True,
    ) -> dict:
        final_pose = self._wait_for_object_pose(timeout=1.0)
        final_xyz = self._object_xyz(final_pose)
        if self.validation_mode in ("none", "off", "disabled"):
            return {"success": True, "reason": "validation disabled"}
        if not target_set:
            return {"success": False, "reason": "failed to set mujoco target_object_name"}
        if not language_set:
            # Metadata only: do not discard a successful physical lift for a
            # transient recorder set_parameters timeout under multi-object load.
            self.get_logger().warn(
                'language_instruction was not confirmed on recorder; continuing with lift gate'
            )
        if not gate_set:
            return {"success": False, "reason": "failed to set recorder upstream_gate"}
        if not reset_ok:
            return {"success": False, "reason": "scene reset failed or timed out"}
        if not motion_ok:
            return {"success": False, "reason": "motion phase did not converge before grasp/lift"}
        if (
            self.motion_mode == "twist"
            and self._trial_max_ee_tracking_error > self.ee_tracking_tolerance_m
        ):
            return {
                "success": False,
                "reason": (
                    f"ee tracking error {self._trial_max_ee_tracking_error:.4f}m exceeds "
                    f"tolerance {self.ee_tracking_tolerance_m:.4f}m"
                ),
            }
        if final_xyz is None or self._trial_initial_object_z is None:
            return {"success": False, "reason": "missing privileged /sim/object_pose"}
        if self._trial_initial_object_xyz is not None and not self._object_pose_in_workspace(
            self._trial_initial_object_xyz
        ):
            return {
                "success": False,
                "reason": (
                    f"initial object pose out of workspace: {self._trial_initial_object_xyz}"
                ),
            }

        max_z = self._trial_max_object_z
        if max_z is None:
            max_z = final_xyz[2]
        lift_delta = float(max_z) - float(self._trial_initial_object_z)
        lifted = lift_delta + 1e-3 >= self.lift_success_delta
        bin_dist = math.hypot(final_xyz[0] - float(bin_x), final_xyz[1] - float(bin_y))
        placed = bin_dist <= self.bin_xy_tolerance

        metrics = (
            f"target={target_obj} lift_delta={lift_delta:.3f}m "
            f"bin_xy_error={bin_dist:.3f}m final_xyz="
            f"({final_xyz[0]:.3f}, {final_xyz[1]:.3f}, {final_xyz[2]:.3f})"
        )
        if self.validation_mode == "lift":
            return {
                "success": lifted,
                "reason": f"lift validation {'passed' if lifted else 'failed'}: {metrics}",
            }
        if self.validation_mode == "place":
            ok = lifted and placed
            return {
                "success": ok,
                "reason": f"place validation {'passed' if ok else 'failed'}: {metrics}",
            }
        return {
            "success": False,
            "reason": f"unknown validation_mode={self.validation_mode!r}",
        }

    def _move_arm_smooth(self, start_pos, end_pos, duration=1.0, start_ori=None, end_ori=None):
        rate_hz = max(1.0, float(self.pose_cmd_rate_hz))
        steps = max(1, int(duration * rate_hz))
        dt = duration / steps
        
        if start_ori is None:
            start_ori = [1.0, 0.0, 0.0, 0.0]
        if end_ori is None:
            end_ori = start_ori
            
        # Ensure shortest path on quaternion sphere
        dot = sum(start_ori[j] * end_ori[j] for j in range(4))
        if dot < 0.0:
            end_ori = [-x for x in end_ori]
            
        for i in range(steps):
            alpha = (i + 1) / float(steps)
            pos = [start_pos[j] * (1.0 - alpha) + end_pos[j] * alpha for j in range(3)]
            
            qw = start_ori[0] * (1.0 - alpha) + end_ori[0] * alpha
            qx = start_ori[1] * (1.0 - alpha) + end_ori[1] * alpha
            qy = start_ori[2] * (1.0 - alpha) + end_ori[2] * alpha
            qz = start_ori[3] * (1.0 - alpha) + end_ori[3] * alpha
            mag = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
            if mag < 1e-6:
                mag = 1.0
            ori = [qw/mag, qx/mag, qy/mag, qz/mag]
            
            self._sync_cmd_setpoint(pos, ori)
            self._move_arm(pos, ori)
            time.sleep(dt)

    def _nominal_home(self):
        if self.initial_pose is not None:
            p = self.initial_pose.position
            o = self.initial_pose.orientation
            return [float(p.x), float(p.y), float(p.z)], [
                float(o.w), float(o.x), float(o.y), float(o.z)
            ]
        return [0.307, 0.0, 0.490], [0.0, 1.0, 0.0, 0.0]

    def _hold_nominal_home(self, duration=3.0):
        home_pos, home_ori = self._nominal_home()
        self._request_safety_reset()
        if self.motion_mode == "pose":
            self._ensure_servo_ready(timeout=3.0)
        self._publish_pose_hold(
            home_pos, home_ori, duration=float(duration), rate_hz=self.pose_cmd_rate_hz
        )
        time.sleep(0.5)

    def _recover_arm_home(self, duration=5.0):
        """Deprecated path: hold home in place instead of Cartesian recovery from bad EE."""
        self._hold_nominal_home(duration=max(3.0, float(duration) * 0.6))

    def _move_toward(self, start_pos, end_pos, duration=1.0, label="target", descend=False, axes='all'):
        actual_start, hold_ori = self._motion_start_from_ee()
        if self.motion_mode == 'twist':
            return self._stream_twist_toward(
                end_pos,
                duration=duration,
                label=label,
                descend=descend,
            )
        move_duration = float(duration)
        if descend:
            move_duration = max(move_duration, float(self.ee_arrival_timeout_s))
        self._sync_cmd_setpoint(actual_start, hold_ori)
        return self._stream_pose_toward(
            end_pos,
            hold_ori,
            duration=move_duration,
            label=label,
            axes=axes,
        )

    def _stream_pose_toward(self, target_pos, hold_ori, duration=1.0, label="target", axes='all'):
        """Small incremental pose commands (teleop-style); avoids singular diagonal jumps."""
        rate_hz = max(1.0, float(self.pose_cmd_rate_hz))
        dt = 1.0 / rate_hz
        step_m = max(0.001, float(self.pose_step_m))
        if axes == 'z':
            step_m = min(step_m, 0.008)
        cmd_pos = list(self._current_command_pos())
        if axes == 'xy':
            cmd_pos[2] = float(target_pos[2])
        elif axes == 'z':
            cmd_pos[0] = float(target_pos[0])
            cmd_pos[1] = float(target_pos[1])
        deadline = time.monotonic() + max(0.5, float(duration))
        last_log = 0.0
        while time.monotonic() < deadline:
            ee = self._ee_xyz()
            if ee is not None and self._position_reached(ee, target_pos, axes=axes):
                if self._confirm_ee_at_target(target_pos, axes=axes, timeout=0.4):
                    xy_err, z_err, _ = position_error(ee, target_pos)
                    self.get_logger().info(
                        f'Pose reached {label}: err_xy={xy_err:.3f} err_z={z_err:.3f}'
                    )
                    hold = list(target_pos)
                    if axes == 'xy':
                        hold[2] = ee[2]
                    elif axes == 'z':
                        hold[0], hold[1] = ee[0], ee[1]
                    self._sync_cmd_setpoint(hold, hold_ori)
                    self._move_arm(hold, hold_ori)
                    return True
            err = [float(target_pos[i]) - float(cmd_pos[i]) for i in range(3)]
            if axes == 'xy':
                err[2] = 0.0
            elif axes == 'z':
                err[0] = err[1] = 0.0
            dist = math.sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2])
            if dist < step_m * 0.5:
                cmd_pos = list(target_pos)
                if axes == 'xy':
                    cmd_pos[2] = float(self._current_command_pos()[2])
                elif axes == 'z':
                    cmd_pos[0] = float(target_pos[0])
                    cmd_pos[1] = float(target_pos[1])
                self._sync_cmd_setpoint(cmd_pos, hold_ori)
                self._move_arm(cmd_pos, hold_ori)
                break
            scale = min(1.0, step_m / dist) if dist > 1e-9 else 1.0
            cmd_pos = [cmd_pos[i] + err[i] * scale for i in range(3)]
            self._sync_cmd_setpoint(cmd_pos, hold_ori)
            self._move_arm(cmd_pos, hold_ori)
            if self._gripper_hold_value is not None:
                self.pub_grip.publish(Float64(data=self._gripper_hold_value))
            now = time.monotonic()
            if ee is not None and now - last_log > 2.0:
                self.get_logger().info(
                    f'Pose {label} ({axes}): ee={[round(v, 3) for v in ee]} '
                    f'cmd={[round(v, 3) for v in cmd_pos]} '
                    f'target={[round(v, 3) for v in target_pos]}'
                )
                last_log = now
            time.sleep(dt)
        return self._wait_for_ee_near(
            target_pos,
            label=label,
            hold_ori=hold_ori,
            timeout=max(4.0, float(duration) * 0.5),
            axes=axes,
        )

    def _record_motion_segment_error(self, segment_max: float):
        self._trial_max_ee_tracking_error = max(
            float(self._trial_max_ee_tracking_error), float(segment_max)
        )

    def _stream_twist_toward(self, target_pos, duration=1.0, label="target", descend=False):
        rate_hz = max(1.0, float(self.pose_cmd_rate_hz))
        dt = 1.0 / rate_hz
        max_linear = self.twist_descend_linear_mps if descend else self.twist_max_linear_mps
        time_limit = max(float(duration), float(self.ee_arrival_timeout_s))
        deadline = time.monotonic() + max(0.5, time_limit)
        last_log = 0.0
        segment_max = 0.0
        cmd_pos = self._current_command_pos()
        reached = False
        self._publish_twist(0.0, 0.0, 0.0)
        while time.monotonic() < deadline:
            ee = self._ee_xyz()
            segment_max = update_max_tracking_error(ee, cmd_pos, segment_max)
            if ee is not None:
                xy_err, z_err, _ = position_error(ee, target_pos)
                if xy_err <= self.ee_xy_tolerance and z_err <= self.ee_z_tolerance:
                    self.get_logger().info(
                        f'Twist reached {label}: err_xy={xy_err:.3f} err_z={z_err:.3f} '
                        f'segment_track={segment_max:.4f}m'
                    )
                    reached = True
                    break
                vx, vy, vz, step_reached = compute_twist_linear(
                    ee, target_pos, max_linear, z_scale=1.0
                )
                if step_reached:
                    reached = True
                    break
                self._publish_twist(vx, vy, vz)
                cmd_pos = [
                    cmd_pos[0] + vx * dt,
                    cmd_pos[1] + vy * dt,
                    cmd_pos[2] + vz * dt,
                ]
            else:
                self._publish_twist(0.0, 0.0, 0.0)
            now = time.monotonic()
            if now - last_log > 2.0:
                if ee is not None:
                    self.get_logger().info(
                        f'Twist {label}: ee={[round(v, 3) for v in ee]} '
                        f'target={[round(v, 3) for v in target_pos]} '
                        f'segment_track={segment_max:.4f}m'
                    )
                else:
                    self.get_logger().warn(f'Twist {label}: waiting for /ee_pose')
                last_log = now
            time.sleep(dt)
        self._record_motion_segment_error(segment_max)
        self._sync_cmd_setpoint(target_pos, self._last_cmd_ori)
        self._publish_twist(0.0, 0.0, 0.0)
        if not reached:
            ee = self._ee_xyz()
            if ee is not None:
                xy_err, z_err, _ = position_error(ee, target_pos)
                self.get_logger().warn(
                    f'Twist {label} timed out: err_xy={xy_err:.3f} err_z={z_err:.3f} '
                    f'segment_track={segment_max:.4f}m'
                )
            else:
                self.get_logger().warn(f'Twist {label} timed out without /ee_pose')
        return reached

    def _publish_twist(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'panda_link0'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        self.pub_twist.publish(msg)
        if self._gripper_hold_value is not None:
            self.pub_grip.publish(Float64(data=self._gripper_hold_value))

    def _publish_twist_hold(self, duration=1.0):
        duration = max(0.0, float(duration))
        if duration <= 0.0:
            return
        dt = 1.0 / max(1.0, float(self.pose_cmd_rate_hz))
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._publish_twist(0.0, 0.0, 0.0)
            time.sleep(dt)

    def _publish_motion_hold(self, pos, ori, duration=1.0):
        if self.motion_mode == 'twist':
            self._sync_cmd_setpoint(pos, ori)
            self._publish_twist_hold(duration)
        else:
            self._publish_pose_hold(pos, ori, duration=duration, rate_hz=self.pose_cmd_rate_hz)

    def _move_arm(self, pos, ori=None):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "panda_link0"
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        if ori is not None:
            msg.pose.orientation.w = float(ori[0])
            msg.pose.orientation.x = float(ori[1])
            msg.pose.orientation.y = float(ori[2])
            msg.pose.orientation.z = float(ori[3])
        elif self.initial_pose:
            msg.pose.orientation = self.initial_pose.orientation
        else:
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.w = 0.0
        self._cmd_pos = [float(pos[0]), float(pos[1]), float(pos[2])]
        if ori is not None:
            self._last_cmd_ori = [
                float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3])
            ]
        self.pub_pose.publish(msg)
        if self._gripper_hold_value is not None:
            self.pub_grip.publish(Float64(data=self._gripper_hold_value))

    def _move_gripper_smooth(self, start, end, duration=1.0):
        duration = max(0.01, float(duration))
        steps = max(1, int(duration * 50))
        dt = duration / steps
        for i in range(steps):
            alpha = (i + 1) / float(steps)
            cmd = float(start) * (1.0 - alpha) + float(end) * alpha
            self.pub_grip.publish(Float64(data=cmd))
            time.sleep(dt)

    def _set_node_parameter(self, node_name, param_name, param_value, timeout=5.0):
        client_key = f"{node_name.replace('/', '_')}_set_param"
        if not hasattr(self, client_key):
            setattr(self, client_key, self.create_client(SetParameters, f'{node_name}/set_parameters'))
            
        cli = getattr(self, client_key)
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f'{node_name}/set_parameters service not available.')
            return False
            
        req = SetParameters.Request()
        param = Parameter()
        param.name = param_name
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = param_value
        req.parameters = [param]
        
        future = cli.call_async(req)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                try:
                    result = future.result()
                    if result and result.results and result.results[0].successful:
                        self.get_logger().info(f'Set {node_name} param {param_name}={param_value} successful.')
                        return True
                    else:
                        self.get_logger().warn(f'Set {node_name} param {param_name} failed.')
                        return False
                except Exception as exc:
                    self.get_logger().warn(f'Set parameter call failed: {exc}')
                    return False
            time.sleep(0.05)
        self.get_logger().warn(f'Set {node_name} param {param_name} timed out.')
        return False

def main(args=None):
    rclpy.init(args=args)
    node = BatchGenerator()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass

if __name__ == '__main__':
    main()
