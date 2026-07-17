from lerobot_recorder.system_telemetry import match_processes


class FakeProcess:
    def __init__(self, command):
        self.command = command

    def cmdline(self):
        return self.command


def test_process_matching_uses_full_python_cmdline_and_ros_remap():
    processes = [
        FakeProcess([
            "/usr/bin/python3",
            "/opt/ros/lib/camera_bridge/camera_bridge_node",
            "--ros-args",
            "-r",
            "__node:=camera_bridge",
        ]),
        FakeProcess([
            "/usr/bin/python3",
            "/opt/ros/lib/lerobot_recorder/lerobot_recorder_node",
            "--ros-args",
            "-r",
            "__node:=lerobot_recorder",
        ]),
    ]
    matched = match_processes(processes)
    assert matched["scene_camera"] == [processes[0]]
    assert matched["recorder"] == [processes[1]]
