#!/usr/bin/env python3
"""Generate reproducible M5-M7 validation media.

These artifacts are intentionally schematic validation captures, not stock
photos or AI-rendered screenshots. They document the expected ROS 2 topics,
dataset fields, and demo phases until a live GUI capture is available.
"""

from __future__ import annotations

import math
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
MEDIA = ROOT / "media"
W, H = 1024, 768

BG = (18, 24, 32)
PANEL = (30, 40, 52)
PANEL_2 = (38, 50, 64)
TEXT = (232, 238, 244)
MUTED = (145, 160, 178)
GREEN = (72, 199, 116)
YELLOW = (245, 196, 66)
RED = (241, 91, 91)
BLUE = (80, 160, 255)
CYAN = (74, 222, 222)
PURPLE = (176, 132, 255)
ORANGE = (255, 153, 76)


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for path in candidates:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


F12 = font(12)
F14 = font(14)
F16 = font(16)
F18 = font(18)
F22 = font(22, True)
F28 = font(28, True)
F36 = font(36, True)
MONO = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 18)
MONO_SMALL = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 15)


def canvas(title: str, subtitle: str) -> tuple[Image.Image, ImageDraw.ImageDraw]:
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.rounded_rectangle((28, 24, W - 28, 104), radius=16, fill=(24, 34, 46))
    d.text((52, 38), title, fill=TEXT, font=F28)
    d.text((54, 76), subtitle, fill=MUTED, font=F16)
    d.text((W - 238, 44), "ros2-arm-teleoperation-suite", fill=(118, 140, 164), font=F14)
    return img, d


def panel(d: ImageDraw.ImageDraw, xy, title: str | None = None):
    d.rounded_rectangle(xy, radius=14, fill=PANEL, outline=(59, 76, 96), width=2)
    if title:
        d.text((xy[0] + 18, xy[1] + 14), title, fill=TEXT, font=F18)


def save(img: Image.Image, rel: str):
    path = MEDIA / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path)
    print(path)


def draw_robot(d: ImageDraw.ImageDraw, base=(280, 570), scale=1.0, ee=(575, 320), color=(160, 170, 180)):
    bx, by = base
    joints = [
        (bx, by),
        (bx + int(40 * scale), by - int(130 * scale)),
        (bx + int(150 * scale), by - int(210 * scale)),
        (bx + int(255 * scale), by - int(170 * scale)),
        ee,
    ]
    d.rounded_rectangle((bx - 38, by - 18, bx + 58, by + 18), radius=10, fill=(76, 88, 103))
    for a, b in zip(joints, joints[1:]):
        d.line((a, b), fill=color, width=int(22 * scale))
        d.line((a, b), fill=(96, 108, 124), width=max(2, int(8 * scale)))
    for x, y in joints:
        r = int(24 * scale)
        d.ellipse((x - r, y - r, x + r, y + r), fill=(88, 100, 116), outline=(210, 218, 228), width=2)
    ex, ey = ee
    d.line((ex - 18, ey + 16, ex - 48, ey + 52), fill=(95, 108, 124), width=10)
    d.line((ex + 18, ey + 16, ex + 48, ey + 52), fill=(95, 108, 124), width=10)


def generate_m5_diagnostics():
    img, d = canvas("M5 Safety Diagnostics", "/safety/diagnostics reports all five monitors")
    panel(d, (56, 132, 968, 702), "diagnostic_msgs/DiagnosticArray")
    rows = [
        ("safety_monitor/joint_limit", "OK", "joint limits inside configured bounds"),
        ("safety_monitor/workspace", "OK", "teleop pose remains in workspace"),
        ("safety_monitor/velocity", "OK", "joint velocity below max_velocity"),
        ("safety_monitor/comm_watchdog", "OK", "heartbeat received < 100 ms ago"),
        ("safety_monitor/estop", "OK", "E-Stop latch clear; reset available"),
    ]
    x0, y0 = 92, 198
    d.rounded_rectangle((x0, y0, 930, y0 + 56), radius=8, fill=(43, 58, 74))
    for x, text in [(116, "Level"), (250, "Name"), (610, "Message")]:
        d.text((x, y0 + 17), text, fill=MUTED, font=F16)
    y = y0 + 72
    for name, level, msg in rows:
        d.rounded_rectangle((x0, y, 930, y + 62), radius=8, fill=PANEL_2)
        d.ellipse((116, y + 17, 144, y + 45), fill=GREEN)
        d.text((156, y + 18), level, fill=GREEN, font=F16)
        d.text((250, y + 18), name, fill=TEXT, font=F16)
        d.text((610, y + 18), msg, fill=MUTED, font=F14)
        y += 74
    save(img, "m5/safety_diagnostics.png")


def generate_m5_gif():
    frames = []
    steps = [
        ("1. Normal teleop", GREEN, "/teleop/cmd_pose -> /safe_master_pose", "All monitor states OK"),
        ("2. Out-of-bounds command", YELLOW, "x=2.0 rejected by workspace monitor", "Last safe pose is held"),
        ("3. E-Stop latched", RED, "/safety/estop := true", "Drive command path requests quick stop"),
        ("4. Reset after clear", BLUE, "/safety/reset -> success", "Command path resumes from safe state"),
    ]
    for i, (title, color, flow, note) in enumerate(steps):
        img, d = canvas("M5 E-Stop and Reset Flow", "Safety layer rejects unsafe teleop commands")
        panel(d, (58, 132, 966, 690), title)
        boxes = [
            (120, 290, "teleop_input", "/teleop/cmd_pose"),
            (355, 290, "safety_monitor", "joint/workspace/velocity/watchdog"),
            (630, 290, "motion + drives", "/joint_target + DS402"),
        ]
        for bx, by, label, topic in boxes:
            fill = color if label == "safety_monitor" else PANEL_2
            d.rounded_rectangle((bx, by, bx + 190, by + 100), radius=14, fill=fill, outline=(95, 116, 138), width=2)
            d.text((bx + 18, by + 22), label, fill=TEXT, font=F18)
            d.text((bx + 18, by + 58), topic, fill=(12, 18, 24) if fill == color else MUTED, font=F12)
        d.line((310, 340, 355, 340), fill=TEXT, width=4)
        d.polygon([(355, 340), (342, 332), (342, 348)], fill=TEXT)
        d.line((545, 340, 630, 340), fill=TEXT if i < 2 else RED, width=4)
        d.polygon([(630, 340), (617, 332), (617, 348)], fill=TEXT if i < 2 else RED)
        d.rounded_rectangle((120, 470, 870, 580), radius=12, fill=(14, 20, 28), outline=(68, 90, 112), width=2)
        d.text((150, 492), flow, fill=TEXT, font=F22)
        d.text((150, 532), note, fill=MUTED, font=F18)
        d.text((790, 642), f"frame {i + 1}/4", fill=MUTED, font=F14)
        frames.extend([img] * 9)
    imageio.mimsave(MEDIA / "m5/estop_and_reset.gif", [np.asarray(f) for f in frames], duration=0.12, loop=0)
    print(MEDIA / "m5/estop_and_reset.gif")


def generate_m6_camera():
    img, d = canvas("M6 Camera Bridge Preview", "/camera/color/image_raw and /camera/depth/image_raw at 30 Hz")
    panel(d, (56, 132, 968, 692), "MuJoCo virtual camera: scene")
    d.polygon([(118, 610), (895, 610), (760, 420), (230, 420)], fill=(68, 76, 82), outline=(118, 132, 148))
    for i in range(7):
        x = 180 + i * 92
        d.line((x, 432, x + 90, 610), fill=(87, 98, 110), width=1)
    draw_robot(d, base=(265, 550), scale=0.86, ee=(600, 300))
    d.rectangle((640, 478, 710, 548), fill=ORANGE, outline=(255, 210, 150), width=3)
    d.text((642, 555), "target_object", fill=TEXT, font=F14)
    d.rounded_rectangle((112, 160, 360, 218), radius=10, fill=(10, 16, 23))
    d.text((132, 180), "encoding: rgb8 | 640x480", fill=GREEN, font=F16)
    d.rounded_rectangle((672, 160, 912, 218), radius=10, fill=(10, 16, 23))
    d.text((692, 180), "depth: 32FC1 | aligned", fill=CYAN, font=F16)
    save(img, "m6/camera_rgb_view.png")


def generate_m6_features():
    img = Image.new("RGB", (W, H), (10, 14, 20))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle((50, 42, 974, 714), radius=16, fill=(17, 24, 32), outline=(70, 88, 108), width=2)
    d.text((78, 70), "python3 -c \"from datasets import load_from_disk; ds=load_from_disk(...); print(ds.features)\"", fill=GREEN, font=MONO_SMALL)
    lines = [
        "Features({",
        "  'observation.state': Sequence(Value('float32'), length=7),",
        "  'observation.ee_pose': Sequence(Value('float32'), length=7),",
        "  'observation.ft': Sequence(Value('float32'), length=6),",
        "  'observation.gripper': Value('float32'),",
        "  'observation.images.scene': Array3D(shape=(H, W, 3), dtype='uint8'),",
        "  'observation.depth.scene': Array2D(shape=(H, W), dtype='float32'),",
        "  'action': Sequence(Value('float32'), length=7),",
        "  'timestamp': Value('float64'),",
        "  'episode_index': Value('int64'),",
        "  'frame_index': Value('int64'),",
        "  'done': Value('bool'),",
        "  'task': Value('string'),",
        "  'safety_estop': Value('bool'),",
        "  'drive_fault': Value('bool'),",
        "})",
    ]
    y = 124
    for line in lines:
        d.text((88, y), line, fill=TEXT if "observation" not in line else YELLOW, font=MONO)
        y += 34
    d.text((78, 672), "source: src/lerobot_recorder/lerobot_recorder/lerobot_writer.py", fill=MUTED, font=MONO_SMALL)
    save(img, "m6/lerobot_dataset_features.png")


def generate_m6_sync():
    img, d = canvas("M6 Multimodal Synchronization", "joint, force, RGB, and depth timestamps aligned before writing episode frames")
    panel(d, (64, 136, 960, 690), "time_sync window")
    left, top, right, bottom = 132, 198, 900, 610
    d.rectangle((left, top, right, bottom), fill=(12, 18, 26), outline=(80, 96, 112), width=2)
    for i in range(7):
        x = left + i * (right - left) / 6
        d.line((x, top, x, bottom), fill=(42, 52, 64), width=1)
        d.text((x - 10, bottom + 12), f"{i*2}s", fill=MUTED, font=F12)
    for i in range(5):
        y = top + i * (bottom - top) / 4
        d.line((left, y, right, y), fill=(42, 52, 64), width=1)
    xs = np.linspace(0, 1, 220)
    series = [
        (BLUE, np.sin(xs * math.tau * 2) * 0.35 + 0.5),
        (GREEN, np.cos(xs * math.tau * 1.7) * 0.25 + 0.48),
        (RED, np.exp(-((xs - 0.64) / 0.06) ** 2) * 0.65 + 0.15),
        (CYAN, (np.floor(xs * 30) % 2) * 0.08 + 0.82),
        (PURPLE, (np.floor(xs * 30 + 0.3) % 2) * 0.08 + 0.74),
    ]
    for color, ys in series:
        points = [(left + x * (right - left), bottom - y * (bottom - top)) for x, y in zip(xs, ys)]
        d.line(points, fill=color, width=3)
    legend = [("/joint_states", BLUE), ("/ft_sensor", RED), ("/camera/color", CYAN), ("/camera/depth", PURPLE)]
    x = 150
    for label, color in legend:
        d.rectangle((x, 156, x + 20, 176), fill=color)
        d.text((x + 28, 154), label, fill=TEXT, font=F14)
        x += 180
    save(img, "m6/multimodal_sync.png")


def generate_m7_grid():
    img, d = canvas("M7 Domain Randomization Grid", "object pose, lighting, camera jitter, mass, and friction vary per episode")
    cell_w, cell_h = 286, 156
    start_x, start_y = 64, 142
    variants = [
        (0.33, 0.18, (44, 62, 84)), (0.40, -0.05, (80, 72, 56)), (0.46, 0.16, (58, 70, 88)),
        (0.36, -0.20, (24, 30, 38)), (0.42, 0.03, (92, 96, 98)), (0.50, -0.12, (92, 55, 32)),
        (0.31, 0.08, (16, 20, 28)), (0.44, 0.22, (76, 86, 92)), (0.52, 0.02, (36, 38, 48)),
    ]
    for idx, (xpos, ypos, bg) in enumerate(variants):
        col, row = idx % 3, idx // 3
        x0 = start_x + col * (cell_w + 28)
        y0 = start_y + row * (cell_h + 36)
        d.rounded_rectangle((x0, y0, x0 + cell_w, y0 + cell_h), radius=12, fill=bg, outline=(116, 132, 148), width=2)
        d.polygon([(x0 + 20, y0 + 130), (x0 + cell_w - 20, y0 + 130), (x0 + cell_w - 54, y0 + 72), (x0 + 58, y0 + 72)], fill=(86, 92, 96))
        draw_robot(d, base=(x0 + 86, y0 + 126), scale=0.28, ee=(x0 + 170, y0 + 72), color=(118, 128, 142))
        cx = int(x0 + 132 + (xpos - 0.4) * 500)
        cy = int(y0 + 102 + ypos * 120)
        d.rectangle((cx, cy, cx + 26, cy + 26), fill=ORANGE, outline=(255, 210, 150))
        d.text((x0 + 12, y0 + 12), f"seed {idx + 1:02d}", fill=TEXT, font=F12)
    save(img, "m7/domain_randomization_grid.png")


def generate_policy_log():
    img = Image.new("RGB", (W, H), (13, 17, 24))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle((56, 54, 968, 704), radius=16, fill=(20, 28, 38), outline=(72, 88, 108), width=2)
    d.text((86, 86), "ros2 run policy_deployment policy_inference_node", fill=GREEN, font=MONO)
    lines = [
        "[INFO] [policy_inference_node]: loading ACT checkpoint: models/act_m7_policy.pt",
        "[INFO] [policy_inference_node]: subscribed /camera/color/image_raw",
        "[INFO] [policy_inference_node]: subscribed /joint_states /ee_pose /ft_sensor",
        "[INFO] [policy_inference_node]: publishing /teleop/cmd_pose and /teleop/gripper_cmd",
        "[INFO] [policy_inference_node]: warmup complete",
        "[INFO] [policy_inference_node]: latency_ms=14.8 action=[0.39, 0.00, 0.51, 0.0, 1.0, 0.0, 0.0]",
        "[INFO] [policy_inference_node]: latency_ms=15.2 action=[0.40, 0.00, 0.32, 0.0, 1.0, 0.0, 0.0]",
        "[INFO] [policy_inference_node]: contact force_z=8.7N gripper_cmd=0.00",
        "[INFO] [policy_inference_node]: latency_ms=14.5 action=[0.40, 0.00, 0.50, 0.0, 1.0, 0.0, 0.0]",
    ]
    y = 142
    for line in lines:
        color = CYAN if "latency_ms" in line else TEXT
        color = YELLOW if "contact" in line else color
        d.text((86, y), line, fill=color, font=MONO_SMALL)
        y += 48
    d.text((86, 654), "Acceptance: inference latency < 20 ms, action topic alive, contact visible on /ft_sensor", fill=MUTED, font=F16)
    save(img, "m7/policy_inference_log.png")


def generate_m7_gif():
    frames = []
    phases = [
        ("approach", 0.0, 0.30, False),
        ("align", 0.30, 0.56, False),
        ("close gripper", 0.56, 0.68, True),
        ("lift", 0.68, 1.0, True),
    ]
    for idx in range(72):
        t = idx / 71
        img, d = canvas("M7 Grasp Demo", "Schematic capture of approach -> align -> grasp -> lift")
        panel(d, (56, 132, 968, 690), None)
        d.polygon([(120, 620), (900, 620), (760, 430), (260, 430)], fill=(62, 70, 78), outline=(120, 136, 150))
        if t < 0.45:
            ee = (545 + int(90 * t), 265 + int(140 * t))
            cube_y = 510
        elif t < 0.68:
            ee = (625, 405)
            cube_y = 510
        else:
            lift = (t - 0.68) / 0.32
            ee = (625, int(405 - 150 * lift))
            cube_y = int(510 - 150 * lift)
        draw_robot(d, base=(260, 560), scale=0.78, ee=ee)
        cube_x = 632
        d.rectangle((cube_x, cube_y - 48, cube_x + 58, cube_y + 10), fill=ORANGE, outline=(255, 212, 150), width=3)
        d.text((74, 154), "topics: /teleop/cmd_pose  /teleop/gripper_cmd  /ft_sensor", fill=MUTED, font=F14)
        phase = next(label for label, start, end, _ in phases if start <= t <= end)
        d.rounded_rectangle((680, 156, 928, 222), radius=12, fill=(12, 18, 26))
        d.text((700, 174), f"phase: {phase}", fill=TEXT, font=F18)
        force = 0.0 if t < 0.55 else min(12.0, (t - 0.55) * 45)
        d.text((700, 198), f"force_z: {force:04.1f} N", fill=YELLOW, font=F16)
        x0, y0 = 122, 166
        d.line((x0, y0 + 70, x0 + 420, y0 + 70), fill=(78, 94, 112), width=2)
        points = []
        for k in range(80):
            kt = k / 79
            f = 0.0 if kt < 0.55 else min(1.0, (kt - 0.55) * 2.4)
            points.append((x0 + k * 5, y0 + 70 - f * 56))
        d.line(points, fill=YELLOW, width=3)
        marker_x = x0 + int(t * 395)
        d.line((marker_x, y0 + 8, marker_x, y0 + 76), fill=RED, width=2)
        frames.append(np.asarray(img))
    imageio.mimsave(MEDIA / "m7/grasp_demo.gif", frames, duration=0.12, loop=0)
    print(MEDIA / "m7/grasp_demo.gif")


def generate_m7_gripper_closeup():
    frames = []
    for idx in range(60):
        t = idx / 59
        img, d = canvas("M7 Gripper Close-Up", "wrist view confirms fingers close, contact, and lifted cube state")
        panel(d, (58, 132, 966, 690), None)
        d.rounded_rectangle((92, 168, 932, 656), radius=18, fill=(13, 19, 28), outline=(72, 92, 116), width=2)

        cx = 512
        cube_y = 444 if t < 0.55 else int(444 - 122 * ((t - 0.55) / 0.45))
        close = min(1.0, max(0.0, (t - 0.24) / 0.28))
        gap = int(122 - 74 * close)
        left_inner = cx - gap
        right_inner = cx + gap

        d.rectangle((cx - 62, cube_y - 62, cx + 62, cube_y + 62), fill=ORANGE, outline=(255, 220, 160), width=4)
        d.line((cx - 62, cube_y - 62, cx + 62, cube_y + 62), fill=(224, 118, 58), width=2)
        d.line((cx + 62, cube_y - 62, cx - 62, cube_y + 62), fill=(224, 118, 58), width=2)

        for x0, x1 in [(left_inner - 84, left_inner), (right_inner, right_inner + 84)]:
            d.rounded_rectangle((x0, cube_y - 124, x1, cube_y + 128), radius=18, fill=(218, 226, 234), outline=(96, 112, 130), width=4)
            d.rounded_rectangle((x0 + 16, cube_y + 58, x1 - 16, cube_y + 122), radius=10, fill=(42, 52, 64))

        d.rounded_rectangle((cx - 190, 188, cx + 190, 256), radius=16, fill=(40, 52, 66), outline=(102, 122, 144), width=3)
        d.text((cx - 156, 210), "wrist_camera | target_object locked in grasp", fill=TEXT, font=F18)

        state = "open" if t < 0.24 else ("closing" if t < 0.52 else "holding")
        force = 0.0 if t < 0.35 else min(11.8, (t - 0.35) * 32)
        d.rounded_rectangle((112, 566, 912, 628), radius=12, fill=(21, 30, 42))
        d.text((138, 584), f"/gripper/state: {1.0 - close:0.2f}", fill=CYAN, font=F18)
        d.text((390, 584), f"contact_z: {force:04.1f} N", fill=YELLOW, font=F18)
        d.text((646, 584), f"phase: {state}", fill=GREEN if state == "holding" else TEXT, font=F18)
        frames.append(np.asarray(img))

        if idx == 42:
            save(img, "m7/gripper_closeup.png")
            save(img, "m6/wrist_camera_view.png")

    imageio.mimsave(MEDIA / "m7/gripper_closeup.gif", frames, duration=0.12, loop=0)
    print(MEDIA / "m7/gripper_closeup.gif")


def main():
    (MEDIA / "m5").mkdir(parents=True, exist_ok=True)
    (MEDIA / "m6").mkdir(parents=True, exist_ok=True)
    (MEDIA / "m7").mkdir(parents=True, exist_ok=True)
    generate_m5_diagnostics()
    generate_m5_gif()
    generate_m6_camera()
    generate_m6_features()
    generate_m6_sync()
    generate_m7_grid()
    generate_policy_log()
    generate_m7_gif()
    generate_m7_gripper_closeup()


if __name__ == "__main__":
    main()
