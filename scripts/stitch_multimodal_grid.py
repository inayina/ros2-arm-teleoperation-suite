#!/usr/bin/env python3
"""Stitch the 4 cameras (Scene, Wrist, Tactile Left, Tactile Right) into a 2x2 grid."""

from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path("/home/ina/dev/ros2-arm-teleoperation-suite")
M6_DIR = ROOT / "media/m6"
OUTPUT_PATH = M6_DIR / "multimodal_sensor_sync_grid.png"


def _font(size: int) -> ImageFont.ImageFont:
    paths = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
    ]
    for path in paths:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def main():
    images_config = [
        ("camera_rgb_view.png", "Scene Camera RGB View"),
        ("wrist_camera_view.png", "Wrist Camera RGB View"),
        ("tactile_left_view.png", "Fingertip Tactile Left View"),
        ("tactile_right_view.png", "Fingertip Tactile Right View"),
    ]

    loaded_images = []
    font = _font(16)

    for filename, label in images_config:
        img_path = M6_DIR / filename
        if not img_path.exists():
            print(f"Error: Required image {img_path} does not exist.")
            return 1
        
        img = Image.open(img_path).convert("RGB")
        # Resize to standard size for consistency (e.g. 320 x 240)
        img = img.resize((320, 240), Image.Resampling.LANCZOS)
        
        # Draw a clean border and label overlay
        draw = ImageDraw.Draw(img)
        # 1px border
        draw.rectangle([0, 0, 319, 239], outline="#cbd5e1", width=2)
        # Background bar for label
        draw.rectangle([5, 5, 250, 28], fill="#1e293b")
        draw.text((10, 8), label, fill="#f8fafc", font=font)
        
        loaded_images.append(img)

    # Create a blank 2x2 grid canvas (640 x 480) with background padding
    grid_img = Image.new("RGB", (640, 480), color="#f1f5f9")
    
    # Paste images
    grid_img.paste(loaded_images[0], (0, 0))      # Top Left
    grid_img.paste(loaded_images[1], (320, 0))    # Top Right
    grid_img.paste(loaded_images[2], (0, 240))    # Bottom Left
    grid_img.paste(loaded_images[3], (320, 240))  # Bottom Right

    grid_img.save(OUTPUT_PATH)
    print(f"Saved: {OUTPUT_PATH}")
    return 0


if __name__ == "__main__":
    exit(main())
