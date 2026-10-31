import argparse
import json
import sys
import time
from pathlib import Path

try:
    import cv2
except ImportError:
    print(
        "OpenCV is required. Install it with: "
        "python -m pip install -r .github\\skills\\preamp-webcam-display-test\\requirements.txt",
        file=sys.stderr,
    )
    raise SystemExit(2)

# Display areas as fractions of the frame (x, y, width, height). Defaults match a webcam placed
# in front of the preamp so the whole front panel is visible; override them if the camera moves.
DEFAULT_LEFT_ROI = "0.3125,0.398,0.2083,0.0926"
DEFAULT_RIGHT_ROI = "0.5208,0.398,0.1875,0.0926"

# Sub-areas of the right display as fractions of its ROI (x0, y0, x1, y1)
RIGHT_SUBREGIONS = {
    "main": (0.0, 0.0, 0.84, 1.0),  # volume digits / mute icon, without the WiFi and temperature column
    "center": (0.5, 0.0, 0.8, 1.0),  # where the volume digits / mute icon are drawn
    "wifi": (0.86, 0.08, 1.0, 0.36),
    "temp": (0.86, 0.46, 1.0, 0.84),
}

# Lit OLED pixels are bright cyan/white; reflections and the blue knob lighting have a low green channel
LIT_THRESHOLD = 140
OCR_SCALE = 5
OCR_TARGET_HEIGHT = 120
SLASH_CLOSE_SIZE = 27


CALIBRATION_FILE = Path(__file__).with_name("camera_calibration.json")


def load_calibration():
    # Written by calibrate_camera.py; missing or unreadable file means the built-in defaults are used
    try:
        return json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def parse_roi(text):
    try:
        values = [float(part) for part in text.split(",")]
    except ValueError:
        raise argparse.ArgumentTypeError("ROI must be four comma-separated numbers: x,y,width,height")
    if len(values) != 4 or any(v < 0 or v > 1 for v in values) or values[0] + values[2] > 1 or values[1] + values[3] > 1:
        raise argparse.ArgumentTypeError("ROI must be x,y,width,height as fractions of the frame (0-1)")
    return tuple(values)


def region(frame, roi, sub=(0.0, 0.0, 1.0, 1.0)):
    height, width = frame.shape[:2]
    x, y, w, h = roi
    x0 = int(round((x + w * sub[0]) * width))
    x1 = int(round((x + w * sub[2]) * width))
    y0 = int(round((y + h * sub[1]) * height))
    y1 = int(round((y + h * sub[3]) * height))
    return frame[y0:y1, x0:x1], (x0, y0, x1, y1)


def count_lit(pixels):
    if pixels.size == 0:
        return 0
    return int(((pixels[..., 1] > LIT_THRESHOLD) & (pixels[..., 0] > LIT_THRESHOLD)).sum())


def prepare_for_ocr(pixels, fill_slashed_zero=False):
    # Light text on a dark background reads poorly in Tesseract: upscale, stretch contrast and invert
    gray = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, None, fx=OCR_SCALE, fy=OCR_SCALE, interpolation=cv2.INTER_CUBIC)
    if fill_slashed_zero:
        # The display font draws zero with a slash that Tesseract's digit mode cannot read; closing fills the thin gap
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (SLASH_CLOSE_SIZE, SLASH_CLOSE_SIZE))
        gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
    gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
    gray = cv2.bitwise_not(gray)
    # Crop tightly around the text: Tesseract misreads a lone glyph surrounded by a large blank area
    ys, xs = (gray < 90).nonzero()
    if len(xs) > 50:
        margin = 40
        y0, y1 = max(ys.min() - margin, 0), min(ys.max() + margin, gray.shape[0])
        x0, x1 = max(xs.min() - margin, 0), min(xs.max() + margin, gray.shape[1])
        gray = gray[y0:y1, x0:x1]
        # Tesseract works best with text around 60-100 px tall; the 5x upscale makes it far larger
        gray = cv2.resize(gray, None, fx=OCR_TARGET_HEIGHT / gray.shape[0], fy=OCR_TARGET_HEIGHT / gray.shape[0], interpolation=cv2.INTER_AREA)
    return cv2.copyMakeBorder(gray, 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=255)


def capture_frame(camera_index, warmup_seconds, timeout_seconds, width, height):
    camera = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    if not camera.isOpened():
        camera.release()
        camera = cv2.VideoCapture(camera_index)
    if not camera.isOpened():
        print(f"Could not open webcam at index {camera_index}.", file=sys.stderr)
        raise SystemExit(1)
    # MJPG is needed for high resolutions over USB 2.0
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    frame = None
    try:
        start = time.monotonic()
        # Keep reading during the warm-up so exposure settles and buffered old frames are discarded
        while time.monotonic() - start < timeout_seconds:
            ok, candidate = camera.read()
            if ok and candidate is not None:
                frame = candidate
                if time.monotonic() - start >= warmup_seconds:
                    break
            else:
                time.sleep(0.1)
    finally:
        camera.release()

    if frame is None:
        print("The webcam opened but did not provide an image frame.", file=sys.stderr)
        raise SystemExit(1)
    return frame


def main():
    parser = argparse.ArgumentParser(description="Capture a still image from a webcam and analyse the preamp displays.")
    cal = load_calibration()
    parser.add_argument("--camera-index", type=int, default=cal.get("camera_index", 0),
                        help="Zero-based webcam index (default: from calibration, else 0)")
    parser.add_argument("--output", required=True, help="Path for the captured image")
    parser.add_argument("--width", type=int, default=cal.get("width", 1920), help="Requested capture width (default: 1920)")
    parser.add_argument("--height", type=int, default=cal.get("height", 1080), help="Requested capture height (default: 1080)")
    parser.add_argument("--warmup-seconds", type=float, default=1.5, help="Camera exposure warm-up time")
    parser.add_argument("--timeout-seconds", type=float, default=10, help="Maximum time to obtain a frame")
    parser.add_argument("--left-roi",     type=parse_roi, default=parse_roi(cal.get("left_roi", DEFAULT_LEFT_ROI)), help="Left display as x,y,w,h frame fractions")
    parser.add_argument("--right-roi", type=parse_roi, default=parse_roi(cal.get("right_roi", DEFAULT_RIGHT_ROI)), help="Right display as x,y,w,h frame fractions")
    parser.add_argument("--analyze", action="store_true",
                        help="Also write <output>-left.png / -right.png (OCR-ready crops), <output>-rois.png (annotated) and <output>.json (lit-pixel counts)")
    args = parser.parse_args()

    if args.camera_index < 0:
        parser.error("--camera-index must be zero or greater")

    frame = capture_frame(args.camera_index, args.warmup_seconds, args.timeout_seconds, args.width, args.height)
    print(f"Captured {frame.shape[1]}x{frame.shape[0]}")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), frame):
        print(f"Could not write captured image to {output}.", file=sys.stderr)
        raise SystemExit(1)

    if args.analyze:
        left_pixels, left_box = region(frame, args.left_roi)
        right_pixels, right_box = region(frame, args.right_roi)
        right_main, _ = region(frame, args.right_roi, RIGHT_SUBREGIONS["main"])

        metrics = {"left": count_lit(left_pixels), "right": count_lit(right_pixels)}
        annotated = frame.copy()
        cv2.rectangle(annotated, left_box[:2], left_box[2:], (0, 255, 0), 1)
        cv2.rectangle(annotated, right_box[:2], right_box[2:], (0, 255, 0), 1)
        for name, sub in RIGHT_SUBREGIONS.items():
            pixels, box = region(frame, args.right_roi, sub)
            metrics["right_" + name] = count_lit(pixels)
            cv2.rectangle(annotated, box[:2], box[2:], (0, 0, 255), 1)

        cv2.imwrite(str(output.with_name(output.stem + "-left.png")), prepare_for_ocr(left_pixels))
        cv2.imwrite(str(output.with_name(output.stem + "-right.png")), prepare_for_ocr(right_main, fill_slashed_zero=True))
        cv2.imwrite(str(output.with_name(output.stem + "-rois.png")), annotated)
        output.with_suffix(".json").write_text(json.dumps(metrics), encoding="utf-8")
        print(f"Lit pixels: {metrics}")

    print(f"Captured {frame.shape[1]}x{frame.shape[0]} image: {output}")


if __name__ == "__main__":
    main()
