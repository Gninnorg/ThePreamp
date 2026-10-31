"""Interactive webcam calibration: position the camera with a live view and mark the two displays.

Saves camera_calibration.json next to this script; capture_display.py uses it as its defaults.
"""
import argparse
import json
import sys

import cv2

from capture_display import (
    CALIBRATION_FILE,
    DEFAULT_LEFT_ROI,
    DEFAULT_RIGHT_ROI,
    RIGHT_SUBREGIONS,
    capture_frame,
    count_lit,
    load_calibration,
    parse_roi,
    region,
)

WINDOW = "ThePreamp camera calibration"
MAX_VIEW_WIDTH = 1280
REFERENCE_FILE = CALIBRATION_FILE.with_name("camera_calibration_ref.png")
MIN_LIT_PIXELS = 30
BAND_RATIO = 1.5
MAX_SHIFT = 0.01
HELP = [
    "1 = mark LEFT display   2 = mark RIGHT display   (drag with mouse)",
    "arrows = nudge, + / - = resize, h = toggle help",
    "r = reset ROIs   s = save   q / Esc = quit",
]


class State:
    def __init__(self, left, right):
        self.rois = {"left": list(left), "right": list(right)}
        self.active = "left"
        self.drag_start = None
        self.drag_now = None
        self.view_size = (1, 1)
        self.show_help = True


def clamp_roi(roi):
    x, y, w, h = roi
    w = min(max(w, 0.01), 1.0)
    h = min(max(h, 0.01), 1.0)
    return [min(max(x, 0.0), 1.0 - w), min(max(y, 0.0), 1.0 - h), w, h]


def on_mouse(event, x, y, _flags, state):
    width, height = state.view_size
    point = (min(max(x / width, 0.0), 1.0), min(max(y / height, 0.0), 1.0))
    if event == cv2.EVENT_LBUTTONDOWN:
        state.drag_start = state.drag_now = point
    elif event == cv2.EVENT_MOUSEMOVE and state.drag_start:
        state.drag_now = point
    elif event == cv2.EVENT_LBUTTONUP and state.drag_start:
        x0, y0 = state.drag_start
        x1, y1 = point
        w, h = abs(x1 - x0), abs(y1 - y0)
        if w > 0.01 and h > 0.01:
            state.rois[state.active] = clamp_roi([min(x0, x1), min(y0, y1), w, h])
        state.drag_start = state.drag_now = None


def draw_overlay(frame, state):
    view = frame.copy()
    height, width = view.shape[:2]
    lit = {}
    for name, roi in state.rois.items():
        active = name == state.active
        color = (0, 255, 0) if active else (0, 200, 200)
        pixels, box = region(frame, roi)
        lit[name] = count_lit(pixels)
        cv2.rectangle(view, box[:2], box[2:], color, 2 if active else 1)
        cv2.putText(view, f"{name}: {lit[name]} lit px", (box[0], max(box[1] - 6, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    for sub in RIGHT_SUBREGIONS.values():
        _, box = region(frame, state.rois["right"], sub)
        cv2.rectangle(view, box[:2], box[2:], (0, 0, 255), 1)
    if state.drag_start and state.drag_now:
        p0 = (int(state.drag_start[0] * width), int(state.drag_start[1] * height))
        p1 = (int(state.drag_now[0] * width), int(state.drag_now[1] * height))
        cv2.rectangle(view, p0, p1, (255, 255, 0), 1)

    lines = [f"Marking: {state.active.upper()} display"] + (HELP if state.show_help else [])
    for i, text in enumerate(lines):
        cv2.putText(view, text, (10, 22 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(view, text, (10, 22 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return view


def nudge(state, dx, dy, dw, dh):
    x, y, w, h = state.rois[state.active]
    step = 0.002
    state.rois[state.active] = clamp_roi([x + dx * step, y + dy * step, w + dw * step, h + dh * step])


def save(state, args, frame):
    data = {
        "camera_index": args.camera_index,
        "width": frame.shape[1],
        "height": frame.shape[0],
        "left_roi": ",".join(f"{v:.4f}" for v in state.rois["left"]),
        "right_roi": ",".join(f"{v:.4f}" for v in state.rois["right"]),
    }
    CALIBRATION_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")
    cv2.imwrite(str(REFERENCE_FILE), frame)
    print(f"Saved {CALIBRATION_FILE}")


def to_gray(frame):
    return cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (0, 0), 3)


def validate(frame, cal):
    """Return a list of problems; empty means the saved calibration still fits the camera view."""
    problems = []
    if not cal.get("left_roi") or not cal.get("right_roi"):
        return ["no saved calibration"]
    left, right = parse_roi(cal["left_roi"]), parse_roi(cal["right_roi"])

    for name, roi in (("left", left), ("right", right)):
        inside, _ = region(frame, roi)
        lit_inside = count_lit(inside)
        if lit_inside < MIN_LIT_PIXELS:
            problems.append(f"{name} display: no lit pixels in the box (display off, screen saver or camera moved)")
            continue
        # Lit pixels in a band around the box mean the display is cut off or the box is misaligned
        x, y, w, h = roi
        grow_x, grow_y = w * 0.3, h * 0.6
        outer = clamp_roi([x - grow_x, y - grow_y, w + 2 * grow_x, h + 2 * grow_y])
        outer_pixels, _ = region(frame, outer)
        lit_band = count_lit(outer_pixels) - lit_inside
        # The other display may overlap the band, so only a clearly dominant band counts
        if lit_band > lit_inside * BAND_RATIO:
            problems.append(f"{name} display: lit pixels outside the box (box misaligned)")

    if REFERENCE_FILE.exists():
        reference = cv2.imread(str(REFERENCE_FILE))
        if reference is not None:
            if reference.shape != frame.shape:
                problems.append("camera resolution differs from the calibration")
            else:
                # Phase correlation finds how far the static scene moved, independent of brightness changes
                (dx, dy), _ = cv2.phaseCorrelate(to_gray(reference).astype("float32"), to_gray(frame).astype("float32"))
                shift = max(abs(dx) / frame.shape[1], abs(dy) / frame.shape[0])
                if shift > MAX_SHIFT:
                    problems.append(f"camera view has shifted {shift * 100:.1f}% since the calibration")
    return problems


def check(args):
    cal = load_calibration()
    if not cal:
        print("No saved calibration.")
        return 1
    frame = capture_frame(args.camera_index, 1.5, 10, args.width, args.height)
    problems = validate(frame, cal)
    for problem in problems:
        print(f"Calibration problem: {problem}")
    print("Calibration is valid." if not problems else "Calibration is NOT valid.")
    return 1 if problems else 0


def open_camera(index, width, height):
    camera = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not camera.isOpened():
        camera.release()
        camera = cv2.VideoCapture(index)
    if not camera.isOpened():
        print(f"Could not open webcam at index {index}.", file=sys.stderr)
        raise SystemExit(1)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return camera


def main():
    cal = load_calibration()
    parser = argparse.ArgumentParser(description="Interactive webcam calibration for the preamp display tests.")
    parser.add_argument("--camera-index", type=int, default=cal.get("camera_index", 0), help="Zero-based webcam index")
    parser.add_argument("--width", type=int, default=cal.get("width", 1920))
    parser.add_argument("--height", type=int, default=cal.get("height", 1080))
    parser.add_argument("--fresh", action="store_true", help="Ignore the saved calibration and start from the defaults")
    parser.add_argument("--check", action="store_true",
                        help="Non-interactive: verify the saved calibration against a camera frame; exit code 0 = valid, 1 = not valid")
    args = parser.parse_args()
    if args.camera_index < 0:
        parser.error("--camera-index must be zero or greater")
    if args.check:
        raise SystemExit(check(args))

    saved = {} if args.fresh else cal
    state = State(parse_roi(saved.get("left_roi", DEFAULT_LEFT_ROI)),
                  parse_roi(saved.get("right_roi", DEFAULT_RIGHT_ROI)))
    camera = open_camera(args.camera_index, args.width, args.height)
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, on_mouse, state)
    print("Turn the preamp on (not in standby) so both displays are lit, then mark the displays.")
    print("\n".join(HELP))

    frame = None
    try:
        while True:
            ok, candidate = camera.read()
            if ok and candidate is not None:
                frame = candidate
            if frame is None:
                if cv2.waitKey(100) in (27, ord("q")):
                    break
                continue

            view = draw_overlay(frame, state)
            if view.shape[1] > MAX_VIEW_WIDTH:
                scale = MAX_VIEW_WIDTH / view.shape[1]
                view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            state.view_size = (view.shape[1], view.shape[0])
            cv2.imshow(WINDOW, view)

            # waitKeyEx keeps the arrow-key codes on Windows
            key = cv2.waitKeyEx(30)
            if key in (27, ord("q")) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            elif key == ord("1"):
                state.active = "left"
            elif key == ord("2"):
                state.active = "right"
            elif key == ord("h"):
                state.show_help = not state.show_help
            elif key == ord("r"):
                state.rois = {"left": list(parse_roi(DEFAULT_LEFT_ROI)), "right": list(parse_roi(DEFAULT_RIGHT_ROI))}
            elif key == ord("s"):
                save(state, args, frame)
            elif key == 2424832:  # left
                nudge(state, -1, 0, 0, 0)
            elif key == 2555904:  # right
                nudge(state, 1, 0, 0, 0)
            elif key == 2490368:  # up
                nudge(state, 0, -1, 0, 0)
            elif key == 2621440:  # down
                nudge(state, 0, 1, 0, 0)
            elif key in (ord("+"), ord("=")):
                nudge(state, 0, 0, 0.002, 0.002)
            elif key in (ord("-"), ord("_")):
                nudge(state, 0, 0, -0.002, -0.002)
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
