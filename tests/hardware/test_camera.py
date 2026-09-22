#!/usr/bin/env python3

"""MANUAL hardware check: real Raspberry Pi Camera Module 3 + perception.

Not an automated test -- it opens the real camera. Run it by hand:

    venv/bin/python tests/hardware/test_camera.py

Exercises the production perception path (`PerceptionPipeline`), so what you
see here is what the agent sees. In a headless session it writes ./frame.jpg
every couple of seconds instead of opening a window.
"""

import os
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def main() -> int:
    try:
        import cv2  # type: ignore
    except Exception as e:
        print(f"ERROR: OpenCV import failed: {e}")
        print("Install deps: venv/bin/python -m pip install -r requirements.txt")
        return 2

    try:
        from robotx.config.logging_setup import setup_logging
        from robotx.config.settings import SETTINGS
        from robotx.hardware.camera import CameraConfig, CameraError, CameraStream
        from robotx.perception.object_detector import draw_detections
        from robotx.perception.pipeline import PerceptionPipeline
    except Exception as e:
        print(f"ERROR: could not import RobotX modules: {e}")
        return 2

    setup_logging("INFO")

    cam = CameraStream(CameraConfig.from_settings(SETTINGS))
    try:
        cam.start()
    except CameraError as e:
        print(f"ERROR: could not start the camera: {e}")
        print("Troubleshooting:")
        print("- List cameras:        rpicam-hello --list-cameras")
        print("- Install Picamera2:   sudo apt install -y python3-picamera2")
        print("- venv:                python3 -m venv --system-site-packages venv")
        return 1

    pipeline = PerceptionPipeline.build(cam, SETTINGS)
    if not pipeline.enabled:
        print("WARN: perception could not be initialized; showing raw frames only.")

    headless = (
        str(os.environ.get("ROBOTX_HEADLESS", "")).strip().lower() in {"1", "true", "yes"}
        or not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    )
    if headless:
        print("Headless mode: writing ./frame.jpg every ~2s instead of opening a window.")
        print("Press Ctrl+C to quit.")
    else:
        print("Press 'q' in the preview window to quit.")

    last_detect_t = 0.0
    last_save_t = 0.0
    first_frame_deadline = time.monotonic() + 5.0
    exit_code = 0

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                if time.monotonic() >= first_frame_deadline:
                    print("ERROR: no frames received from the camera.")
                    print(f"Camera status: {cam.describe()}")
                    exit_code = 1
                    break
                time.sleep(0.1)
                continue

            now = time.monotonic()
            detections = []
            if pipeline.enabled and (now - last_detect_t) >= 0.5:
                result = pipeline.step_once()
                last_detect_t = now
                print(
                    f"status={result.status.value} "
                    f"detections={len(result.detections)} "
                    f"{result.processing_ms:.0f}ms "
                    f"backend={result.backend}"
                )
                for d in result.detections:
                    print(f"  - {d.label} conf={d.confidence:.2f} area={d.area_px}px bbox={d.bbox}")
                detections = [d.to_dict() for d in result.detections]

            if headless:
                if (now - last_save_t) >= 2.0:
                    annotated = frame.copy()
                    draw_detections(annotated, detections)
                    if cv2.imwrite("frame.jpg", annotated):
                        print(f"Frame OK: {frame.shape} (wrote frame.jpg)")
                    else:
                        print("WARN: could not write frame.jpg")
                    last_save_t = now
                time.sleep(0.01)
            else:
                annotated = frame.copy()
                draw_detections(annotated, detections)
                try:
                    cv2.imshow("RobotX Camera", annotated)
                except cv2.error as e:
                    print(f"ERROR: cv2.imshow failed (no display?): {e}")
                    exit_code = 3
                    break
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cam.stop()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass

    print("Exited cleanly." if exit_code == 0 else "Exited with errors.")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
