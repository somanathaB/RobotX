#!/usr/bin/env python3

import os
import time
from typing import Optional


def main() -> int:
    try:
        import cv2  # type: ignore
    except Exception as e:
        print(f"OpenCV import failed: {e}")
        print("Install deps: pip install -r requirements.txt")
        return 2

    try:
        from robotx.config.settings import SETTINGS
    except Exception:
        class _S:
            camera_index = 0
            camera_width = 640
            camera_height = 480
            camera_fps = 20
            detection_backend = "opencv"
            detection_min_conf = 0.35
            yolo_model_path = "yolov8n.pt"
        SETTINGS = _S()  # type: ignore

    cam = None

    detector = None
    detector_enabled = False
    try:
        from robotx.perception.object_detector import ObjectDetector  # type: ignore

        detector = ObjectDetector(
            backend=str(getattr(SETTINGS, "detection_backend", "opencv")),
            min_conf=float(getattr(SETTINGS, "detection_min_conf", 0.35)),
            yolo_model_path=str(getattr(SETTINGS, "yolo_model_path", "yolov8n.pt")),
        )
        detector_enabled = True
    except Exception as e:
        print(f"Detector not available (continuing without detection): {e}")

    try:
        from robotx.hardware.camera import CameraStream  # type: ignore
    except Exception as e:
        print(f"ERROR: could not import RobotX camera module: {e}")
        return 2

    cam = CameraStream(
        index=int(getattr(SETTINGS, "camera_index", 0)),
        width=int(getattr(SETTINGS, "camera_width", 640)),
        height=int(getattr(SETTINGS, "camera_height", 480)),
        fps=int(getattr(SETTINGS, "camera_fps", 20)),
    )

    try:
        cam.start()
    except Exception as e:
        print(f"ERROR: Could not start Picamera2 camera: {e}")
        print("Troubleshooting:")
        print("- Test camera detection: rpicam-hello --list-cameras")
        print("- Ensure Picamera2 is installed: sudo apt install -y python3-picamera2")
        print("- If using a venv: create it with --system-site-packages")
        try:
            cam.stop()
        except Exception:
            pass
        return 1

    force_headless = str(os.environ.get("ROBOTX_HEADLESS", "")).strip().lower() in {"1", "true", "yes"}
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    headless = force_headless or not has_display
    if headless:
        print("Headless mode detected (no GUI display).")
        print("- Skipping cv2.imshow() (prevents Qt 'could not connect to display' abort)")
        print("- Will periodically write ./frame.jpg so you can verify capture")

    if headless:
        print("Camera opened. Press Ctrl+C to quit.")
    else:
        print("Camera opened. Press 'q' to quit.")

    last_det_t = 0.0
    det_interval_s = 0.5
    last_print_t = 0.0

    frame_count = 0
    last_save_t = 0.0
    first_frame_deadline_t = time.monotonic() + 5.0

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                if time.monotonic() >= first_frame_deadline_t:
                    age = None
                    try:
                        age = cam.last_frame_age_s()
                    except Exception:
                        pass
                    print("ERROR: No frames received from camera.")
                    if age is not None:
                        print(f"Last-good-frame age: {age:.2f}s")
                    else:
                        print("No frames have been captured yet.")
                    return 1

                # Warm-up period: Picamera2 can take a moment to deliver the first frame.
                time.sleep(0.1)
                continue

            frame_count += 1
            now = time.monotonic()
            if detector_enabled and detector is not None and (now - last_det_t) >= det_interval_s:
                try:
                    dets = detector.detect_objects(frame)
                    if dets and (now - last_print_t) >= 0.5:
                        print("Detections:", dets[:5])
                        last_print_t = now
                except Exception as e:
                    print(f"WARN: detection failed: {e}")
                last_det_t = now

            if headless:
                # Write a frame every ~2 seconds for verification.
                if (now - last_save_t) >= 2.0:
                    try:
                        cv2.imwrite("frame.jpg", frame)
                        print(f"Frame OK: {frame.shape} (wrote frame.jpg)")
                    except Exception as e:
                        print(f"WARN: could not write frame.jpg: {e}")
                    last_save_t = now
                time.sleep(0.01)
            else:
                try:
                    cv2.imshow("RobotX Camera", frame)
                except cv2.error as e:
                    print("ERROR: cv2.imshow failed (likely headless/no DISPLAY).")
                    print(f"Details: {e}")
                    print("Tip: run with an attached display or X forwarding.")
                    return 3

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        if cam is not None:
            try:
                cam.stop()
            except Exception:
                pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    print("Exited cleanly.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
