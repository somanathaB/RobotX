#!/usr/bin/env python3

import os
import time
import subprocess
from collections import deque
from typing import Any, Dict, List, Optional, Tuple


os.environ.setdefault("OPENCV_VIDEOIO_PRIORITY_LIST", "V4L2")


# --- Test/debug mode ---
# When enabled, prints extra debugging information (in addition to the mandatory per-frame logs).
TEST_MODE = True


def _safe_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return int(default)


def _short_detections(detections: List[Dict[str, Any]], limit: int = 6) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for d in (detections or [])[: int(max(0, limit))]:
        if not isinstance(d, dict):
            continue
        out.append(
            {
                "label": str(d.get("label", "object")),
                "area": _safe_int(d.get("area", 0), 0),
                "bbox": tuple(d.get("bbox")) if isinstance(d.get("bbox"), (list, tuple)) else d.get("bbox"),
            }
        )
    return out


def _draw_roi(frame, roi) -> None:
    if frame is None or not roi or len(roi) != 4:
        return
    try:
        import cv2  # type: ignore
    except Exception:
        return

    try:
        x1, y1, x2, y2 = [int(v) for v in roi]
    except Exception:
        return
    cv2.rectangle(frame, (x1, y1), (x2 - 1, y2 - 1), (0, 255, 255), 2)
    cv2.putText(frame, "ROI", (x1 + 10, y1 + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


def _draw_zone_lines(frame) -> Tuple[int, int]:
    """Draw 3 vertical zones and return (left_split, right_split)."""

    if frame is None:
        return (0, 0)
    try:
        import cv2  # type: ignore
    except Exception:
        return (0, 0)

    h, w = frame.shape[:2]
    x1 = int(w // 3)
    x2 = int((2 * w) // 3)
    cv2.line(frame, (x1, 0), (x1, h - 1), (0, 255, 255), 2)
    cv2.line(frame, (x2, 0), (x2, h - 1), (0, 255, 255), 2)
    return (x1, x2)


def _draw_primary(frame, primary) -> None:
    if frame is None or primary is None:
        return
    try:
        import cv2  # type: ignore
    except Exception:
        return

    try:
        bb = getattr(primary, "bbox", None)
        if not bb or len(bb) != 4:
            return
        x1, y1, x2, y2 = [int(v) for v in bb]
    except Exception:
        return

    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)


def _overlay_text(frame, *, action: str, fps: float, area: int, num_objects: int) -> None:
    if frame is None:
        return
    try:
        import cv2  # type: ignore
    except Exception:
        return

    action = str(action)
    cv2.putText(frame, f"ACTION: {action}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(frame, f"AREA: {int(area)}", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"FPS: {float(fps):.1f}", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.putText(frame, f"Objects: {num_objects}", (10, 120),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)


def _draw_trajectory(frame, primary_track) -> None:
    if frame is None or not isinstance(primary_track, dict):
        return
    try:
        import cv2  # type: ignore
    except Exception:
        return

    try:
        cx = int(primary_track.get("cx"))
        cy = int(primary_track.get("cy"))
        vx = float(primary_track.get("vx", 0.0))
        vy = float(primary_track.get("vy", 0.0))
    except Exception:
        return

    # Draw motion arrow (scaled for visibility).
    scale = 6.0
    x2 = int(round(float(cx) + scale * float(vx)))
    y2 = int(round(float(cy) + scale * float(vy)))
    cv2.arrowedLine(frame, (cx, cy), (x2, y2), (255, 255, 0), 2, tipLength=0.3)


def _draw_detections_with_primary(frame, detections, primary) -> None:
    """Draw bboxes in GREEN (primary handled separately)."""

    if frame is None:
        return
    try:
        import cv2  # type: ignore
    except Exception:
        return

    for d in (detections or []):
        if not isinstance(d, dict):
            continue
        bbox = d.get("bbox")
        if not bbox or len(bbox) != 4:
            continue

        try:
            x1, y1, x2, y2 = [int(v) for v in bbox]
        except Exception:
            continue

        label = str(d.get("label", "object"))
        try:
            area = int(d.get("area", 0) or 0)
        except Exception:
            area = 0

        color = (0, 255, 0)
        thickness = 2

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(
            frame,
            f"{label} {area}",
            (x1, max(15, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )


def main() -> int:
    try:
        import cv2  # type: ignore
    except Exception as e:
        print(f"ERROR: OpenCV import failed: {e}")
        return 2

    try:
        from robotx.perception.experimental.vision_controller import VisionController, VisionControllerConfig
        from robotx.perception.experimental.decision_engine import DecisionEngine
        from robotx.perception.temporal_filter import ActionSmoother
        from robotx.perception.object_tracker import PrimaryObjectTracker
    except Exception as e:
        print(f"ERROR: imports failed: {e}")
        return 2

    # Headless-friendly: write annotated frames to disk.
    save_debug = str(os.environ.get("ROBOTX_SAVE_DEBUG", "1")).strip().lower() in {"1", "true", "yes"}
    frames_dir = str(os.environ.get("ROBOTX_FRAMES_DIR", "frames")).strip() or "frames"
    debug_path = os.path.join(frames_dir, "frame.jpg")
    save_every = 1

    # Pi-friendly defaults
    try:
        controller = VisionController(
            cfg=VisionControllerConfig(
                backend="auto",
                yolo_model_path="yolov8n.pt",
                min_conf=0.35,
                detection_hz=5.0,
                inference_width=320,
                inference_height=240,
                stop_area_ratio=0.12,
                slow_area_ratio=0.05,
                track_min_hits=2,
                temporal_window=6,
                temporal_min_presence=3,
                action_smooth_window=5,
            ),
            camera_index=0,
            width=640,
            height=480,
            fps=20,
        )
    except Exception as e:
        print(f"ERROR: Failed to start camera/vision pipeline: {e}")

        print(
            "\nNote: Raspberry Pi Camera Module 3 uses libcamera/Picamera2 (not cv2.VideoCapture).\n"
            "Picamera2 is typically installed via apt and may not be visible inside a plain venv.\n"
            "Recommended venv setup:\n"
            "  python3 -m venv --system-site-packages venv_cam\n"
            "  ./venv_cam/bin/pip install -r requirements.txt\n"
            "  ./venv_cam/bin/python test_vision.py\n"
        )
        try:
            out = subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True, stderr=subprocess.STDOUT)
            print("\nv4l2-ctl --list-devices:\n" + out.strip())
        except Exception:
            pass
        try:
            out = subprocess.check_output(["rpicam-hello", "--list-cameras"], text=True, stderr=subprocess.STDOUT)
            print("\nrpicam-hello --list-cameras:\n" + out.strip())
        except Exception:
            pass
        try:
            out = subprocess.check_output(["lsusb"], text=True, stderr=subprocess.STDOUT)
            print("\nlsusb:\n" + out.strip())
        except Exception:
            pass

        print(
            "\nFix checklist:\n"
            "- USB webcam: unplug/replug, try a powered hub, then confirm it appears in `lsusb` and `v4l2-ctl --list-devices` as a UVC camera.\n"
            "- CSI camera: re-seat the ribbon cable (orientation matters), then run `rpicam-hello --list-cameras`.\n"
            "- If no camera appears in any list, this is a hardware/OS detection issue (not a Python code issue).\n"
        )
        return 1

    # Warmup: if we never get a frame, fail fast with actionable diagnostics.
    warmup_deadline = time.monotonic() + 5.0
    while time.monotonic() < warmup_deadline:
        out = controller.step()
        if out.get("frame") is not None:
            break
        time.sleep(0.05)
    else:
        print("ERROR: Camera opened but no frames were received.")
        try:
            out = subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True, stderr=subprocess.STDOUT)
            print("v4l2-ctl --list-devices:\n" + out.strip())
        except Exception as e:
            print(f"(Could not run v4l2-ctl for diagnostics: {e})")

        print(
            "\nTroubleshooting:\n"
            "- If using a CSI camera: make sure it is connected, enabled, and detected by libcamera.\n"
            "  Try: rpicam-hello --list-cameras\n"
            "- If using a USB webcam: it should usually appear as /dev/video0 and in v4l2-ctl output.\n"
        )
        controller.close()
        return 1

    print("Starting vision pipeline (headless).")
    print("- Quit: Ctrl+C")
    print("- Detector backend is 'auto': uses YOLOv8 if ultralytics/torch are installed, otherwise falls back to OpenCV.")
    if TEST_MODE:
        print("\nTEST MODE ENABLED")
        print("Validation test cases (expected):")
        print("- TEST 1 — EMPTY SCENE: Detections: [] | Action: MOVE_FORWARD")
        print("- TEST 2 — MOVING OBJECT: Motion detected: True | Detections: not empty | Action: SLOW or STOP")
        print("- TEST 3 — STATIC OBJECT: Mode: STATIC | Detections: not empty | Stable action")
        print("- TEST 4 — CLOSE OBJECT: area increases | Action: MOVE_FORWARD → SLOW → STOP")
        print("- TEST 5 — LEFT/RIGHT POSITION: LEFT→TURN_RIGHT RIGHT→TURN_LEFT CENTER→STOP")
        print("Performance requirement: FPS must be > 10 minimum\n")
    if save_debug:
        os.makedirs(frames_dir, exist_ok=True)
        print(f"- Debug frames: writing {debug_path} (every {save_every} frames)")

    frame_count = 0
    fps_hist = deque(maxlen=30)
    last_wall_t = time.time()

    engine = DecisionEngine()
    primary_tracker = PrimaryObjectTracker()
    smoother = ActionSmoother()

    action_hist = deque(maxlen=8)

    try:
        while True:
            out = controller.step()
            frame = out["frame"]
            detections: List[Dict[str, Any]] = list(out.get("detections", []) or [])
            # 🔥 REMOVE NOISE (very important)
            detections = [d for d in detections if d.get("area", 0) > 2000]

            print("Raw areas:", [d.get("area", 0) for d in detections])

            if not detections:
                print("⚠️ NO OBJECT DETECTED")
            stable_labels = out.get("stable_labels", [])
            controller_action = out.get("action", "STOP")
            primary_det = out.get("primary")
            state = out.get("state", "FORWARD")
            position = out.get("position", "UNKNOWN")
            in_collision = bool(out.get("in_collision", False))
            roi = out.get("roi")
            primary_track = out.get("primary_track")
            dbg: Dict[str, Any] = dict(out.get("detector_debug", {}) or {})

            # --- detect -> track -> decide -> smooth -> act ---
            primary = primary_tracker.update(detections)

            # 🚨 CRITICAL FIX — ignore stale detections
            if not detections:
                primary = None

            action = engine.decide(primary)
            action = smoother.update(action)

            frame_count += 1

            # --- FPS (wall clock) ---
            now_wall = time.time()
            dt = float(now_wall - float(last_wall_t))
            last_wall_t = float(now_wall)
            fps_inst = 0.0 if dt <= 1e-6 else (1.0 / dt)
            fps_hist.append(float(fps_inst))
            fps = float(sum(fps_hist)) / float(max(1, len(fps_hist)))

            action_hist.append(str(action))

            if frame is None:
                print("WARN: no frame")
                time.sleep(0.05)
                continue

            # --- Required structured logs (EVERY FRAME) ---
            # Mode/motion info comes from OpenCV backend debug; keep strict MOTION/STATIC output.
            # 🔥 CLEAR MODE BASED ON REAL DETECTION
            motion_detected = bool(dbg.get("motion_detected", False))
            if detections:
                mode = "REAL_OBJECT"
            else:
                mode = "MOTION" if motion_detected else "STATIC"
            if mode == "MOTION":
                contours_n = _safe_int(dbg.get("motion_contours", dbg.get("contours", 0)), 0)
            else:
                contours_n = _safe_int(dbg.get("static_contours", dbg.get("contours", 0)), 0)

            primary_area = 0
            primary_cx = 0
            primary_obj: Optional[Dict[str, Any]] = None
            if primary is not None:
                try:
                    primary_area = _safe_int(getattr(primary, "area", 0), 0)
                    primary_cx = _safe_int(getattr(primary, "cx", 0), 0)
                    primary_obj = {
                        "label": str(getattr(primary, "label", "object")),
                        "area": int(primary_area),
                        "cx": int(primary_cx),
                        "missed": _safe_int(getattr(primary, "missed", 0), 0),
                    }
                except Exception:
                    primary_obj = None

            print("\n" + "=" * 50)
            print(f"FPS: {fps:.1f}")
            print(f"Mode: {mode}")
            print(f"Motion detected: {bool(motion_detected)}")
            print(f"Contours: {int(contours_n)}")
            print(f"Detections: {_short_detections(detections)}")
            print(f"Primary: {primary_obj}")
            if primary_obj is None:
                print("Primary area: 0")
                print("Primary cx: 0")
            else:
                print(f"Primary area: {int(primary_area)}")
                print(f"Primary cx: {int(primary_cx)}")
            print(f"Action: {str(action)}")

            if TEST_MODE:
                backend = str(out.get("backend", "unknown"))
                stable_s = stable_labels if isinstance(stable_labels, list) else []
                extra = f"Extra: backend={backend} state={state} pos={position} collision={bool(in_collision)}"
                if str(controller_action) != str(action):
                    extra += f" controller_action={controller_action}"
                if stable_s:
                    extra += f" stable={stable_s}"
                print(extra)

            # --- Performance + fail-safe warnings ---
            if fps > 0.0 and fps < 10.0:
                print("WARNING: Low FPS (<10)")
            if primary is not None and _safe_int(getattr(primary, "missed", 0), 0) > 2:
                print("WARNING: Unstable detection")
            if detections and primary is None:
                print("WARNING: Unstable detection")
            if len(set(action_hist)) >= 4 and detections:
                print("WARNING: Unstable detection")

            print("----------------------")

            # Save annotated frame only every N frames to limit disk writes.
            if save_debug and save_every > 0 and (frame_count % int(save_every)) == 0:
                try:
                    annotated = frame.copy()

                    # Draw detections (green boxes).
                    _draw_detections_with_primary(annotated, detections, primary)

                    # Draw primary (red box).
                    _draw_primary(annotated, primary)

                    # Draw 3-zone split lines.
                    _draw_zone_lines(annotated)

                    # Draw ROI (lower 70% used for processing).
                    _draw_roi(annotated, roi)

                    # Draw collision zone + blue center dot + zone/action labels.
                    try:
                        from robotx.perception.experimental.vision_controller import draw_avoidance_debug

                        draw_avoidance_debug(annotated, primary=primary_det, action=str(action))
                    except Exception:
                        pass

                    # Draw trajectory arrow from smoothed tracker.
                    _draw_trajectory(annotated, primary_track)

                    # Overlay state + action explicitly (force visibility).
                    try:
                        cv2.putText(
                            annotated,
                            f"STATE: {state}",
                            (20, 115),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (255, 255, 255),
                            2,
                        )
                        cv2.putText(
                            annotated,
                            f"POSITION: {position}  COLLISION: {bool(in_collision)}",
                            (20, 145),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (255, 255, 255),
                            2,
                        )
                        cv2.putText(
                            annotated,
                            f"ACTION: {str(action)}",
                            (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (255, 255, 255),
                            2,
                        )
                    except Exception:
                        pass

                    # Required overlay: Action / Area / FPS
                    _overlay_text(annotated, action=str(action), fps=float(fps), area=int(primary_area), num_objects=len(detections))

                    # Optionally annotate stable labels.
                    if isinstance(stable_labels, list) and stable_labels:
                        try:
                            cv2.putText(
                                annotated,
                                "Stable: " + ",".join(str(x) for x in stable_labels),
                                (20, 85),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (255, 255, 255),
                                2,
                            )
                        except Exception:
                            pass

                    cv2.imwrite(debug_path, annotated)
                except Exception as e:
                    print(f"WARN: debug frame write failed: {e}")

            time.sleep(0.2)  # FPS max

    except KeyboardInterrupt:
        pass
    finally:
        controller.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
