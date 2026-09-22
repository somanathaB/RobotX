"""Perception: result model, detector filtering, and the production pipeline."""

import time
import unittest

from robotx.perception.object_detector import filter_detections
from robotx.perception.pipeline import PerceptionPipeline, PipelineConfig
from robotx.perception.types import (
    Detection,
    FrameMetadata,
    PerceptionResult,
    PerceptionStatus,
    detections_from_dicts,
)


def make_detection(label="obstacle", conf=0.6, bbox=(100, 100, 200, 200)):
    x1, y1, x2, y2 = bbox
    return Detection(
        label=label, confidence=conf, bbox=bbox, area_px=(x2 - x1) * (y2 - y1)
    )


def ok_result(detections=(), width=640, height=480, delta=0):
    return PerceptionResult(
        timestamp=time.time(),
        status=PerceptionStatus.OK,
        detections=tuple(detections),
        frame=FrameMetadata(width=width, height=height, age_s=0.05),
        backend="test",
        largest_area_delta_px=delta,
    )


class TestDetectionModel(unittest.TestCase):
    def test_center_and_area(self):
        det = make_detection(bbox=(100, 50, 300, 150))
        self.assertEqual(det.center, (200, 100))
        self.assertEqual(det.cx, 200)
        self.assertEqual(det.area_px, 200 * 100)

    def test_no_distance_field_is_reported(self):
        # The camera is monocular: no distance may appear anywhere in output.
        payload = make_detection().to_dict()
        for forbidden in ("distance", "distance_m", "depth", "range_m"):
            self.assertNotIn(forbidden, payload)

    def test_from_dict_rejects_malformed(self):
        self.assertIsNone(Detection.from_dict({"bbox": (1, 2, 3)}))
        self.assertIsNone(Detection.from_dict({"bbox": "nope"}))
        self.assertIsNone(Detection.from_dict({}))
        # Zero-area / inverted boxes are not objects.
        self.assertIsNone(Detection.from_dict({"bbox": (10, 10, 10, 20)}))
        self.assertIsNone(Detection.from_dict({"bbox": (30, 10, 10, 20)}))

    def test_from_dict_computes_missing_area(self):
        det = Detection.from_dict({"label": "person", "confidence": 0.9, "bbox": (0, 0, 10, 20)})
        self.assertIsNotNone(det)
        self.assertEqual(det.area_px, 200)

    def test_detections_from_dicts_drops_bad_rows(self):
        rows = [
            {"label": "a", "bbox": (0, 0, 10, 10)},
            {"label": "b", "bbox": "bad"},
            "not a dict",
        ]
        self.assertEqual(len(detections_from_dicts(rows)), 1)


class TestPerceptionResult(unittest.TestCase):
    def test_only_ok_is_usable(self):
        self.assertTrue(ok_result().is_usable)
        for status in (
            PerceptionStatus.NO_FRAME,
            PerceptionStatus.DETECTOR_ERROR,
            PerceptionStatus.DISABLED,
            PerceptionStatus.STALE,
        ):
            self.assertFalse(PerceptionResult.unavailable(status).is_usable, status)

    def test_empty_ok_result_differs_from_unavailable(self):
        # "I saw nothing" and "I could not see" must not be the same value.
        self.assertTrue(ok_result().is_usable)
        self.assertFalse(PerceptionResult.unavailable(PerceptionStatus.NO_FRAME).is_usable)

    def test_largest_picks_biggest_area(self):
        small = make_detection(bbox=(0, 0, 10, 10))
        big = make_detection(bbox=(0, 0, 100, 100))
        self.assertIs(ok_result([small, big]).largest, big)

    def test_largest_is_none_without_detections(self):
        self.assertIsNone(ok_result().largest)

    def test_has_label_respects_confidence(self):
        result = ok_result([make_detection(label="person", conf=0.4)])
        self.assertTrue(result.has_label("person"))
        self.assertFalse(result.has_label("person", min_confidence=0.5))
        self.assertFalse(result.has_label("obstacle"))

    def test_summary_is_serializable_and_honest(self):
        summary = PerceptionResult.unavailable(
            PerceptionStatus.NO_FRAME, error="camera returned no frame"
        ).summary()
        self.assertEqual(summary["status"], "NO_FRAME")
        self.assertEqual(summary["count"], 0)
        self.assertIsNone(summary["largest_area_px"])
        self.assertEqual(summary["error"], "camera returned no frame")


class TestDetectorFiltering(unittest.TestCase):
    def test_keeps_plausible_box(self):
        kept = filter_detections(
            [{"label": "obstacle", "confidence": 0.6, "bbox": (100, 100, 200, 200)}],
            frame_w=640,
            frame_h=480,
        )
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0]["area"], 10000)

    def test_rejects_full_frame_span(self):
        kept = filter_detections(
            [{"label": "obstacle", "confidence": 0.9, "bbox": (0, 0, 640, 480)}],
            frame_w=640,
            frame_h=480,
        )
        self.assertEqual(kept, [])

    def test_rejects_tiny_and_huge_areas(self):
        tiny = filter_detections(
            [{"label": "o", "confidence": 0.9, "bbox": (10, 10, 20, 20)}],
            frame_w=640,
            frame_h=480,
        )
        self.assertEqual(tiny, [])

    def test_rejects_extreme_aspect_ratio(self):
        sliver = filter_detections(
            [{"label": "o", "confidence": 0.9, "bbox": (10, 10, 310, 45)}],
            frame_w=640,
            frame_h=480,
        )
        self.assertEqual(sliver, [])

    def test_rejects_box_extending_past_frame(self):
        kept = filter_detections(
            [{"label": "o", "confidence": 0.9, "bbox": (600, 100, 700, 200)}],
            frame_w=640,
            frame_h=480,
        )
        self.assertEqual(kept, [])

    def test_ranks_by_area_times_confidence_and_truncates(self):
        rows = [
            {"label": "o", "confidence": 0.2, "bbox": (10, 10, 160, 160)},
            {"label": "o", "confidence": 0.9, "bbox": (200, 200, 350, 350)},
            {"label": "o", "confidence": 0.5, "bbox": (400, 100, 530, 230)},
            {"label": "o", "confidence": 0.4, "bbox": (50, 300, 160, 410)},
        ]
        kept = filter_detections(rows, frame_w=640, frame_h=480, max_keep=2)
        self.assertEqual(len(kept), 2)
        self.assertAlmostEqual(kept[0]["confidence"], 0.9)

    def test_ignores_non_dict_rows(self):
        self.assertEqual(filter_detections(["x", None, 3], frame_w=640, frame_h=480), [])


class FakeCamera:
    """Stand-in for CameraStream: no hardware, scripted frames."""

    def __init__(self, frame=None, age=0.05):
        self.frame = frame
        self.age = age

    def get_frame(self):
        return self.frame

    def last_frame_age_s(self):
        return None if self.frame is None else self.age


class FakeFrame:
    """Minimal numpy-array stand-in: only `.shape` is used at 1:1 scale."""

    def __init__(self, width=320, height=240):
        self.shape = (height, width, 3)


class FakeDetector:
    backend = "fake"

    def __init__(self, rows=None, raises=None):
        self.rows = rows or []
        self.raises = raises
        self.calls = 0

    def detect_objects(self, frame):
        self.calls += 1
        if self.raises is not None:
            raise self.raises
        return self.rows


class TestPerceptionPipeline(unittest.TestCase):
    def make_pipeline(self, camera, detector, **overrides):
        cfg = PipelineConfig(
            detection_hz=10.0,
            # Match FakeFrame so no cv2 resize is attempted.
            inference_width=320,
            inference_height=240,
            **overrides,
        )
        return PerceptionPipeline(camera, detector, cfg)

    def test_no_frame_yields_no_frame_status(self):
        pipeline = self.make_pipeline(FakeCamera(frame=None), FakeDetector())
        result = pipeline.step_once()
        self.assertIs(result.status, PerceptionStatus.NO_FRAME)
        self.assertFalse(result.is_usable)

    def test_stale_frame_is_not_treated_as_current(self):
        camera = FakeCamera(frame=FakeFrame(), age=30.0)
        pipeline = self.make_pipeline(camera, FakeDetector(), frame_stale_after_s=2.0)
        result = pipeline.step_once()
        self.assertIs(result.status, PerceptionStatus.NO_FRAME)
        self.assertIn("old", result.error)

    def test_detector_exception_is_reported_not_swallowed(self):
        detector = FakeDetector(raises=RuntimeError("inference exploded"))
        pipeline = self.make_pipeline(FakeCamera(frame=FakeFrame()), detector)
        result = pipeline.step_once()
        self.assertIs(result.status, PerceptionStatus.DETECTOR_ERROR)
        self.assertIn("inference exploded", result.error)
        self.assertEqual(result.detections, ())

    def test_successful_cycle_produces_detections_and_metadata(self):
        detector = FakeDetector(
            rows=[{"label": "obstacle", "confidence": 0.7, "bbox": (10, 10, 60, 60), "area": 2500}]
        )
        pipeline = self.make_pipeline(FakeCamera(frame=FakeFrame()), detector)
        result = pipeline.step_once()

        self.assertIs(result.status, PerceptionStatus.OK)
        self.assertEqual(len(result.detections), 1)
        self.assertEqual(result.detections[0].label, "obstacle")
        self.assertEqual(result.frame.width, 320)
        self.assertEqual(result.frame.height, 240)
        self.assertEqual(result.backend, "fake")

    def test_area_delta_tracks_growth_between_cycles(self):
        detector = FakeDetector(
            rows=[{"label": "obstacle", "confidence": 0.7, "bbox": (0, 0, 50, 50)}]
        )
        pipeline = self.make_pipeline(FakeCamera(frame=FakeFrame()), detector)

        first = pipeline.step_once()
        self.assertEqual(first.largest_area_delta_px, 2500)

        detector.rows = [{"label": "obstacle", "confidence": 0.7, "bbox": (0, 0, 100, 100)}]
        second = pipeline.step_once()
        self.assertEqual(second.largest_area_delta_px, 10000 - 2500)

    def test_latest_downgrades_aged_result_to_stale(self):
        detector = FakeDetector(rows=[])
        pipeline = self.make_pipeline(
            FakeCamera(frame=FakeFrame()), detector, stale_after_s=0.0
        )
        pipeline.step_once()
        time.sleep(0.01)
        self.assertIs(pipeline.latest().status, PerceptionStatus.STALE)

    def test_disabled_pipeline_reports_disabled(self):
        pipeline = self.make_pipeline(None, None)
        self.assertFalse(pipeline.enabled)
        self.assertIs(pipeline.step_once().status, PerceptionStatus.DISABLED)


if __name__ == "__main__":
    unittest.main()
