"""Person detection (OpenCV HOG) and single-frame persistence.

OpenCV is not needed: the HOG detector is a stand-in with the same
`detectMultiScale` interface, and the motion/edge path is stubbed. What is
under test is that people are asked for, kept, and acted on at once, and that
an obstacle seen in one frame only is not.
"""

import time
import unittest

from robotx.control.decision import DecisionMaker
from robotx.navigation.navigator import NavigationState, NavigationStatus
from robotx.perception.object_detector import DetectorConfig, ObjectDetector, filter_detections
from robotx.perception.pipeline import PerceptionPipeline, PipelineConfig
from robotx.perception.types import PerceptionStatus
from tests.unit.test_perception import FakeCamera, FakeDetector, FakeFrame


class FakeHog:
    """cv2.HOGDescriptor.detectMultiScale: (rects, weights as an (n, 1) array)."""

    def __init__(self, rects=(), weights=()):
        self.rects, self.weights, self.calls = list(rects), [[w] for w in weights], 0

    def detectMultiScale(self, frame, **kwargs):
        self.calls += 1
        return self.rects, self.weights


def opencv_detector(hog, edges=()):
    """An `opencv` ObjectDetector without OpenCV: HOG injected, motion/edges stubbed."""

    d = ObjectDetector.__new__(ObjectDetector)
    d.cfg, d.backend, d.min_conf, d.last_debug = DetectorConfig(), "opencv", 0.35, {}
    d._hog, d._yolo, d._bg, d._bg_frames = hog, None, None, 0
    d._detect_opencv = lambda frame: list(edges)
    return d


class TestHogIsInvoked(unittest.TestCase):
    def test_the_opencv_backend_asks_hog_for_people(self):
        hog = FakeHog(rects=[(100, 40, 64, 128)], weights=[1.4])
        out = opencv_detector(hog).detect_objects(FakeFrame())
        self.assertEqual(hog.calls, 1)
        self.assertEqual([d["label"] for d in out], ["person"])
        self.assertEqual(out[0]["bbox"], (100, 40, 164, 168))
        self.assertEqual(out[0]["confidence"], 1.0)          # weight clipped to [0, 1]

    def test_a_low_weight_person_is_still_a_person(self):
        out = opencv_detector(FakeHog(rects=[(100, 40, 64, 128)], weights=[0.05])).detect_objects(FakeFrame())
        self.assertEqual([d["label"] for d in out], ["person"])

    def test_no_people_no_person(self):
        out = opencv_detector(FakeHog()).detect_objects(FakeFrame())
        self.assertEqual(out, [])


class TestPeopleSurviveTheOutputFilter(unittest.TestCase):
    def test_a_near_person_filling_the_frame_height_is_kept(self):
        rows = [{"label": "person", "confidence": 0.9, "bbox": (100, 0, 260, 480)}]
        self.assertEqual([d["label"] for d in filter_detections(rows, frame_w=640, frame_h=480)], ["person"])
        # The same box as an obstacle is still rejected as background.
        rows[0]["label"] = "obstacle"
        self.assertEqual(filter_detections(rows, frame_w=640, frame_h=480), [])

    def test_a_person_cut_off_by_the_frame_edge_is_kept_clipped(self):
        rows = [{"label": "person", "confidence": 0.9, "bbox": (560, 100, 700, 400)}]
        kept = filter_detections(rows, frame_w=640, frame_h=480)
        self.assertEqual(kept[0]["bbox"], (560, 100, 640, 400))

    def test_a_person_is_never_ranked_out_by_bigger_obstacles(self):
        rows = [{"label": "obstacle", "confidence": 0.9, "bbox": (10 + 150 * i, 200, 150 + 150 * i, 400)}
                for i in range(4)]
        rows.append({"label": "person", "confidence": 0.1, "bbox": (300, 100, 340, 180)})
        kept = filter_detections(rows, frame_w=640, frame_h=480, max_keep=3)
        self.assertIn("person", [d["label"] for d in kept])
        self.assertEqual(len(kept), 3)

    def test_a_speck_is_not_a_person(self):
        rows = [{"label": "person", "confidence": 0.9, "bbox": (10, 10, 20, 20)}]
        self.assertEqual(filter_detections(rows, frame_w=640, frame_h=480), [])


OBSTACLE = {"label": "obstacle", "confidence": 0.6, "bbox": (120, 120, 200, 200)}
PERSON = {"label": "person", "confidence": 0.8, "bbox": (40, 20, 104, 148)}


class TestPersistence(unittest.TestCase):
    def pipeline(self, detector, camera=None, **cfg):
        return PerceptionPipeline(camera or FakeCamera(frame=FakeFrame()), detector,
                                  PipelineConfig(detection_hz=10.0, **cfg))

    def labels(self, result):
        return [d.label for d in result.detections]

    def test_an_obstacle_in_one_frame_only_is_not_reported(self):
        det = FakeDetector(rows=[OBSTACLE])
        p = self.pipeline(det)
        self.assertEqual(self.labels(p.step_once()), [])
        det.rows = []
        self.assertEqual(self.labels(p.step_once()), [])

    def test_an_obstacle_in_two_consecutive_frames_is_reported(self):
        p = self.pipeline(FakeDetector(rows=[OBSTACLE]))
        p.step_once()
        self.assertEqual(self.labels(p.step_once()), ["obstacle"])

    def test_boxes_that_do_not_overlap_do_not_confirm_each_other(self):
        det = FakeDetector(rows=[OBSTACLE])
        p = self.pipeline(det)
        p.step_once()
        det.rows = [dict(OBSTACLE, bbox=(260, 120, 320, 200))]
        self.assertEqual(self.labels(p.step_once()), [])

    def test_a_person_is_reported_on_the_first_frame(self):
        p = self.pipeline(FakeDetector(rows=[PERSON]))
        self.assertEqual(self.labels(p.step_once()), ["person"])

    def test_a_gap_breaks_the_run(self):
        camera = FakeCamera(frame=FakeFrame())
        p = self.pipeline(FakeDetector(rows=[OBSTACLE]), camera)
        p.step_once()
        camera.frame = None
        self.assertIs(p.step_once().status, PerceptionStatus.NO_FRAME)
        camera.frame = FakeFrame()
        self.assertEqual(self.labels(p.step_once()), [])      # first frame again

    def test_one_cycle_setting_acts_at_once(self):
        p = self.pipeline(FakeDetector(rows=[OBSTACLE]), confirm_cycles=1)
        self.assertEqual(self.labels(p.step_once()), ["obstacle"])

    def test_stale_results_stay_unusable(self):
        p = self.pipeline(FakeDetector(rows=[PERSON]), stale_after_s=0.5)
        p.step_once()
        p._result = type(p._result)(**{**p._result.__dict__, "timestamp": time.time() - 5})
        self.assertIs(p.latest().status, PerceptionStatus.STALE)
        self.assertFalse(p.latest().is_usable)


class TestAPersonStopsTheRover(unittest.TestCase):
    def test_a_first_frame_person_is_a_stop(self):
        p = PerceptionPipeline(FakeCamera(frame=FakeFrame()), opencv_detector(
            FakeHog(rects=[(40, 20, 64, 128)], weights=[0.3])), PipelineConfig(detection_hz=10.0))
        result = p.step_once()
        nav = NavigationState(status=NavigationStatus.NAVIGATING, heading_error_deg=0.0,
                              distance_to_target_m=50.0, waypoints_total=1)
        intent = DecisionMaker().decide(mission_active=True, navigation=nav, perception=result)
        self.assertTrue(intent.is_stop)
        self.assertEqual(intent.reason, "person detected")


if __name__ == "__main__":
    unittest.main()
