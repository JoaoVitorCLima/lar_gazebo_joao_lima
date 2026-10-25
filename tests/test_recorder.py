"""Queued camera frames must never reopen a finalized video."""
import importlib.util
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import MagicMock, patch


class RecorderTest(unittest.TestCase):
    def setUp(self):
        names = ['cv2', 'rospy', 'cv_bridge', 'sensor_msgs', 'sensor_msgs.msg',
                 'std_msgs', 'std_msgs.msg']
        dependencies = {name: MagicMock() for name in names}
        path = Path(__file__).resolve().parents[1] / 'scripts/record_top_view.py'
        spec = importlib.util.spec_from_file_location('recorder_under_test', path)
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, dependencies):
            spec.loader.exec_module(self.module)
        self.recorder = self.module.Recorder.__new__(self.module.Recorder)
        self.recorder.closed = False
        self.recorder.lock = threading.RLock()
        self.writer = MagicMock()
        self.recorder.writer = self.writer

    def test_queued_frame_after_close_does_not_reopen_file(self):
        with patch.object(self.module.shutil, 'which', return_value=None):
            self.recorder.close()
        self.recorder.frame(MagicMock())
        self.writer.release.assert_called_once()
        self.module.cv2.VideoWriter.assert_not_called()
        self.assertIsNone(self.recorder.writer)

    def test_repeated_close_is_safe(self):
        with patch.object(self.module.shutil, 'which', return_value=None):
            self.recorder.close()
            self.recorder.close()
        self.writer.release.assert_called_once()


if __name__ == '__main__':
    unittest.main()
