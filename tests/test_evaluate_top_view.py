"""The evaluator must not use its CSV after shutdown has closed it."""
import importlib.util
from collections import deque
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import MagicMock, patch


class EvaluationTest(unittest.TestCase):
    def setUp(self):
        names = [
            'cv2', 'numpy', 'rospy', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
            'nav_msgs.msg', 'std_msgs', 'std_msgs.msg', 'tf',
            'tf.transformations',
        ]
        dependencies = {name: MagicMock() for name in names}
        path = Path(__file__).resolve().parents[1] / 'scripts/evaluate_top_view.py'
        spec = importlib.util.spec_from_file_location('evaluation_under_test', path)
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, dependencies):
            spec.loader.exec_module(self.module)

        self.evaluation = self.module.Evaluation.__new__(self.module.Evaluation)
        self.evaluation.finished = False
        self.evaluation.closed = False
        self.evaluation.lock = threading.RLock()
        self.evaluation.truth = deque(maxlen=2000)
        self.evaluation.pending = deque(maxlen=100)
        self.evaluation.file = MagicMock()
        self.evaluation.writer = MagicMock()

    def test_close_is_idempotent_and_late_callbacks_are_ignored(self):
        self.evaluation.close()
        self.evaluation.close()

        self.evaluation.ground_truth(MagicMock())
        self.evaluation.vision(MagicMock())
        self.evaluation.done(MagicMock(data=True))

        self.evaluation.file.flush.assert_called_once_with()
        self.evaluation.file.close.assert_called_once_with()
        self.evaluation.writer.writerow.assert_not_called()

    def test_done_is_idempotent(self):
        self.evaluation.flush = MagicMock()

        self.evaluation.done(MagicMock(data=True))
        self.evaluation.done(MagicMock(data=True))

        self.assertTrue(self.evaluation.finished)
        self.evaluation.flush.assert_called_once_with()
        self.evaluation.file.flush.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
