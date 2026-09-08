from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ui.map_view import MapView


class Button:
    def __init__(self, checked):
        self.checked = checked
        self.text = ''

    def isChecked(self):
        return self.checked

    def setChecked(self, value):
        self.checked = value

    def setText(self, text):
        self.text = text


class RecordingStateTests(unittest.TestCase):
    def test_old_active_snapshot_cannot_undo_stop(self):
        view = SimpleNamespace(_record_requested=False, _recording=False,
                               _syncing_record_button=False, record_button=Button(False))
        MapView._set_record_button_state(view, True)
        self.assertFalse(view.record_button.isChecked())
        self.assertIs(view._record_requested, False)
        MapView._set_record_button_state(view, False)
        self.assertIsNone(view._record_requested)
        self.assertFalse(view._recording)

    def test_old_stopped_snapshot_cannot_undo_start(self):
        view = SimpleNamespace(_record_requested=True, _recording=True,
                               _syncing_record_button=False, record_button=Button(True))
        MapView._set_record_button_state(view, False)
        self.assertTrue(view.record_button.isChecked())
        MapView._set_record_button_state(view, True)
        self.assertIsNone(view._record_requested)
