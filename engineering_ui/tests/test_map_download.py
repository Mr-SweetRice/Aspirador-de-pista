from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ble.map_download import MapDownload


class MapDownloadTests(unittest.TestCase):
    def test_two_panels_do_not_start_two_downloads(self):
        download = MapDownload()
        self.assertTrue(download.start(0))
        self.assertFalse(download.start(0))
        self.assertTrue(download.accept(0, 0, 12, 855))
        self.assertFalse(download.accept(0, 0, 12, 855))
        self.assertEqual(download.offset, 12)

    def test_duplicates_during_complete_map_do_not_reset_progress(self):
        download = MapDownload()
        download.start(0)
        for offset in range(0, 855, 12):
            count = min(12, 855 - offset)
            self.assertTrue(download.accept(0, offset, count, 855))
            self.assertFalse(download.accept(0, offset, count, 855))
        self.assertTrue(download.complete)

    def test_switch_slot_rejects_old_reply(self):
        download = MapDownload()
        download.start(0)
        download.start(1)
        self.assertFalse(download.accept(0, 0, 12, 855))
        self.assertTrue(download.accept(1, 0, 12, 317))

    def test_out_of_order_and_truncated_replies_do_not_advance(self):
        download = MapDownload()
        download.start(0)
        self.assertFalse(download.accept(0, 12, 12, 855))
        self.assertFalse(download.accept(0, 0, 0, 855))
        self.assertEqual(download.offset, 0)
