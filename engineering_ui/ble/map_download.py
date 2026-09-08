"""One map download shared by all panels; duplicate replies never fork it."""
class MapDownload:
    def __init__(self):
        self.slot = None
        self.offset = 0
        self.total = None
        self.attempts = 0

    def start(self, slot):
        if self.slot == slot:
            return False
        self.slot, self.offset, self.total, self.attempts = slot, 0, None, 0
        return True

    def cancel(self):
        self.slot = None

    def accept(self, slot, offset, count, total):
        if slot != self.slot or offset != self.offset:
            return False
        if count < 0 or total < offset + count or (count == 0 and offset < total):
            return False
        if self.total is not None and total != self.total:
            return False
        self.total = total
        self.offset += count
        self.attempts = 0
        return True

    @property
    def complete(self):
        return self.total is not None and self.offset == self.total
