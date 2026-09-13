"""Continuously drain live UVC capture so processing sees the newest decoded frame."""
import threading
import time


class LatestFrameCapture:
    def __init__(self, capture):
        self.capture = capture
        self.condition = threading.Condition()
        self.stopped = threading.Event()
        self.frame = None
        self.sequence = 0
        self.consumed = 0
        self.failed = False
        self.last_capture_monotonic = 0.0
        self._timestamp = 0.0
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        try:
            while not self.stopped.is_set():
                ok, frame = self.capture.read()
                timestamp = time.monotonic()
                with self.condition:
                    if not ok or frame is None:
                        self.failed = True
                        self.condition.notify_all()
                        break
                    self.frame = frame
                    self._timestamp = timestamp
                    self.sequence += 1
                    self.condition.notify_all()
        finally:
            self.capture.release()
            with self.condition:
                self.failed = True
                self.condition.notify_all()

    def read(self):
        with self.condition:
            ready = self.condition.wait_for(
                lambda: self.sequence != self.consumed or self.failed or self.stopped.is_set(), timeout=1.0
            )
            if not ready or self.failed or self.stopped.is_set():
                return False, None
            self.consumed = self.sequence
            self.last_capture_monotonic = self._timestamp
            return True, self.frame

    def release(self):
        self.stopped.set()
        with self.condition:
            self.condition.notify_all()
        # The reader owns release(); avoid releasing VideoCapture during a driver read.
        self.thread.join(timeout=1.5)
