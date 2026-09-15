"""Timestamp-preserving streaming playback of recordings made by calibration.py."""
from contextlib import contextmanager
import json
import math
from pathlib import Path


@contextmanager
def video_clock(video, explicit=None):
    """Use our raw host-time sidecar when present; otherwise report FPS fallback.

    The sidecar is read incrementally: long videos do not accumulate timestamp
    arrays in memory. Missing/misaligned/non-monotonic rows fail explicitly.
    """
    path = Path(explicit).expanduser() if explicit is not None else None
    candidate = Path(video).expanduser().with_name('raw-timestamps.jsonl')
    if path is None and Path(video).name == 'raw.avi' and candidate.is_file():
        path = candidate
    stream = path.open() if path is not None else None
    previous = -math.inf

    def at(frame_id, fps):
        nonlocal previous
        if stream is None:
            return frame_id / fps
        line = stream.readline()
        if not line:
            raise ValueError('Video timestamp sidecar ended before the video')
        entry = json.loads(line)
        stamp = float(entry['elapsed_s'])
        if (entry.get('frame_id') != frame_id or not math.isfinite(stamp)
                or stamp < 0 or stamp <= previous):
            raise ValueError('Video timestamps must be aligned, finite, and strictly increasing')
        previous = stamp
        return stamp

    try:
        yield at, ('recorded_host_read_complete' if stream else 'video_frame_index_over_reported_fps')
    finally:
        if stream is not None:
            stream.close()
