"""Victim registry core: turns a stream of per-frame detections into one entry per victim.

Pure Python (no ROS) so the dedup rules can be unit-tested.

Dedup rules, applied to each detection (marker_id, x, y in map frame, camera range):
  1. Same ArUco ID as an existing victim  -> same victim.
     Exception: if that victim is already confirmed and the detection is farther than
     outlier_distance from it, the detection is rejected as a bad pose / TF moment.
  2. Else, within merge_distance of an existing victim (any ID) -> same victim.
     Covers a misread ID on a marker already in the registry.
  3. Else -> new victim.

Merged position = weighted mean of all observations, weight = 1 / range^2,
because pose error grows with distance to the marker.
A victim is reported only after min_observations detections (rejects one-frame false positives).
"""
import math
from collections import Counter
from dataclasses import dataclass, field


@dataclass
class VictimTrack:
    x: float
    y: float
    z: float
    weight_sum: float = 0.0
    observations: int = 0
    id_votes: Counter = field(default_factory=Counter)
    best_range: float = math.inf
    first_seen: float = 0.0
    last_seen: float = 0.0

    @property
    def marker_id(self):
        # Majority vote, so a rare misread cannot rename a victim.
        return self.id_votes.most_common(1)[0][0]

    def distance_xy(self, x, y):
        return math.hypot(self.x - x, self.y - y)

    def add(self, marker_id, x, y, z, rng, stamp, weight):
        total = self.weight_sum + weight
        self.x = (self.x * self.weight_sum + x * weight) / total
        self.y = (self.y * self.weight_sum + y * weight) / total
        self.z = (self.z * self.weight_sum + z * weight) / total
        self.weight_sum = total
        self.observations += 1
        self.id_votes[marker_id] += 1
        self.best_range = min(self.best_range, rng)
        if self.observations == 1:
            self.first_seen = stamp
        self.last_seen = stamp


class VictimRegistry:
    NEW = 'new'
    MERGED_ID = 'merged_same_id'
    MERGED_NEAR = 'merged_nearby'
    REJECTED = 'rejected_outlier'

    def __init__(self, merge_distance=0.5, outlier_distance=1.0, min_observations=3, min_weight_range=0.5):
        self.merge_distance = merge_distance
        self.outlier_distance = outlier_distance
        self.min_observations = min_observations
        self.min_weight_range = min_weight_range
        self.tracks = []

    def weight(self, rng):
        return 1.0 / max(rng, self.min_weight_range) ** 2

    def add_detection(self, marker_id, x, y, z, rng, stamp):
        """Returns (track, event). track is None only for rejected detections with no match."""
        w = self.weight(rng)

        same_id = [t for t in self.tracks if t.marker_id == marker_id]
        if same_id:
            track = min(same_id, key=lambda t: t.distance_xy(x, y))
            if (track.observations >= self.min_observations
                    and track.distance_xy(x, y) > self.outlier_distance):
                return track, self.REJECTED
            track.add(marker_id, x, y, z, rng, stamp, w)
            return track, self.MERGED_ID

        nearby = [t for t in self.tracks if t.distance_xy(x, y) <= self.merge_distance]
        if nearby:
            track = min(nearby, key=lambda t: t.distance_xy(x, y))
            track.add(marker_id, x, y, z, rng, stamp, w)
            return track, self.MERGED_NEAR

        track = VictimTrack(x=x, y=y, z=z)
        track.add(marker_id, x, y, z, rng, stamp, w)
        self.tracks.append(track)
        return track, self.NEW

    def is_confirmed(self, track):
        return track.observations >= self.min_observations

    def confirmed(self):
        return sorted((t for t in self.tracks if self.is_confirmed(t)), key=lambda t: t.marker_id)

    def to_dict(self):
        return {
            'frame_id': 'map',
            'dedup': {
                'merge_distance_m': self.merge_distance,
                'outlier_distance_m': self.outlier_distance,
                'min_observations': self.min_observations,
                'weighting': '1/range^2',
            },
            'victim_count': len(self.confirmed()),
            'victims': [
                {
                    'id': t.marker_id,
                    'x': round(t.x, 3),
                    'y': round(t.y, 3),
                    'z': round(t.z, 3),
                    'observations': t.observations,
                    'best_range_m': round(t.best_range, 3),
                    'first_seen_s': round(t.first_seen, 2),
                    'last_seen_s': round(t.last_seen, 2),
                }
                for t in self.confirmed()
            ],
        }
