from sar_mission.registry_core import VictimRegistry


def test_same_id_merges_into_one_victim():
    reg = VictimRegistry(min_observations=3)
    for i in range(10):
        reg.add_detection(0, -4.0 + 0.01 * i, 4.9, 0.25, 2.0, i)
    assert len(reg.confirmed()) == 1
    assert reg.confirmed()[0].observations == 10


def test_nearby_misread_id_merges_and_keeps_majority_id():
    reg = VictimRegistry(merge_distance=0.5, min_observations=3)
    for i in range(5):
        reg.add_detection(5, -1.2, 0.08, 0.25, 2.0, i)
    _, event = reg.add_detection(7, -1.1, 0.1, 0.25, 2.0, 6)
    assert event == VictimRegistry.MERGED_NEAR
    assert len(reg.tracks) == 1
    assert reg.confirmed()[0].marker_id == 5


def test_far_same_id_outlier_rejected_after_confirmation():
    reg = VictimRegistry(outlier_distance=1.0, min_observations=3)
    for i in range(3):
        reg.add_detection(1, 5.9, 3.0, 0.25, 2.0, i)
    _, event = reg.add_detection(1, 3.0, 3.0, 0.25, 2.0, 4)
    assert event == VictimRegistry.REJECTED
    assert abs(reg.confirmed()[0].x - 5.9) < 1e-9


def test_different_ids_far_apart_are_separate_victims():
    reg = VictimRegistry(min_observations=1)
    reg.add_detection(0, -4.0, 4.9, 0.25, 2.0, 0)
    reg.add_detection(2, 4.0, -4.9, 0.25, 2.0, 1)
    assert [t.marker_id for t in reg.confirmed()] == [0, 2]


def test_unconfirmed_single_frame_not_reported():
    reg = VictimRegistry(min_observations=3)
    reg.add_detection(3, -5.9, -2.0, 0.25, 2.0, 0)
    assert reg.confirmed() == []


def test_close_observations_weigh_more():
    reg = VictimRegistry(min_observations=1)
    reg.add_detection(4, 1.0, 0.0, 0.25, 1.0, 0)   # weight 1
    reg.add_detection(4, 2.0, 0.0, 0.25, 3.0, 1)   # weight 1/9
    assert abs(reg.confirmed()[0].x - (1.0 * 1 + 2.0 / 9) / (1 + 1 / 9)) < 1e-9
