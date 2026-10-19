"""Pruebas sin ROS del ritmo de cuadros de nodo_video."""
import pytest

from control_bebop.frame_pacer import FramePacer


def _total(pacer, stamps):
    return sum(pacer.copies(t) for t in stamps)


def test_camera_faster_than_video_drops_frames():
    # Bebop real: ~33 Hz durante 10 s en un video de 15 fps → ~150 cuadros
    p = FramePacer(15.0)
    stamps = [i / 33.0 for i in range(330)]
    assert _total(p, stamps) == pytest.approx(15.0 * stamps[-1] + 1, abs=1)


def test_camera_slower_than_video_repeats_frames():
    # Gazebo lento: 7 Hz en un video de 15 fps; la duración se conserva
    p = FramePacer(15.0)
    stamps = [i / 7.0 for i in range(71)]
    assert _total(p, stamps) == pytest.approx(15.0 * stamps[-1] + 1, abs=1)


def test_same_rate_writes_each_frame_once():
    p = FramePacer(30.0)
    assert [p.copies(100.0 + i / 30.0) for i in range(60)] == [1] * 60


def test_long_gap_and_clock_jump_do_not_fill():
    p = FramePacer(10.0, max_gap=1.0)
    assert _total(p, [0.0, 0.1, 0.2]) == 3
    assert p.copies(30.0) == 1           # pausa de 30 s: no 300 cuadros
    assert p.copies(30.1) == 1
    assert p.copies(5.0) == 1            # reloj hacia atrás
    assert p.copies(5.1) == 1


def test_reset_starts_new_video():
    p = FramePacer(15.0)
    _total(p, [0.0, 1.0])
    p.reset()
    assert p.copies(50.0) == 1
    assert p.written == 1


def test_invalid_fps():
    with pytest.raises(ValueError):
        FramePacer(0.0)
