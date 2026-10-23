"""Pruebas sin ROS de route_around (regreso a casa rodeando zonas prohibidas)."""
import math

from control_bebop.safety import _crosses, route_around

# Aprox. la estación de bomberos en el marco del Matrice (drone_gazebo/config/matrice.yaml)
BUILDING = (-16.0, 0.86, 12.0, 19.14)


def _segments(start, route):
    pts = [start] + route
    return list(zip(pts, pts[1:]))


def test_clear_path_goes_straight():
    assert route_around((0.0, -9.6), (0.0, 0.0), [BUILDING], 2.0) == [(0.0, 0.0)]


def test_no_boxes_goes_straight():
    assert route_around((5.0, 5.0), (0.0, 0.0), [], 2.0) == [(0.0, 0.0)]


def test_far_side_of_building_goes_around():
    start = (-2.0, 30.0)   # punto de la órbita detrás del edificio
    route = route_around(start, (0.0, 0.0), [BUILDING], 2.0)
    assert route[-1] == (0.0, 0.0)
    assert len(route) == 3   # dos esquinas
    for a, b in _segments(start, route):
        assert not _crosses(a, b, BUILDING, 0.0)
    # Por el lado corto (x = 12 + 2) y no por el largo (x = -16 - 2)
    assert all(math.isclose(x, 14.0) for x, _ in route[:-1])


def test_start_inside_margin_still_avoids_building():
    start = (0.0, 20.0)    # a 0.86 m de la pared trasera
    route = route_around(start, (0.0, 0.0), [BUILDING], 2.0)
    for a, b in _segments(start, route):
        assert not _crosses(a, b, BUILDING, 0.0)


def test_home_next_to_building_is_reachable():
    # Home queda a 0.86 m del edificio, dentro del margen
    route = route_around((14.0, 21.14), (0.0, 0.0), [BUILDING], 2.0)
    assert route[-1] == (0.0, 0.0)
    for a, b in _segments((14.0, 21.14), route):
        assert not _crosses(a, b, BUILDING, 0.0)


def test_touching_edge_is_not_crossing():
    assert not _crosses((0.0, 0.0), (2.0, 0.0), (0.0, 0.0, 1.0, 1.0), 0.0)
    assert _crosses((-1.0, 0.5), (2.0, 0.5), (0.0, 0.0, 1.0, 1.0), 0.0)
