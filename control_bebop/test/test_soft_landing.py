"""Pruebas sin ROS del aterrizaje suave (SoftLanding) con un modelo simple del Bebop."""
import random

import pytest

from control_bebop.safety import SoftLanding, SoftLandingParams, landing_speed


def simulate(z0, gain, tau=0.4, delay=0.3, noise=0.02, p=None, seed=1, alt_hz=13.0):
    """Baja desde z0 con la rutina. El dron: vz = gain·gaz con retardo y lag de primer orden.

    La altura llega a ``alt_hz`` con ruido (sonar) y cuantizada a 1 cm. Devuelve un dict
    con la altura real y la velocidad de descenso cuando la rutina manda land.
    """
    p = p or SoftLandingParams()
    rng = random.Random(seed)
    dt, t = 0.05, 0.0
    z, v_down = z0, 0.0                       # v_down > 0 = bajando
    landing = SoftLanding(p)
    landing.start(t)
    cmds = [0.0] * int(delay / dt)            # retardo del comando
    z_meas, next_alt = z0, 0.0
    gazs, out = [], {}
    while t < 60.0:
        if t >= next_alt:
            z_meas = round(max(0.0, z + rng.gauss(0.0, noise)), 2)
            next_alt += 1.0 / alt_hz
        gaz, land_now, done, reason = landing.update(t, z_meas)
        gazs.append(gaz)
        if land_now:
            out.update(t_land=t, z_land=z, v_land=v_down, reason=reason)
        if done:
            out['t_done'] = t
            break
        cmds.append(gaz)
        v_target = -cmds.pop(0) * gain
        v_down += (v_target - v_down) * dt / tau
        z = max(0.0, z - v_down * dt)
        t += dt
    out['gazs'] = gazs
    return out


@pytest.mark.parametrize('gain', [0.4, 0.8, 1.1])
@pytest.mark.parametrize('z0', [0.6, 1.1, 1.8])
def test_arrives_slowly_for_any_measured_gain(gain, z0):
    r = simulate(z0, gain)
    assert 'v_land' in r and 't_done' in r
    assert r['reason'] == 'altura de contacto'
    assert r['z_land'] < 0.2                  # land a pocos cm del piso
    assert r['v_land'] < 0.2                  # el nativo llega a ~0.7 m/s
    assert r['t_land'] < 25.0


def test_never_commands_climb_and_respects_slew_and_cap():
    p = SoftLandingParams()
    gazs = simulate(1.2, 0.8)['gazs']
    assert max(gazs) <= 0.0
    assert min(gazs) >= -p.max_cmd
    while gazs and gazs[-1] == 0.0:           # tras land el control pasa al firmware (gaz → 0)
        gazs = gazs[:-1]
    steps = [abs(b - a) for a, b in zip(gazs, gazs[1:])]
    assert max(steps) <= p.slew * 0.2 + 1e-9  # dt de la rutina está acotado a 0.2 s


def test_starts_gently_from_hover():
    gazs = simulate(1.0, 0.8)['gazs']
    assert abs(gazs[0]) <= 0.05 * 0.5 + 1e-9  # el primer paso no da un escalón


def test_descent_speed_profile_is_monotonic_and_bounded():
    p = SoftLandingParams()
    speeds = [landing_speed(z / 100.0, p) for z in range(0, 200, 5)]
    assert speeds == sorted(speeds)
    assert min(speeds) == pytest.approx(p.v_touch)
    assert max(speeds) == pytest.approx(p.v_max)


def test_without_altitude_descends_by_time_then_lands():
    p = SoftLandingParams()
    landing = SoftLanding(p)
    landing.start(0.0)
    t, last = 0.0, None
    while t < 20.0:
        gaz, land_now, done, reason = landing.update(t, None)
        if land_now:
            last = (t, reason)
            break
        assert -p.max_cmd <= gaz <= 0.0
        t += 0.05
    assert last is not None and last[1] == 'sin altura medida'
    assert last[0] == pytest.approx(p.fallback_time, abs=0.1)


def test_stalled_altitude_near_floor_lands_instead_of_hovering():
    p = SoftLandingParams()
    landing = SoftLanding(p)
    landing.start(0.0)
    t = 0.0
    while t < 10.0:
        _, land_now, _, reason = landing.update(t, 0.2)   # el sonar se queda en 20 cm
        if land_now:
            break
        t += 0.05
    assert land_now and 'piso' in reason


def test_timeout_forces_land():
    p = SoftLandingParams(timeout=3.0)
    landing = SoftLanding(p)
    landing.start(0.0)
    t, reason = 0.0, ''
    while t < 6.0:
        _, land_now, _, reason = landing.update(t, 1.5)   # altura que nunca baja
        if land_now:
            break
        t += 0.05
    assert reason == 'tiempo máximo' and t == pytest.approx(3.0, abs=0.1)


def test_done_only_after_settle_and_gaz_is_zero_after_land():
    p = SoftLandingParams(settle_time=1.0)
    landing = SoftLanding(p)
    landing.start(0.0)
    t = 0.0
    while t < 2.0:                                # en el piso: z ≤ cut_height sostenida
        t += 0.05
        gaz, land_now, done, _ = landing.update(t, p.cut_height)
        if land_now:
            break
    assert land_now and not done and gaz == 0.0
    assert t == pytest.approx(p.contact_confirm, abs=0.06)
    assert landing.update(t + 0.5, 0.0)[2] is False
    assert landing.update(t + 1.2, 0.0)[2] is True


def test_single_low_spike_does_not_trigger_land():
    """Los bags del 28 oct tienen lecturas de 0.03–0.09 m de ~0.15 s estando a 0.4 m."""
    p = SoftLandingParams()
    landing = SoftLanding(p)
    landing.start(0.0)
    t = 0.0
    while t < 1.0:                                # baja normal desde 1.0 m
        landing.update(t, 1.0 - 0.1 * t)
        t += 0.05
    for _ in range(3):                            # pico de 0.15 s
        t += 0.05
        _, land_now, _, _ = landing.update(t, 0.05)
        assert not land_now
    t += 0.05
    _, land_now, _, _ = landing.update(t, 0.85)
    assert not land_now and landing.contact_since is None


def noisy_plateau(z_floor, noise, spike_every=25, seed=3, t_end=30.0, z0=1.2, v=0.25):
    """Baja a v m/s y se queda en z_floor (lectura sesgada); ruido y picos como en los bags."""
    rng = random.Random(seed)
    landing = SoftLanding(SoftLandingParams())
    landing.start(0.0)
    t, k, z_meas = 0.0, 0, z0
    t_floor = (z0 - z_floor) / v
    while t < t_end:
        if k % 4 == 0:                            # ~5 lecturas/s nuevas, el resto repite
            z = max(z_floor, z0 - v * t)
            z_meas = z + rng.gauss(0.0, noise)
            if (k // 4) % spike_every == spike_every - 1:
                z_meas = rng.choice((0.03, z + 0.25))
        _, land_now, _, reason = landing.update(t, z_meas)
        if land_now:
            return t, t_floor, reason
        t += 0.05
        k += 1
    return None, t_floor, ''


@pytest.mark.parametrize('z_floor', [0.25, 0.40, 0.50])
@pytest.mark.parametrize('seed', [1, 2, 3, 4, 5])
def test_biased_floor_reading_lands_soon_after_stalling(z_floor, seed):
    """Meseta en 0.25–0.5 m (la z medida en el piso, bags del 28 oct): land en pocos segundos."""
    t_land, t_floor, reason = noisy_plateau(z_floor, 0.08, seed=seed)
    assert t_land is not None and 'piso' in reason
    assert t_land - t_floor < 5.0


def test_slow_legitimate_descent_is_not_taken_for_a_stall():
    p = SoftLandingParams()
    landing = SoftLanding(p)
    landing.start(0.0)
    rng = random.Random(7)
    t, z_meas = 0.0, 0.0
    while t < 5.0:                                # 0.6 → 0.15 m a 0.1 m/s, ruido ±0.05
        if round(t / 0.05) % 4 == 0:
            z_meas = 0.6 - 0.1 * t + rng.gauss(0.0, 0.05)
        _, land_now, _, reason = landing.update(t, z_meas)
        assert not (land_now and 'piso' in reason), f'falso estancamiento a t={t:.2f}'
        if land_now:
            break
        t += 0.05


def run_with_resistance(b0, gain=1.0, floor=0.05, zone=0.5, seed=1, t_end=40.0):
    """Dron con zona de resistencia: bajo ``zone`` necesita gaz < −b0 para bajar (bags del
    28 oct: a ~0.45 m el gaz medio de −0.3 solo lo dejaba quieto). Pasa la vz de odometría
    (13 Hz, con ruido y picos). Devuelve (t_land, z_land, v_land, tiempo parado en la zona)."""
    rng = random.Random(seed)
    landing = SoftLanding(SoftLandingParams())
    landing.start(0.0)
    dt, t, z, v, tau = 0.05, 0.0, 1.2, 0.0, 0.4
    cmds = [0.0] * 6
    z_meas, vz_meas, parado = 1.2, 0.0, 0.0
    while t < t_end:
        if round(t / dt) % 4 == 0:
            z_meas = round(z + rng.gauss(0.0, 0.05), 2)
            vz_meas = -v + rng.gauss(0.0, 0.06)
            if rng.random() < 0.04:
                vz_meas += rng.choice((-0.5, 0.5))
        gaz, land_now, _, _ = landing.update(t, z_meas, vz_meas)
        if t > 2.0 and z < zone and abs(v) < 0.03:
            parado += dt
        if land_now:
            return t, z - floor, v, parado
        cmds.append(gaz)
        v += (gain * max(0.0, -cmds.pop(0) - (b0 if z < zone else 0.0)) - v) * dt / tau
        z = max(floor, z - v * dt)
        if z <= floor:
            v = 0.0
        t += dt
    return None


@pytest.mark.parametrize('b0', [0.0, 0.15, 0.3])
@pytest.mark.parametrize('seed', [1, 2, 3])
def test_pushes_through_resistance_near_floor_and_lands_soft(b0, seed):
    """El PI sube el gaz cuando el dron deja de bajar y atraviesa la zona.

    Con resistencia fuerte (0.3) prioriza no quedarse colgado: puede mandar land al frenarse.
    """
    t_land, z_land, v_land, parado = run_with_resistance(b0, seed=seed)
    if b0 <= 0.15:
        assert z_land < 0.15                  # land a pocos cm del piso
    assert v_land < 0.2                       # el nativo llega a ~0.7 m/s
    assert parado < 2.5


def test_gaz_rises_when_drone_stops_descending():
    """vz≈0 con el dron aún alto en la rampa: el gaz va más allá del gaz de crucero."""
    p = SoftLandingParams()
    landing = SoftLanding(p)
    landing.start(0.0)
    t, gazs = 0.0, []
    while t < 5.0:
        gaz, land_now, _, _ = landing.update(t, 1.0, 0.0)   # sin bajar a 1.0 m
        gazs.append(gaz)
        t += 0.05
    assert -gazs[-1] > landing_speed(1.0, p) / p.cmd_gain + 0.1
    assert -gazs[-1] <= p.max_cmd + 1e-9


def test_blocked_at_floor_height_lands_soon_by_vz():
    """Meseta a 0.4 m (el dron no baja más): land ~1 s después de frenarse, no 18 s."""
    p = SoftLandingParams()
    landing = SoftLanding(p)
    landing.start(0.0)
    t, land_at = 0.0, None
    while t < 20.0:
        z = max(0.4, 1.2 - 0.25 * t)
        vz = -0.25 if z > 0.4 else 0.0
        _, land_now, _, reason = landing.update(t, z, vz)
        if land_now:
            land_at = (t, reason)
            break
        t += 0.05
    t_block = (1.2 - 0.4) / 0.25
    assert land_at is not None and 'piso' in land_at[1]
    assert land_at[0] - t_block < 1.6


def test_vz_spike_does_not_trigger_stall():
    p = SoftLandingParams()
    landing = SoftLanding(p)
    landing.start(0.0)
    t = 0.0
    while t < 4.0:                            # baja 0.6 → 0.2 m a 0.1 m/s con vz limpia
        landing.update(t, 0.6 - 0.1 * t, -0.1)
        t += 0.05
    for _ in range(6):                        # pico de vz≈0 de 0.3 s
        t += 0.05
        _, land_now, _, _ = landing.update(t, 0.2, 0.0)
        assert not land_now
    for _ in range(40):                       # sigue bajando normal: sin land por estancamiento
        t += 0.05
        _, land_now, _, _ = landing.update(t, 0.2, -0.1)
        assert not land_now
