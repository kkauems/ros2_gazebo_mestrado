"""Lógica da detecção de patinagem (slip_detector), sem ROS rodando."""

import random

from agrobot_webots.slip_detector import SlipDetector

WHEEL_DT = 1 / 31.25    # driver: passo de 32 ms
VO_DT = 0.1             # câmera RGB-D a ~10 Hz
LOST_VARIANCE = 9999.0


def run(detector, duration, wheel_speed, camera, start=0.0):
    """
    Roda o detector por duration segundos.

    wheel_speed(t) dá a velocidade das rodas; camera(t) dá (vx, variância) da
    VO. Devolve a lista (t, patinando, saída das rodas para o EKF).
    """
    log = []
    t = start
    next_vo = start + VO_DT
    while t < start + duration:
        t += WHEEL_DT
        detector.add_wheel(t, wheel_speed(t), 0.0)
        if t >= next_vo:
            vx, variance = camera(next_vo)
            detector.add_reference(next_vo, vx, 0.0, variance, variance)
            next_vo += VO_DT
        log.append((t, detector.slipping, detector.wheel_output(t)))
    return log


def noisy(value, rng, std=0.03, variance=0.001):
    return lambda t: (value(t) + rng.gauss(0.0, std), variance)


def test_normal_driving_no_alarm():
    rng = random.Random(1)
    detector = SlipDetector()

    def speed(t):
        return min(0.4, 0.5 * t)    # acelera até 0,4 m/s
    run(detector, 60.0, speed, noisy(speed, rng))
    assert detector.slip_count == 0


def test_spike_after_vo_loss_is_ignored():
    """O caso dos alarmes falsos: perda, depois ~1 s a -2 m/s."""
    rng = random.Random(2)
    detector = SlipDetector()

    def camera(t):
        if 10.0 <= t < 10.3:
            return 0.0, LOST_VARIANCE
        if 10.3 <= t < 11.3:
            return -2.0 + rng.gauss(0.0, 0.1), 0.001
        return 0.4 + rng.gauss(0.0, 0.03), 0.001
    log = run(detector, 20.0, lambda t: 0.4, camera)
    assert detector.slip_count == 0
    assert all(output is None for _, _, output in log)
    assert detector.rejected['rapida demais'] > 0
    assert detector.rejected['logo apos perda'] == 3


def test_short_plausible_disagreement_is_ignored():
    """Uma discordância de 1 s, com velocidade possível, não é patinagem."""
    rng = random.Random(3)
    detector = SlipDetector()

    def camera(t):
        value = -0.3 if 10.0 <= t < 11.0 else 0.4
        return value + rng.gauss(0.0, 0.03), 0.001
    run(detector, 20.0, lambda t: 0.4, camera)
    assert detector.slip_count == 0


def test_stuck_robot_is_detected_and_released():
    """Preso numa pedra de 5 s a 11 s: rodas a 0,5 m/s, câmera parada."""
    rng = random.Random(4)
    detector = SlipDetector()

    def camera(t):
        value = 0.0 if 5.0 <= t < 11.0 else 0.5
        return value + rng.gauss(0.0, 0.03), 0.001
    log = run(detector, 20.0, lambda t: 0.5, camera)
    assert detector.slip_count == 1
    started = next(t for t, slipping, _ in log if slipping)
    ended = next(t for t, slipping, _ in log if t > started and not slipping)
    assert 5.0 < started < 7.0             # detecta em até ~2 s
    assert 11.0 < ended < 12.5
    # Patinando, o EKF recebe a câmera (~0), não as rodas (0,5).
    outputs = [o for t, _, o in log if 7.0 < t < 11.0]
    assert all(o is not None and abs(o[0]) < 0.1 for o in outputs)
    # Sem patinagem, as rodas passam sem mudança.
    assert all(o is None for t, _, o in log if t < 5.0 or t > 13.0)


def test_stuck_with_camera_dropouts_holds_camera_speed():
    """Preso e sacudindo: a câmera se perde por 1 s, a patinagem continua."""
    rng = random.Random(5)
    detector = SlipDetector()

    def camera(t):
        if 8.0 <= t < 9.0:
            return 0.0, LOST_VARIANCE
        value = 0.0 if 5.0 <= t < 14.0 else 0.5
        return value + rng.gauss(0.0, 0.03), 0.001
    log = run(detector, 20.0, lambda t: 0.5, camera)
    assert detector.slip_count == 1
    during = [(t, o) for t, _, o in log if 8.0 < t < 9.3]
    assert all(o is not None and abs(o[0]) < 0.1 for _, o in during)
    # A incerteza cresce enquanto a câmera não volta.
    assert during[-1][1][2] > during[0][1][2]


def test_camera_gone_too_long_gives_wheels_back():
    rng = random.Random(6)
    detector = SlipDetector(hold_time=3.0)

    def camera(t):
        if t >= 8.0:
            return 0.0, LOST_VARIANCE
        value = 0.0 if t >= 5.0 else 0.5
        return value + rng.gauss(0.0, 0.03), 0.001
    log = run(detector, 15.0, lambda t: 0.5, camera)
    assert detector.slip_count == 1
    assert all(o is None for t, _, o in log if t > 11.2)


def test_dragged_robot_is_detected():
    """Rodas paradas e a câmera vendo o robô andar (arrastado)."""
    rng = random.Random(7)
    detector = SlipDetector()

    def camera(t):
        value = 0.3 if t >= 5.0 else 0.0
        return value + rng.gauss(0.0, 0.03), 0.001
    run(detector, 10.0, lambda t: 0.0, camera)
    assert detector.slip_count == 1
    assert detector.slipping
