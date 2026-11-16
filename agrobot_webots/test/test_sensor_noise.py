"""Ruído dos sensores (sensor_noise), sem ROS nem Webots."""

import math

import numpy as np

from agrobot_webots import sensor_noise as sn
from agrobot_webots.sensor_noise import SensorNoise

DT = 0.032                  # passo do driver


def drive_imu(noise, duration, yaw_rate=0.0):
    steps = int(round(duration / DT))
    gyros = []
    for _ in range(steps):
        noise.imu.update(DT, [0.0, 0.0, yaw_rate], [0.0, 0.0, sn.G])
        gyros.append(noise.imu.gyro)
    return np.array(gyros)


def test_same_seed_same_noise():
    a = drive_imu(SensorNoise(7), 5.0)
    b = drive_imu(SensorNoise(7), 5.0)
    c = drive_imu(SensorNoise(8), 5.0)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


def test_gyro_white_noise_matches_datasheet_density():
    gyros = drive_imu(SensorNoise(1), 60.0)
    # Desvio por amostra = densidade / sqrt(dt) (~0,056 °/s); o bias quase
    # não muda em 1 min, então o desvio das amostras é o do ruído branco.
    expected = sn.GYRO_NOISE_DENSITY / math.sqrt(DT)
    assert abs(gyros[:, 2].std() / expected - 1.0) < 0.1


def test_yaw_drifts_like_a_consumer_mems_imu():
    # Parado por 5 min (uma missão típica): a deriva de yaw tem que ser de
    # alguns graus, não zero (Webots puro) nem dezenas de graus.
    errors = []
    for seed in range(1, 41):
        noise = SensorNoise(seed)
        drive_imu(noise, 300.0)
        errors.append(noise.imu.yaw_error)
    rms = math.degrees(math.sqrt(np.mean(np.square(errors))))
    assert 1.0 < rms < 8.0


def test_gyro_scale_error_shows_up_in_turns():
    # Um giro de 360° em 2π/0,5 s: o erro de escala (1%) domina o do bias.
    errors = []
    for seed in range(1, 41):
        noise = SensorNoise(seed)
        drive_imu(noise, 2 * math.pi / 0.5, yaw_rate=0.5)
        errors.append(noise.imu.yaw_error)
    rms = math.degrees(math.sqrt(np.mean(np.square(errors))))
    # 1% de 360° = 3,6°.
    assert 2.0 < rms < 6.0


def test_orientation_carries_yaw_error_and_small_tilt():
    noise = SensorNoise(3)
    noise.imu.yaw_error = math.radians(10.0)
    x, y, z, w = noise.imu.orientation((0.0, 0.0, 0.0, 1.0))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    assert abs(math.degrees(yaw) - 10.0) < 0.1
    assert abs(math.degrees(roll)) < 5.0
    assert abs(math.degrees(pitch)) < 5.0
    assert abs(x * x + y * y + z * z + w * w - 1.0) < 1e-9


def test_wheel_distance_error_is_about_one_percent():
    errors = []
    radius = 0.22
    for seed in range(1, 41):
        noise = SensorNoise(seed)
        angle = 10.0 / radius          # 10 m de rodado
        measured = noise.wheels.measure('left', angle) * radius
        errors.append(measured / 10.0 - 1.0)
    assert 0.005 < np.std(errors) < 0.02


def test_encoder_resolution():
    noise = SensorNoise(1)
    step = 2 * math.pi / sn.ENCODER_COUNTS_PER_REV
    angle = noise.wheels.measure('right', 1.234)
    assert abs(angle / step - round(angle / step)) < 1e-9


def test_lidar_keeps_no_return_and_adds_two_cm():
    noise = SensorNoise(2)
    ranges = [3.0] * 5000 + [math.inf] * 10
    out = noise.lidar(ranges)
    assert np.all(np.isinf(out[-10:]))
    assert abs(out[:-10].std() - sn.LIDAR_RANGE_STD) < 0.002
    assert abs(out[:-10].mean() - 3.0) < 0.002


def test_depth_noise_grows_with_square_of_distance():
    noise = SensorNoise(4)
    depth = np.array([1.0] * 20000 + [4.0] * 20000 + [math.inf] * 5,
                     dtype=np.float32)
    out = np.frombuffer(noise.depth(depth.tobytes()), dtype=np.float32)
    assert np.all(np.isinf(out[-5:]))
    near = out[:20000].std()
    far = out[20000:40000].std()
    # D455: ~2,4 mm a 1 m e ~3,8 cm a 4 m (16 vezes mais).
    assert abs(near - 0.0024) < 0.0004
    assert abs(far - 0.038) < 0.004


def test_color_noise_spares_alpha():
    noise = SensorNoise(5)
    image = np.full((10000, 4), 128, dtype=np.uint8)
    image[:, 3] = 255
    out = np.frombuffer(noise.color(image.tobytes()),
                        dtype=np.uint8).reshape(-1, 4)
    assert np.all(out[:, 3] == 255)
    assert abs(out[:, :3].astype(float).std() - sn.COLOR_NOISE_STD) < 0.2
