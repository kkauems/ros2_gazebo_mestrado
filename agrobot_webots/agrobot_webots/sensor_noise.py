"""
Ruído dos sensores do Agrobot no Webots.

Os sensores do Webots são perfeitos: o InertialUnit dá a orientação exata,
os encoders não têm resolução e o LiDAR e a câmera RGB-D medem sem erro. Isso
favorece quem confia neles (o EKF confia no yaw do IMU). Aqui entra o erro
de sensores reais de baixo custo, aplicado pelo agrobot_driver antes de
publicar. Os valores vêm de datasheets e estão justificados ao lado de cada
constante; todos os desvios são 1 sigma.

Sem ROS: o módulo só usa numpy, para poder ser testado sem o Webots.
"""

import math
import os

import numpy as np

G = 9.80665
DEG = math.pi / 180.0

# --- IMU: MEMS de 6 eixos de classe consumidor (BMI160, MPU-6050,
# ICM-20948), calibrada parada ao ligar, sem magnetômetro.
#
# Giroscópio. Densidade de ruído: BMI160 0,007, MPU-6050 0,005 e ICM-20948
# 0,015 °/s/√Hz; 0,01 °/s/√Hz equivale a um ARW de 0,6 °/√h.
GYRO_NOISE_DENSITY = 0.01 * DEG                 # rad/s/√Hz
# O offset de fábrica (±3 a ±20 °/s) sai na calibração parada ao ligar; o que
# sobra é o erro dessa média mais a variação com a temperatura: 0,01 °/s
# (36 °/h) por eixo, fixo na rodada.
GYRO_TURN_ON_BIAS_STD = 0.01 * DEG              # rad/s
# Instabilidade do bias de MEMS de consumo: 10 a 30 °/h. Processo de
# Gauss-Markov de 1a ordem com 0,005 °/s (18 °/h) e 2 min de correlação.
GYRO_BIAS_INSTABILITY = 0.005 * DEG             # rad/s
GYRO_BIAS_TAU = 120.0                           # s
# Erro de fator de escala: ±1% (BMI160) a ±3% (MPU-6050) antes de calibrar;
# 1% depois. Cada giro de 180° erra ~1,8° no yaw integrado.
GYRO_SCALE_STD = 0.01
#
# Acelerômetro. Densidade de ruído: BMI160 180, MPU-6050 400 µg/√Hz.
ACCEL_NOISE_DENSITY = 200e-6 * G                # m/s²/√Hz
# Offset residual depois de calibrar (BMI160: ±40 mg de fábrica).
ACCEL_BIAS_STD = 0.02 * G                       # m/s²
#
# Orientação. Sem magnetômetro (no campo ele sofre com motores, cercas e
# implementos de ferro), a fusão a bordo integra o giroscópio em yaw: o erro
# de yaw é a integral do erro do giroscópio em z e não volta para zero.
# Roll e pitch vêm da gravidade: o offset do acelerômetro vira inclinação fixa
# (20 mg -> ~1,1°), mais um ruído pequeno.
TILT_NOISE_STD = 0.1 * DEG                      # rad

# --- Encoders das rodas. 1024 pulsos em quadratura = 4096 contagens por
# volta da roda (0,34 mm de rodado). O raio efetivo de um pneu agrícola varia
# com a carga e a pressão: 1% de erro comum depois de calibrar, mais 0,3%
# de diferença entre os lados (o que faz a odometria curvar sozinha).
ENCODER_COUNTS_PER_REV = 4096
WHEEL_RADIUS_STD = 0.01
WHEEL_RADIUS_SIDE_STD = 0.003

# --- LiDAR 2D (classe SICK TiM5xx / Hokuyo UST-10LX / RPLIDAR S2): erro
# estatístico de 20 mm em cada raio.
LIDAR_RANGE_STD = 0.02                          # m

# --- Câmera RGB-D (Intel RealSense D455, estéreo ativo). O erro da
# profundidade cresce com o quadrado da distância:
#   sigma_z = z² * subpixel / (f * B)
# com base B = 95 mm, f = 447 px (848 px de largura e 87° de FOV, a resolução
# nativa da profundidade) e subpixel de 0,1 px (a Intel cita 0,05 a 0,1;
# usamos o pior, porque é externo). Dá 2,4 mm a 1 m e 3,8 cm a 4 m (<1%,
# dentro do "<2% a 4 m" do datasheet).
DEPTH_BASELINE = 0.095                          # m
DEPTH_FOCAL_PX = 447.0
DEPTH_SUBPIXEL = 0.1                            # px
DEPTH_NOISE_PER_M2 = DEPTH_SUBPIXEL / (DEPTH_FOCAL_PX * DEPTH_BASELINE)
# Cor: ruído de leitura e de fóton de um sensor CMOS de dia, ~2 níveis de
# 255 (0,8%).
COLOR_NOISE_STD = 2.0                           # níveis de 0 a 255


def random_seed():
    """Semente nova (para registrar no log e poder repetir a rodada)."""
    return int.from_bytes(os.urandom(4), 'little') or 1


def quaternion_multiply(a, b):
    """Produto de quaternions (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class GaussMarkov:
    """Processo de Gauss-Markov de 1a ordem (bias que vaga e volta)."""

    def __init__(self, rng, std, tau, size):
        self.rng = rng
        self.std = std
        self.tau = tau
        self.value = rng.normal(0.0, std, size)

    def step(self, dt):
        decay = math.exp(-dt / self.tau)
        self.value = (self.value * decay + self.std * math.sqrt(
            1.0 - decay * decay) * self.rng.standard_normal(self.value.shape))
        return self.value


class ImuNoise:
    """Giroscópio, acelerômetro e orientação de uma IMU MEMS de 6 eixos."""

    def __init__(self, rng):
        self.rng = rng
        self.gyro_bias = rng.normal(0.0, GYRO_TURN_ON_BIAS_STD, 3)
        self.gyro_drift = GaussMarkov(
            rng, GYRO_BIAS_INSTABILITY, GYRO_BIAS_TAU, 3)
        self.gyro_scale = 1.0 + rng.normal(0.0, GYRO_SCALE_STD, 3)
        self.accel_bias = rng.normal(0.0, ACCEL_BIAS_STD, 3)
        # Inclinação fixa que o offset do acelerômetro causa (pequenos
        # ângulos): a gravidade "aparece" deslocada em x e y.
        self.roll_bias = self.accel_bias[1] / G
        self.pitch_bias = -self.accel_bias[0] / G
        self.yaw_error = 0.0
        self.gyro = None
        self.accel = None

    def update(self, dt, true_gyro, true_accel):
        """Mede um passo de dt segundos; guarda gyro, accel e yaw_error."""
        # Ruído branco discreto: densidade / sqrt(dt).
        white = math.sqrt(1.0 / dt)
        gyro = None
        if true_gyro is not None:
            true_gyro = np.asarray(true_gyro, dtype=float)
            gyro = (true_gyro * self.gyro_scale + self.gyro_bias
                    + self.gyro_drift.step(dt)
                    + GYRO_NOISE_DENSITY * white
                    * self.rng.standard_normal(3))
            # A fusão integra o giroscópio em yaw: o erro em z acumula.
            self.yaw_error += float(gyro[2] - true_gyro[2]) * dt
            self.yaw_error = math.atan2(
                math.sin(self.yaw_error), math.cos(self.yaw_error))
        accel = None
        if true_accel is not None:
            accel = (np.asarray(true_accel, dtype=float) + self.accel_bias
                     + ACCEL_NOISE_DENSITY * white
                     * self.rng.standard_normal(3))
        self.gyro = None if gyro is None else [float(v) for v in gyro]
        self.accel = None if accel is None else [float(v) for v in accel]

    def tilt_error(self):
        roll = self.roll_bias + self.rng.normal(0.0, TILT_NOISE_STD)
        pitch = self.pitch_bias + self.rng.normal(0.0, TILT_NOISE_STD)
        return roll, pitch

    def orientation(self, q):
        """
        Orientação medida a partir da verdadeira (x, y, z, w).

        Yaw errado em torno do z do mundo (deriva da integração) e roll/pitch
        errados no corpo (inclinação vista pela gravidade).
        """
        roll, pitch = self.tilt_error()
        half = self.yaw_error / 2.0
        yaw_error = (0.0, 0.0, math.sin(half), math.cos(half))
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        tilt_error = (sr * cp, cr * sp, -sr * sp, cr * cp)
        return quaternion_multiply(
            quaternion_multiply(yaw_error, tuple(q)), tilt_error)


class WheelEncoderNoise:
    """Resolução do encoder e raio efetivo diferente do nominal."""

    def __init__(self, rng):
        common = rng.normal(0.0, WHEEL_RADIUS_STD)
        self.scale = {
            side: 1.0 + common + rng.normal(0.0, WHEEL_RADIUS_SIDE_STD)
            for side in ('left', 'right')}
        self.step = 2.0 * math.pi / ENCODER_COUNTS_PER_REV

    def measure(self, side, angle):
        """
        Ângulo da roda (rad) como o driver o vê.

        O erro de raio entra como escala no ângulo: o driver multiplica pelo
        raio nominal e erra a distância na mesma proporção.
        """
        return math.floor(angle * self.scale[side] / self.step) * self.step


class SensorNoise:
    """Todos os ruídos, com um gerador só (repetível pela semente)."""

    def __init__(self, seed):
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.imu = ImuNoise(self.rng)
        self.wheels = WheelEncoderNoise(self.rng)
        self.color_table = None

    def lidar(self, ranges):
        """Ruído em cada raio; +inf (sem retorno) fica +inf."""
        values = np.asarray(ranges, dtype=np.float32)
        finite = np.isfinite(values)
        values[finite] += (LIDAR_RANGE_STD * self.rng.standard_normal(
            int(finite.sum()), dtype=np.float32))
        return values

    def depth(self, depth_bytes):
        """Profundidade float32 em bytes -> bytes com o ruído da D455."""
        depth = np.frombuffer(depth_bytes, dtype=np.float32).copy()
        finite = np.isfinite(depth)
        z = depth[finite]
        depth[finite] = z + (DEPTH_NOISE_PER_M2 * z * z
                             * self.rng.standard_normal(
                                 z.size, dtype=np.float32))
        return depth.tobytes()

    def color(self, bgra_bytes):
        """
        Imagem BGRA em bytes -> bytes com ruído em B, G e R.

        Sortear 300 mil normais por quadro custava ~9 ms no driver; o ruído
        sai de uma tabela sorteada uma vez (duas imagens), lida a partir de
        um pixel sorteado a cada quadro. Assim o padrão muda de quadro para
        quadro e o custo cai para ~1 ms.
        """
        image = np.frombuffer(bgra_bytes, dtype=np.uint8)
        pixels = image.size // 4
        table = self.color_table
        if table is None or table.shape[0] < 2 * pixels:
            table = np.rint(COLOR_NOISE_STD * self.rng.standard_normal(
                (2 * pixels, 4))).astype(np.int16)
            table[:, 3] = 0             # alfa sem ruído
            self.color_table = table
        start = int(self.rng.integers(0, table.shape[0] - pixels + 1))
        noisy = image.astype(np.int16)
        noisy += table[start:start + pixels].reshape(-1)
        np.clip(noisy, 0, 255, out=noisy)
        return noisy.astype(np.uint8).tobytes()
