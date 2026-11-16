"""
Lógica da detecção de patinagem, sem ROS (o nó é o wheel_slip_monitor).

Compara a velocidade das rodas com a da odometria visual (VO) da câmera RGB-D
e decide quando o robô está patinando (rodas girando com ele parado) ou sendo
arrastado (parado nas rodas, andando na câmera).

A VO erra de vez em quando, e um alarme falso faz o EKF seguir uma velocidade
errada. Por isso cada leitura dela passa por filtros antes de contar:
- variância válida (o rtabmap marca a odometria perdida com 9999 ou 0);
- velocidade possível para o robô (abaixo de max_reference_speed). Logo
  depois de se perder, a VO recomeça do zero (Odom/ResetCountdown: 1) e as
  primeiras velocidades saíam com uns 2 m/s para trás, com o robô a 0,4 m/s
  para frente;
- fora das skip_after_loss primeiras leituras depois de uma perda, pelo
  mesmo motivo.

E a decisão olha uma janela, não uma leitura:
- entra em patinagem quando, nas leituras válidas dos últimos detect_time
  segundos, há pelo menos min_samples e uma fração min_fraction delas
  discorda das rodas. Os alarmes falsos duravam ~1 s; uma pedra prende o
  robô por vários segundos;
- sai quando, nos últimos release_time segundos, a mesma fração concorda;
- patinando, o EKF recebe a mediana das velocidades válidas da VO no último
  meio segundo (uma leitura ruim sozinha não muda nada).
"""

from collections import deque
import math
from statistics import median

# Acima desta variância (m²/s²) a leitura da VO não vale (perdida = 9999).
MAX_REFERENCE_VARIANCE = 1.0
# Quanto das rodas guardar para comparar com a câmera (s).
WHEEL_HISTORY = 3.0
# Janela da mediana da VO usada no lugar das rodas (s).
OUTPUT_WINDOW = 0.5

# Motivos de descarte de uma leitura da VO (contados em rejected).
LOST = 'perdida'
TOO_FAST = 'rapida demais'
AFTER_LOSS = 'logo apos perda'


class SlipDetector:
    """Decide a patinagem a partir das rodas e da odometria visual."""

    def __init__(self, speed_tolerance=0.10, relative_tolerance=0.30,
                 detect_time=1.5, release_time=1.0, min_samples=8,
                 min_fraction=0.8, max_reference_speed=0.8,
                 skip_after_loss=3, hold_time=3.0,
                 min_reference_variance=0.0025, hold_std_growth=0.1):
        self.speed_tolerance = speed_tolerance
        self.relative_tolerance = relative_tolerance
        self.detect_time = detect_time
        self.release_time = release_time
        self.min_samples = min_samples
        self.min_fraction = min_fraction
        self.max_reference_speed = max_reference_speed
        self.skip_after_loss = skip_after_loss
        self.hold_time = hold_time
        self.min_reference_variance = min_reference_variance
        self.hold_std_growth = hold_std_growth

        self.wheels = deque()       # (t, vx, vy)
        # Leituras válidas da VO: (t, vx, vy, var_x, var_y, discorda)
        self.samples = deque()
        self.last_reference_time = None
        self.to_skip = 0
        self.slipping = False
        self.slip_started = None
        self.slip_count = 0
        self.rejected = {LOST: 0, TOO_FAST: 0, AFTER_LOSS: 0}

    # ---- entradas -------------------------------------------------------

    def add_wheel(self, t, vx, vy):
        self.wheels.append((t, vx, vy))
        while self.wheels and self.wheels[0][0] < t - WHEEL_HISTORY:
            self.wheels.popleft()

    def add_reference(self, t, vx, vy, var_x, var_y):
        """
        Uma leitura da VO.

        Devolve (mudou, motivo): mudou é True quando a patinagem começou ou
        acabou nesta leitura; motivo é um texto para o log.
        """
        previous = self.last_reference_time
        self.last_reference_time = t

        if not 0.0 < var_x < MAX_REFERENCE_VARIANCE:
            self.rejected[LOST] += 1
            self.to_skip = self.skip_after_loss
            return False, ''
        if not 0.0 < var_y < MAX_REFERENCE_VARIANCE:
            var_y = var_x
        if self.to_skip > 0:
            self.to_skip -= 1
            self.rejected[AFTER_LOSS] += 1
            return False, ''
        if math.hypot(vx, vy) > self.max_reference_speed:
            self.rejected[TOO_FAST] += 1
            return False, ''
        if previous is None or t <= previous:
            return False, ''

        # A velocidade da VO é o deslocamento entre dois quadros: compara com
        # a média das rodas no mesmo intervalo (a câmera chega atrasada, e
        # nas acelerações a última mensagem das rodas discordaria).
        wheel = self.wheel_mean(previous, t)
        if wheel is None:
            return False, ''
        difference = math.hypot(wheel[0] - vx, wheel[1] - vy)
        tolerance = (self.speed_tolerance + self.relative_tolerance
                     * max(math.hypot(*wheel), math.hypot(vx, vy)))
        disagree = difference > tolerance
        self.samples.append((t, vx, vy, var_x, var_y, disagree))
        horizon = max(self.detect_time, self.release_time, OUTPUT_WINDOW)
        while self.samples and self.samples[0][0] < t - horizon:
            self.samples.popleft()

        if not self.slipping:
            count, disagreeing = self.window(t, self.detect_time)
            if (count >= self.min_samples
                    and disagreeing >= self.min_fraction * count):
                self.start(t)
                return True, (f'rodas {wheel[0]:+.2f} m/s, câmera '
                              f'{vx:+.2f} m/s, {disagreeing}/{count} '
                              f'leituras discordando em '
                              f'{self.detect_time:.1f} s')
        else:
            count, disagreeing = self.window(t, self.release_time)
            needed = max(2, self.min_samples // 2)
            if (count >= needed
                    and count - disagreeing >= self.min_fraction * count):
                self.stop()
                return True, 'rodas e câmera concordam'
        return False, ''

    # ---- saída ----------------------------------------------------------

    def wheel_output(self, t):
        """
        O que o EKF recebe no lugar das rodas neste instante.

        None: passar as rodas sem mudança. Senão (vx, vy, var_x, var_y), a
        velocidade da câmera com a sua incerteza. Patinando sem leitura
        válida da câmera há mais de hold_time, desiste (stop) e devolve None.
        """
        if not self.slipping:
            return None
        if not self.samples or t - self.samples[-1][0] > self.hold_time:
            self.stop()
            return None
        last = self.samples[-1][0]
        recent = [s for s in self.samples if s[0] >= last - OUTPUT_WINDOW]
        vx = median(s[1] for s in recent)
        vy = median(s[2] for s in recent)
        var_x = max(median(s[3] for s in recent), self.min_reference_variance)
        var_y = max(median(s[4] for s in recent), self.min_reference_variance)
        # A incerteza cresce enquanto a câmera não manda leitura nova.
        extra = (self.hold_std_growth * max(t - last, 0.0)) ** 2
        return vx, vy, var_x + extra, var_y + extra

    # ---- auxiliares -----------------------------------------------------

    def wheel_mean(self, start, end):
        samples = [(vx, vy) for t, vx, vy in self.wheels if start < t <= end]
        if not samples:
            return None
        return (sum(s[0] for s in samples) / len(samples),
                sum(s[1] for s in samples) / len(samples))

    def window(self, t, duration):
        """(leituras, discordantes) válidas nos últimos duration segundos."""
        inside = [s for s in self.samples if s[0] > t - duration]
        return len(inside), sum(1 for s in inside if s[5])

    def start(self, t):
        self.slipping = True
        self.slip_started = t
        self.slip_count += 1

    def stop(self):
        self.slipping = False
