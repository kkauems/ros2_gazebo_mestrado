"""Lógica do cmd_vel_mux e do teleop_keyboard (sem ROS rodando)."""

from agrobot_webots.cmd_vel_mux import MuxState
from agrobot_webots.teleop_keyboard import KeyboardState


def test_mux_teleop_has_priority_and_nav_returns():
    mux = MuxState(teleop_timeout=0.5, nav_timeout=0.5)
    assert mux.active(0.0) == 'none'
    assert mux.command(0.0) is None
    mux.receive('nav', 'nav_cmd', 1.0)
    assert mux.command(1.1) == 'nav_cmd'
    mux.receive('teleop', 'teleop_cmd', 1.2)
    mux.receive('nav', 'nav_cmd', 1.3)
    assert mux.active(1.3) == 'teleop'
    assert mux.command(1.3) == 'teleop_cmd'
    # Operador parou de mandar: o Nav2 volta.
    mux.receive('nav', 'nav_cmd', 1.8)
    assert mux.active(1.8) == 'nav'
    # Ninguém manda: nada a publicar.
    assert mux.active(2.4) == 'none'


def test_keyboard_steps_and_limits():
    state = KeyboardState()
    for _ in range(3):
        state.press('w')
    assert state.linear == 0.3
    for _ in range(10):
        state.press('\x1b[A')   # seta para cima
    assert state.linear == 0.5
    state.press('a')
    state.press('d')
    state.press('d')
    assert state.angular == -0.2
    assert state.publishing()


def test_keyboard_stop_hold_and_quit():
    state = KeyboardState()
    state.press('w')
    state.press(' ')
    assert not state.moving() and not state.publishing()
    state.press('p')
    assert state.hold and state.publishing()
    state.press('w')
    assert not state.hold and state.linear == 0.1
    state.press('x')            # tecla sem uso: nada muda
    assert state.linear == 0.1
    state.press('q')
    assert state.quit and not state.publishing()
