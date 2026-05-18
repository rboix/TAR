"""Memoria episódica en RAM para brain_node.

Esta versión (Fase 3) introduce el campo `mode` y un buffer ligero de
observaciones acumuladas. La memoria episódica completa con plan de
investigación se desarrollará en la Fase 4.
"""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


Mode = str  # "guided" | "autonomous"


@dataclass
class ConversationTurn:
    timestamp: str
    user: str
    robot_said: str
    action: str
    robot_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class ObservationEntry:
    timestamp: str
    label: str
    detail: str = ''


class AgentMemory:
    def __init__(self, max_history: int = 10):
        self._max_history = max_history
        self.mode: Mode = 'guided'
        self.conversation_history: list[ConversationTurn] = []
        self.observations: list[ObservationEntry] = []

    def set_mode(self, mode: Mode) -> None:
        if mode not in ('guided', 'autonomous'):
            raise ValueError(f'Modo inválido: {mode}')
        self.mode = mode

    def add_turn(self, user: str, robot_said: str, action: str,
                 robot_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        ts = datetime.now().strftime('%H:%M:%S')
        turn = ConversationTurn(
            timestamp=ts, user=user, robot_said=robot_said,
            action=action, robot_pose=robot_pose,
        )
        self.conversation_history.append(turn)
        if len(self.conversation_history) > self._max_history:
            self.conversation_history.pop(0)

    def add_observations(self, observations: list[dict]) -> None:
        """Acumula observaciones (formato {label, detail})."""
        ts = datetime.now().strftime('%H:%M:%S')
        for obs in observations:
            label = obs.get('label', '').strip()
            if not label:
                continue
            self.observations.append(ObservationEntry(
                timestamp=ts,
                label=label,
                detail=obs.get('detail', '') or '',
            ))

    def format_mode(self) -> str:
        return 'AUTÓNOMO' if self.mode == 'autonomous' else 'GUIADO'

    def format_history(self) -> str:
        if not self.conversation_history:
            return '(sin historial previo)'
        lines = []
        for t in self.conversation_history[-5:]:
            lines.append(
                f'[{t.timestamp}] Usuario: "{t.user}" → Robot: "{t.robot_said}" '
                f'(acción: {t.action})'
            )
        return '\n'.join(lines)

    def format_observations(self) -> str:
        if not self.observations:
            return '(ninguna)'
        lines = []
        for o in self.observations[-10:]:
            detail = f' — {o.detail}' if o.detail else ''
            lines.append(f'[{o.timestamp}] {o.label}{detail}')
        return '\n'.join(lines)
