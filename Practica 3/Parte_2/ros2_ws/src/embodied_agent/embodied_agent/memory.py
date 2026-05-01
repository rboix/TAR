"""Memoria episódica y mapa semántico en RAM para brain_node."""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


@dataclass
class SemanticEntry:
    position: tuple[float, float]
    last_seen: str
    confidence: float


@dataclass
class ConversationTurn:
    timestamp: str
    user: str
    robot_said: str
    action: str
    robot_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)


class AgentMemory:
    def __init__(self, max_history: int = 10):
        self._max_history = max_history
        self.conversation_history: list[ConversationTurn] = []
        self.semantic_map: dict[str, SemanticEntry] = {}

    def add_turn(self, user: str, robot_said: str, action: str,
                 robot_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)):
        ts = datetime.now().strftime('%H:%M:%S')
        turn = ConversationTurn(
            timestamp=ts, user=user, robot_said=robot_said,
            action=action, robot_pose=robot_pose,
        )
        self.conversation_history.append(turn)
        if len(self.conversation_history) > self._max_history:
            self.conversation_history.pop(0)

    def update_semantic_map(self, label: str, position: tuple[float, float],
                            confidence: float = 0.8):
        ts = datetime.now().strftime('%H:%M:%S')
        self.semantic_map[label] = SemanticEntry(
            position=position, last_seen=ts, confidence=confidence)

    def get_object_position(self, label: str) -> Optional[tuple[float, float]]:
        entry = self.semantic_map.get(label)
        return entry.position if entry else None

    def format_history(self) -> str:
        if not self.conversation_history:
            return '(sin historial)'
        lines = []
        for t in self.conversation_history[-5:]:
            lines.append(
                f'[{t.timestamp}] Usuario: "{t.user}" → Robot: "{t.robot_said}" (acción: {t.action})')
        return '\n'.join(lines)

    def format_semantic_map(self) -> str:
        if not self.semantic_map:
            return '(mapa vacío)'
        lines = []
        for label, entry in self.semantic_map.items():
            lines.append(
                f'- {label}: posición {entry.position}, visto {entry.last_seen}, '
                f'confianza {entry.confidence:.0%}')
        return '\n'.join(lines)
