"""Memoria episódica en RAM para brain_node (Fase 4).

Estructura completa de session_state (sección 13 del CLAUDE.md):
  - mode: "guided" | "autonomous"
  - conversation_history: turnos usuario/robot con pose
  - observations: pistas acumuladas en la conversación, deduplicadas por label
  - investigation_observations: observaciones del modo autónomo, vinculadas a un paso
  - current_plan: plan de inspección del modo autónomo
  - last_pose: última pose conocida del robot (x, y, theta) en frame "map"/"odom"
"""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


Mode = str  # "guided" | "autonomous"
Pose = tuple[float, float, float]  # (x, y, theta)


@dataclass
class ConversationTurn:
    timestamp: str
    user: str
    robot_said: str
    action: str
    robot_pose: Pose = (0.0, 0.0, 0.0)


@dataclass
class ObservationEntry:
    """Pista detectada en la conversación. Deduplicada por `label`."""
    timestamp: str
    label: str
    detail: str = ''
    count: int = 1  # nº de veces que se ha vuelto a ver
    robot_pose: Pose = (0.0, 0.0, 0.0)


@dataclass
class InvestigationObservation:
    """Observación de un paso concreto del plan en modo autónomo."""
    step: int
    target: str
    observation: str
    timestamp: str
    robot_pose: Pose = (0.0, 0.0, 0.0)
    image_ref: Optional[str] = None


@dataclass
class PlanStep:
    step: int
    target: str
    reason: str = ''
    status: str = 'pending'  # pending | in_progress | completed | failed


class AgentMemory:
    """Memoria episódica del detective.

    Filosofía:
      - `observations` se deduplica por label: si Gemini vuelve a reportar
        "botella volcada" en otro turno, se actualiza el detalle y se
        incrementa el contador (en lugar de generar duplicados que
        ensucian el contexto).
      - `investigation_observations` NO se deduplica: cada paso del plan
        debe quedar registrado aunque coincida con una pista previa.
      - El historial conversacional se trunca al pasar `max_history`.
    """

    def __init__(self, max_history: int = 10):
        self._max_history = max_history
        self.mode: Mode = 'guided'
        self.conversation_history: list[ConversationTurn] = []
        self.observations: list[ObservationEntry] = []
        self.investigation_observations: list[InvestigationObservation] = []
        self.current_plan: list[PlanStep] = []
        self.last_pose: Pose = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------------ modo

    def set_mode(self, mode: Mode) -> None:
        if mode not in ('guided', 'autonomous'):
            raise ValueError(f'Modo inválido: {mode}')
        self.mode = mode

    # ------------------------------------------------------------------ pose

    def set_pose(self, pose: Pose) -> None:
        self.last_pose = pose

    def get_pose(self) -> Pose:
        return self.last_pose

    # --------------------------------------------------------------- turnos

    def add_turn(self, user: str, robot_said: str, action: str,
                 robot_pose: Optional[Pose] = None) -> None:
        ts = datetime.now().strftime('%H:%M:%S')
        pose = robot_pose if robot_pose is not None else self.last_pose
        turn = ConversationTurn(
            timestamp=ts, user=user, robot_said=robot_said,
            action=action, robot_pose=pose,
        )
        self.conversation_history.append(turn)
        if len(self.conversation_history) > self._max_history:
            self.conversation_history.pop(0)

    # -------------------------------------------------------- observaciones

    def add_observations(self, observations: list[dict]) -> int:
        """Acumula observaciones (formato {label, detail}) con dedup por label.

        Devuelve cuántas observaciones nuevas se han añadido (las repetidas
        sólo actualizan el detalle e incrementan `count`).
        """
        ts = datetime.now().strftime('%H:%M:%S')
        new_count = 0
        for obs in observations:
            label = (obs.get('label') or '').strip()
            if not label:
                continue
            detail = (obs.get('detail') or '').strip()
            existing = self._find_observation(label)
            if existing is None:
                self.observations.append(ObservationEntry(
                    timestamp=ts,
                    label=label,
                    detail=detail,
                    count=1,
                    robot_pose=self.last_pose,
                ))
                new_count += 1
            else:
                existing.count += 1
                # Sólo sobrescribimos el detalle si Gemini aporta uno nuevo
                # no vacío y distinto del previo.
                if detail and detail != existing.detail:
                    existing.detail = detail
                existing.timestamp = ts
        return new_count

    def _find_observation(self, label: str) -> Optional[ObservationEntry]:
        norm = label.lower().strip()
        for o in self.observations:
            if o.label.lower().strip() == norm:
                return o
        return None

    def add_investigation_observation(self, step: int, target: str,
                                      observation: str,
                                      image_ref: Optional[str] = None) -> None:
        ts = datetime.now().strftime('%H:%M:%S')
        self.investigation_observations.append(InvestigationObservation(
            step=step, target=target, observation=observation,
            timestamp=ts, robot_pose=self.last_pose, image_ref=image_ref,
        ))

    # ------------------------------------------------------------------ plan

    def set_plan(self, plan: list[dict]) -> None:
        self.current_plan = [
            PlanStep(
                step=int(p.get('step', i + 1)),
                target=p.get('target', ''),
                reason=p.get('reason', ''),
                status='pending',
            )
            for i, p in enumerate(plan)
        ]

    def update_plan_step(self, step: int, status: str) -> None:
        for s in self.current_plan:
            if s.step == step:
                s.status = status
                return

    # ------------------------------------------------------------------ reset

    def reset_investigation(self) -> None:
        """Limpia observaciones y plan pero mantiene historial y modo."""
        self.observations.clear()
        self.investigation_observations.clear()
        self.current_plan.clear()

    def reset_all(self) -> None:
        self.conversation_history.clear()
        self.observations.clear()
        self.investigation_observations.clear()
        self.current_plan.clear()

    # ------------------------------------------------------------- formato

    def format_mode(self) -> str:
        return 'AUTÓNOMO' if self.mode == 'autonomous' else 'GUIADO'

    def format_pose(self) -> str:
        x, y, th = self.last_pose
        return f'x={x:.2f} m, y={y:.2f} m, θ={th:.2f} rad'

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
        for o in self.observations:
            detail = f' — {o.detail}' if o.detail else ''
            seen = f' (visto x{o.count})' if o.count > 1 else ''
            lines.append(f'- {o.label}{detail}{seen}')
        return '\n'.join(lines)

    def format_plan(self) -> str:
        if not self.current_plan:
            return '(sin plan activo)'
        lines = []
        for s in self.current_plan:
            reason = f' — {s.reason}' if s.reason else ''
            lines.append(f'  Paso {s.step} [{s.status}]: {s.target}{reason}')
        return '\n'.join(lines)

    def format_investigation_observations(self) -> str:
        if not self.investigation_observations:
            return '(ninguna)'
        lines = []
        for o in self.investigation_observations:
            lines.append(
                f'  Paso {o.step} ({o.target}) [{o.timestamp}]: {o.observation}'
            )
        return '\n'.join(lines)
