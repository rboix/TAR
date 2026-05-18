"""Esquemas pydantic para validar las respuestas JSON de Gemini.

Corresponde al esquema general de la sección 11 del CLAUDE.md.

Reparto de capas (sección 8 de CLAUDE.md):
- Gemini NO calcula coordenadas absolutas ni velocidades.
- Para `navigate`/`inspect` debe devolver `image_bbox` (bbox del objeto en la
  imagen, normalizada [0,1]). Nuestro código convierte pixel + profundidad →
  3D → map → goal de Nav2.
- `distance` (opcional) en navigate/inspect = "para a X m del objeto"
  (stop distance). Si no viene, usamos 0.5 m por defecto.
- `degrees` en `rotate`/`panoramic` = giro en grados (positivo = antihorario).

Campos NO incluidos en este esquema (deliberadamente):
- `angle`: Gemini no es fiable estimando ángulos. Si hay que orientar al
  robot, el goal de Nav2 lo decide a partir de la pose objetivo.
"""
from typing import Literal, Optional

from pydantic import BaseModel, Field, field_validator, model_validator


VALID_ACTIONS = (
    'none',
    'navigate',
    'rotate',
    'panoramic',
    'inspect',
    'investigate',
    'ask_user',
)


class ActionParams(BaseModel):
    """Parámetros de la acción. Cada campo aplica sólo a ciertas acciones.

    - `target`: descripción del objetivo (todas las acciones de movimiento).
    - `image_bbox`: [x_min, y_min, x_max, y_max] normalizado a [0,1] sobre
      la imagen RGB actual (sólo `navigate` / `inspect`).
    - `distance`: distancia de parada en metros antes del objeto (sólo
      `navigate` / `inspect`). Default 0.5 m si Gemini no la rellena.
    - `degrees`: grados a girar (sólo `rotate` / `panoramic`).
    """
    target: Optional[str] = None
    image_bbox: Optional[list[float]] = None
    distance: Optional[float] = None
    degrees: Optional[float] = None

    model_config = {'extra': 'ignore'}

    @model_validator(mode='before')
    @classmethod
    def _renormalize_bbox(cls, values):
        """Repara `image_bbox` ANTES de validar campo a campo.

        Gemini 2.5 Flash devuelve a veces el bbox en escala 0–1000, o en
        píxeles, o incluso mezclado ([0.63, 420, 0.72, 590]). Heurística
        simple y robusta: cualquier valor > 1 se divide entre 1000.
        Esto cubre todos los casos observados:
          - normalizado puro [0.4, 0.6, 0.6, 0.9]  → sin cambios
          - escala 0–1000   [400, 600, 600, 900]   → [0.4, 0.6, 0.6, 0.9]
          - híbrido         [0.63, 420, 0.72, 590] → [0.63, 0.42, 0.72, 0.59]
        Si después de normalizar sigue siendo malformado, el field_validator
        lo rechazará.
        """
        if not isinstance(values, dict):
            return values
        bbox = values.get('image_bbox')
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return values
        try:
            vals = [float(v) for v in bbox]
        except (TypeError, ValueError):
            return values
        if any(v > 1.0 for v in vals):
            vals = [v / 1000.0 if v > 1.0 else v for v in vals]
            values['image_bbox'] = vals
        return values

    @field_validator('image_bbox')
    @classmethod
    def bbox_well_formed(cls, v):
        if v is None:
            return v
        if len(v) != 4:
            raise ValueError('image_bbox debe tener 4 valores [x0,y0,x1,y1]')
        for x in v:
            if not 0.0 <= float(x) <= 1.0:
                raise ValueError(
                    'image_bbox debe estar normalizado entre 0 y 1')
        x0, y0, x1, y1 = v
        if x1 <= x0 or y1 <= y0:
            raise ValueError(
                'image_bbox inválido: se requiere x1>x0 e y1>y0')
        return [float(x) for x in v]


class Observation(BaseModel):
    label: str
    detail: Optional[str] = None

    model_config = {'extra': 'ignore'}


class GeminiResponse(BaseModel):
    """Esquema general de respuesta de Gemini (turno conversacional)."""

    action: Literal[
        'none', 'navigate', 'rotate', 'panoramic',
        'inspect', 'investigate', 'ask_user',
    ]
    action_params: ActionParams = Field(default_factory=ActionParams)
    speech: str
    reasoning: Optional[str] = None
    observations: list[Observation] = Field(default_factory=list)

    model_config = {'extra': 'ignore'}

    @field_validator('speech')
    @classmethod
    def speech_not_empty(cls, v: str) -> str:
        if not v or not v.strip():
            raise ValueError('"speech" no puede estar vacío')
        return v.strip()
