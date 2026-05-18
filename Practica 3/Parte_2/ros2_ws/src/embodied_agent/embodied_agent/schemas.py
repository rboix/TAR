"""Esquemas pydantic para validar las respuestas JSON de Gemini.

Corresponde al esquema general de la sección 11 del CLAUDE.md.
Los esquemas de plan de investigación e hipótesis final se añadirán
en la fase 7.
"""
from typing import Literal, Optional

from pydantic import BaseModel, Field, field_validator


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
    target: Optional[str] = None
    distance: Optional[float] = None
    angle: Optional[float] = None
    degrees: Optional[float] = None

    model_config = {'extra': 'ignore'}


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
