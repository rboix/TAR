"""Wrapper para la API de Gemini (Vertex AI via credenciales.json)."""
import json
import os
from pathlib import Path

from google import genai
from google.genai import types


CREDENTIALS_PATH = Path(__file__).parents[4] / 'credenciales.json'
GEMINI_MODEL = 'gemini-3.1-flash-lite-preview'

_client: genai.Client | None = None


def get_client() -> genai.Client:
    global _client
    if _client is not None:
        return _client

    if not CREDENTIALS_PATH.exists():
        raise FileNotFoundError(
            f'No se encontró credenciales.json en {CREDENTIALS_PATH}. '
            'Colócalo en Practica 3/Parte_2/credenciales.json'
        )

    os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = str(CREDENTIALS_PATH)
    project = json.loads(CREDENTIALS_PATH.read_text()).get('project_id', '')
    _client = genai.Client(vertexai=True, project=project)
    return _client


def call_gemini(system_prompt: str, user_text: str, image_bytes: bytes | None = None) -> dict:
    client = get_client()
    parts: list = [types.Part.from_text(text=user_text)]
    if image_bytes:
        parts.insert(0, types.Part.from_bytes(data=image_bytes, mime_type='image/jpeg'))

    config = types.GenerateContentConfig(
        system_instruction=system_prompt,
        response_mime_type='application/json',
        temperature=0.3,
    )
    response = client.models.generate_content(
        model=GEMINI_MODEL,
        contents=parts,
        config=config,
    )
    return json.loads(response.text)
