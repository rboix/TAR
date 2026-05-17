"""Wrapper para la API de Gemini (Vertex AI via credenciales.json)."""
import json
import os
from pathlib import Path

from google import genai
from google.genai import types


GEMINI_MODEL = 'publishers/google/models/gemini-3.1-flash-lite'

_client: genai.Client | None = None


def _find_credentials() -> Path:
    env_path = os.environ.get('GOOGLE_APPLICATION_CREDENTIALS')
    candidates = []
    if env_path:
        candidates.append(Path(env_path))
    candidates += [
        Path('/workspace/credenciales.json'),
        Path(__file__).parents[4] / 'credenciales.json',
        Path.cwd() / 'credenciales.json',
    ]
    for p in candidates:
        if p.exists():
            return p
    raise FileNotFoundError(
        'No se encontró credenciales.json. Buscado en: '
        + ', '.join(str(c) for c in candidates)
        + '. Colócalo en Practica 3/Parte_2/credenciales.json'
    )


def get_client() -> genai.Client:
    global _client
    if _client is not None:
        return _client

    cred_path = _find_credentials()
    os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = str(cred_path)
    project = json.loads(cred_path.read_text()).get('project_id', '')
    _client = genai.Client(vertexai=True, project=project, location='global')
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
