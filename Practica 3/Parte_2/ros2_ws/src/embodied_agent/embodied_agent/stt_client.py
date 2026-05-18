"""Speech-to-text.

Backend por defecto: **OpenAI Whisper API** (`whisper-1`). Es muchísimo
más preciso para español conversacional que cualquier modelo local que
quepa en CPU, y la diferencia se nota especialmente en frases cortas
("acércate", "gira", etc.).

Fallback automático a Whisper local si:
  - `STT_BACKEND=local` está forzado por variable de entorno, o
  - falta `OPENAI_API_KEY`, o
  - falla el import del SDK de OpenAI.

Variables de entorno:
  STT_BACKEND        = "openai" | "local"          (default: openai)
  OPENAI_API_KEY     = clave de la API             (requerido para openai)
  OPENAI_STT_MODEL   = "whisper-1" | "gpt-4o-mini-transcribe"
                       (default: whisper-1, más rápido y barato)
  WHISPER_MODEL_SIZE = "tiny" | "base" | "small" | "medium" | "large"
                       (solo para backend local; default: small)
"""
import io
import os
import tempfile
import wave
from typing import Optional

import numpy as np


# ============================================================ configuración

# IMPORTANTE: `os.environ.get(k, default)` solo devuelve `default` si la
# variable NO EXISTE. Si run.sh exporta `-e STT_BACKEND="${STT_BACKEND}"`
# y la variable está vacía en el host, en el container la variable SÍ
# existe pero vale "". Por eso usamos `(env or default)` — así una
# variable vacía también cae al default.
STT_BACKEND = (os.environ.get('STT_BACKEND') or 'openai').lower()
OPENAI_STT_MODEL = os.environ.get('OPENAI_STT_MODEL') or 'whisper-1'
WHISPER_MODEL_SIZE = os.environ.get('WHISPER_MODEL_SIZE') or 'small'

# Prompt orientativo (mejora transcripción de jerga del proyecto).
STT_PROMPT = (
    'Robot detective en un laboratorio. Investiga la escena. '
    'Acércate, inspecciona, panorámica, gira, vuelve. '
    'Mochila, botella, papeles, silla, llaves, pistas, derrame, huida.'
)


# ================================================================ estado

_local_model = None
_openai_client = None
_active_backend: Optional[str] = None


# =========================================================== helpers WAV

def _audio_to_wav_bytes(audio: np.ndarray, sample_rate: int) -> bytes:
    """Convierte float32 [-1,1] a un WAV mono int16 en memoria."""
    audio_int16 = np.clip(audio * 32767.0, -32768, 32767).astype(np.int16)
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(audio_int16.tobytes())
    return buf.getvalue()


# ========================================================== inicialización

def initialize(logger=None) -> str:
    """Carga el cliente STT. Devuelve el nombre del backend que quedó activo.

    `logger` es opcional (un `rclpy.logging.RcutilsLogger` típicamente).
    Si no se pasa, los mensajes van a stdout.
    """
    global _active_backend, _local_model, _openai_client

    def _log(level: str, msg: str):
        if logger is None:
            print(f'[stt_client] {level.upper()}: {msg}')
        else:
            getattr(logger, level)(msg)

    _log('info',
         f'STT config: STT_BACKEND="{STT_BACKEND}" '
         f'OPENAI_STT_MODEL="{OPENAI_STT_MODEL}" '
         f'WHISPER_MODEL_SIZE="{WHISPER_MODEL_SIZE}" '
         f'OPENAI_API_KEY={"set" if os.environ.get("OPENAI_API_KEY") else "UNSET"}')

    if STT_BACKEND == 'openai':
        try:
            from openai import OpenAI
            api_key = os.environ.get('OPENAI_API_KEY')
            if not api_key:
                raise RuntimeError('OPENAI_API_KEY no está definida')
            _openai_client = OpenAI(api_key=api_key)
            _active_backend = 'openai'
            _log('info', f'STT backend: OpenAI API ({OPENAI_STT_MODEL})')
            return _active_backend
        except Exception as e:
            _log('warn',
                 f'STT backend OpenAI no disponible ({e}); '
                 f'cayendo a Whisper local "{WHISPER_MODEL_SIZE}"')

    # Backend local: explícito o fallback.
    os.environ['NUMBA_DISABLE_JIT'] = '1'
    os.environ.pop('COVERAGE_PROCESS_START', None)
    import whisper
    _local_model = whisper.load_model(WHISPER_MODEL_SIZE)
    _active_backend = 'local'
    _log('info', f'STT backend: Whisper local ({WHISPER_MODEL_SIZE})')
    return _active_backend


def active_backend() -> str:
    return _active_backend or 'uninitialized'


# ================================================================== API

def transcribe(audio: np.ndarray, sample_rate: int = 16000
               ) -> tuple[str, float]:
    """Transcribe audio mono float32 a texto.

    Devuelve `(text, no_speech_prob)`. `no_speech_prob` es 1.0 cuando el
    backend no expone esa señal (caso de la API de OpenAI) — el caller
    debería usar otra heurística para decidir si hay habla
    (típicamente RMS + texto no vacío).
    """
    if _active_backend is None:
        raise RuntimeError(
            'stt_client.initialize() no se ha llamado antes de transcribe()')
    if _active_backend == 'openai':
        return _transcribe_openai(audio, sample_rate)
    return _transcribe_local(audio, sample_rate)


def _transcribe_openai(audio: np.ndarray,
                       sample_rate: int) -> tuple[str, float]:
    wav_bytes = _audio_to_wav_bytes(audio, sample_rate)
    # El SDK acepta una tupla (filename, bytes, mime) para archivos en RAM.
    response = _openai_client.audio.transcriptions.create(
        model=OPENAI_STT_MODEL,
        file=('chunk.wav', wav_bytes, 'audio/wav'),
        language='es',
        prompt=STT_PROMPT,
    )
    return (response.text or '').strip(), 0.0


def _transcribe_local(audio: np.ndarray,
                      sample_rate: int) -> tuple[str, float]:
    # Whisper local trabaja con archivos: escribimos un WAV temporal.
    tmp_path = None
    try:
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            tmp_path = f.name
            f.write(_audio_to_wav_bytes(audio, sample_rate))
        result = _local_model.transcribe(
            tmp_path, language='es', initial_prompt=STT_PROMPT)
        segs = result.get('segments', [])
        no_speech = float(segs[0].get('no_speech_prob', 1.0)) if segs else 1.0
        return (result.get('text', '') or '').strip(), no_speech
    finally:
        if tmp_path:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
