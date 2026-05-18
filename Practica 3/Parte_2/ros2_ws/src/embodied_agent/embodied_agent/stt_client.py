"""Speech-to-text con Whisper local."""
import os
import tempfile
import numpy as np


_model = None

DEFAULT_MODEL_SIZE = os.environ.get('WHISPER_MODEL_SIZE', 'small')
WHISPER_PROMPT = (
    'Robot detective en un laboratorio. Investiga la escena. '
    'Acércate, inspecciona, panorámica, gira, vuelve. '
    'Mochila, botella, papeles, silla, llaves, pistas, derrame, huida.'
)


def load_model(size: str = DEFAULT_MODEL_SIZE) -> None:
    global _model
    import whisper
    _model = whisper.load_model(size)


def transcribe(audio_data: np.ndarray, sample_rate: int = 16000) -> str:
    """Transcribe audio numpy array a texto."""
    if _model is None:
        load_model()
    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
        import soundfile as sf
        sf.write(f.name, audio_data, sample_rate)
        result = _model.transcribe(
            f.name, language='es', initial_prompt=WHISPER_PROMPT)
    return result.get('text', '').strip()
