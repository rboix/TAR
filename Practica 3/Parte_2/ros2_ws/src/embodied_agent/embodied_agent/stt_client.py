"""Speech-to-text con Whisper local."""
import tempfile
import numpy as np


_model = None


def load_model(size: str = 'base') -> None:
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
        result = _model.transcribe(f.name, language='es')
    return result.get('text', '').strip()
