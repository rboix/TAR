"""Text-to-speech: ElevenLabs con fallback a gTTS."""
import os
import io
import tempfile


def speak(text: str) -> None:
    """Sintetiza y reproduce texto por audio."""
    if _try_elevenlabs(text):
        return
    _speak_gtts(text)


def _try_elevenlabs(text: str) -> bool:
    api_key = os.environ.get('ELEVENLABS_API_KEY', '')
    voice_id = os.environ.get('ELEVENLABS_VOICE_ID', '')
    if not api_key or not voice_id:
        return False
    try:
        from elevenlabs.client import ElevenLabs
        import pygame
        client = ElevenLabs(api_key=api_key)
        audio = client.text_to_speech.convert(
            voice_id=voice_id,
            text=text,
            model_id='eleven_multilingual_v2',
        )
        audio_bytes = b''.join(audio)
        pygame.mixer.init()
        pygame.mixer.music.load(io.BytesIO(audio_bytes))
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.wait(100)
        return True
    except Exception:
        return False


def _speak_gtts(text: str) -> None:
    try:
        from gtts import gTTS
        import pygame
        tts = gTTS(text=text, lang='es')
        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            tts.save(f.name)
            pygame.mixer.init()
            pygame.mixer.music.load(f.name)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.wait(100)
    except Exception as e:
        print(f'[tts] fallback gTTS también falló: {e}')
