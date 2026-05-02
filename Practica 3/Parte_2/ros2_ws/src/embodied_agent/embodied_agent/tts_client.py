"""Text-to-speech: ElevenLabs → gTTS → espeak (fallbacks en orden).
Reproduce audio con ffplay (evita problemas de ALSA/pygame).
"""
import io
import os
import subprocess
import tempfile


def speak(text: str) -> None:
    if _try_elevenlabs(text):
        return
    if _try_gtts(text):
        return
    _speak_espeak(text)


# 
def _play_mp3_bytes(audio_bytes: bytes) -> None:
    """Convierte MP3 a WAV con ffmpeg y reproduce con paplay (PulseAudio)."""
    mp3_path, wav_path = None, None
    try:
        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            f.write(audio_bytes)
            mp3_path = f.name
        wav_path = mp3_path.replace('.mp3', '.wav')

        # Reproduce el MP3 convertido a WAV
        # Usamos paplay para evitar problemas de ALSA/pygame.
        subprocess.run(
            ['ffmpeg', '-y', '-loglevel', 'quiet', '-i', mp3_path, wav_path],
            check=True, capture_output=True,
        )
        subprocess.run(['paplay', wav_path], check=True, capture_output=True)
    finally:
        for p in [mp3_path, wav_path]:
            if p:
                try:
                    os.unlink(p)
                except OSError:
                    pass


# Llama a la API de ElevenLabs para sintetizar texto
# Si falla, devuelve False para probar gTTS.
def _try_elevenlabs(text: str) -> bool:
    api_key = os.environ.get('ELEVENLABS_API_KEY', '')
    voice_id = os.environ.get('ELEVENLABS_VOICE_ID', '')
    if not api_key or not voice_id:
        return False
    try:
        from elevenlabs.client import ElevenLabs
        client = ElevenLabs(api_key=api_key)
        chunks = client.text_to_speech.convert(
            voice_id=voice_id,
            text=text,
            model_id='eleven_multilingual_v2',
            output_format='mp3_44100_128',
        )
        _play_mp3_bytes(b''.join(chunks))
        return True
    except Exception as e:
        print(f'[tts] ElevenLabs falló ({e}), probando gTTS')
        return False


# Usa gTTS para sintetizar texto. Si falla, devuelve False para probar espeak.
def _try_gtts(text: str) -> bool:
    try:
        from gtts import gTTS
        tts = gTTS(text=text, lang='es')
        buf = io.BytesIO()
        tts.write_to_fp(buf)
        _play_mp3_bytes(buf.getvalue())
        return True
    except Exception as e:
        print(f'[tts] gTTS falló ({e}), probando espeak')
        return False


def _speak_espeak(text: str) -> None:
    try:
        subprocess.run(
            ['espeak', '-v', 'es', '-s', '150', text],
            check=True, capture_output=True,
        )
    except Exception as e:
        print(f'[tts] espeak también falló: {e}')
