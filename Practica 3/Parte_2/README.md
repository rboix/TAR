## Fase 0 — Configuración del entorno

Hay que configurar las claves en el .env para que funcione todo

Dentro de la misma carpeta:

```bash
docker build --build-arg USERNAME=$USER --build-arg USER_UID=$(id -u) -t embodied_agent:latest .
```

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch embodied_agent embodied_agent.launch.py  # Prueba inicial para ver que todo funciona
```

---

## Fase 1 — Voz

Implementación de lo relacionado con la voz.

- `speech_node` se encarga de gestionar cuando tiene que hablar el robot
- `tts_client` se encarga de hablar (en principio se pensó por usar ElevenLabs pero la prueba gratuita no incluye muchos modelos y se optó por gTTS)
- `audio_in_node` se encarga de transcribir lo que dice la persona usando Whisper, para posteriormente pasárselo al modelo de Gemini. Se graba en ventanas de 10 segundos y filtrando ruido

### Pruebas

**Prueba 1: probar `speech_node` para ver si funciona la "voz" del robot**

Terminal 1:
```bash
ros2 run embodied_agent speech_node
```

Terminal 2:
```bash
ros2 topic pub --once /robot_speech std_msgs/String "data: 'Hola, soy tu robot'"
```

**Prueba 2: probar `audio_in_node` para ver si funciona la transcripción con Whisper**

Terminal 1:
```bash
ros2 run embodied_agent audio_in_node
```

Terminal 2:
```bash
ros2 topic echo /user_speech
```
