## Fase 0 - Configuración del entorno

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

## Fase 1 - Voz

Implementación de lo relacionado con la voz.

- `speech_node` se encarga de gestionar cuando tiene que hablar el robot
- `tts_client` se encarga de hablar (en principio se pensó por usar ElevenLabs pero la prueba gratuita no incluye muchos modelos y se optó por gTTS)
- `audio_in_node` se encarga de transcribir lo que dice la persona usando Whisper, para posteriormente pasárselo al modelo de Gemini. Se graba en ventanas de 10 segundos y filtrando ruido

Notas anti-eco (para que Whisper no transcriba lo que dice el robot):
- `speech_node` publica `/robot_speaking` mientras reproduce el TTS.
- `audio_in_node` ignora audio durante `/robot_speaking` y un “tail” extra, y además filtra transcripciones muy parecidas a lo último que publicó `/robot_speech`.
- Ajustes opcionales por variables de entorno:
  - `AUDIO_POST_SPEECH_TAIL_S` (default `4.0`)
  - `ROBOT_SPEAKING_END_TAIL_S` (default `0.4`)
  - `AUDIO_ROBOT_ECHO_WINDOW_S` (default `12.0`)
  - `AUDIO_ROBOT_ECHO_MIN_SIM` (default `0.86`)

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

---

## Fase 2 - Cerebro mínimo

Implementación de `brain_node` con llamada a Gemini.

- `brain_node` se suscribe a `/user_speech` y `/camera/image_raw`
- Cuando recibe texto del usuario, captura el último frame de la cámara y llama a Gemini
- Parsea la respuesta JSON y la guarda de forma incremental en `/workspace/interactions.jsonl`
- Publica `True` en `/brain_ready` al terminar para que `audio_in_node` vuelva a escuchar
- `gemini_client` se autentica con `credenciales.json` vía Vertex AI

### Prueba: probar `brain_node` con conversación de texto

Terminal 1:
```bash
ros2 run embodied_agent brain_node
```

Terminal 2 (simular voz del usuario):
```bash
ros2 topic pub --once /user_speech std_msgs/msg/String "data: '¿qué ves a tu alrededor?'"
```

Terminal 3 (ver que brain_ready se publica):
```bash
ros2 topic echo /brain_ready
```

Las interacciones se guardan en `/workspace/interactions.jsonl` (una línea JSON por interacción).


## Fase 3 - Ajustar a detective
```bash
ros2 launch embodied_agent embodied_agent_sim_tb4_detective.launch.py
```

Para eliminar el dock:

```bash
ign service -s /world/warehouse/remove --reqtype ignition.msgs.Entity \
  --reptype ignition.msgs.Boolean --timeout 2000 \
  --req 'name: "turtlebot4/standard_dock", type: MODEL'
```
