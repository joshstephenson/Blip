# Blip
 
Blip is a cat-ear inspired desktop robot that perceives its environment and responds with genuine emotional expression. It detects sound and movement, generates unique internal emotional states, and communicates those states through animated ear shapes, expressive eye forms, and procedurally generated sounds — no two responses are quite the same.
 
## What It Does
 
Blip sits on a desk and pays attention. When something happens nearby — a sound, a presence, a change in the room — it notices. Its ears swivel, its eyes shift, and it makes a sound. The combination of ear position, eye expression, and audio is driven by an internal emotional model, so Blip's reactions feel considered rather than mechanical.
 
## Hardware
 
| Component | Part |
|---|---|
| Microcontroller | ESP32 DevKit |
| Distance / presence sensor | VL53L1X (Time-of-Flight) |
| Eye / ear displays | 1.28" round TFT screens (×2) |
| Ear actuators | MG90S micro servos (×2) |
| Microphone | MAX9814 (auto gain electret) |
| Audio amplifier | MAX98357A (I2S, 3W) |
| Speaker | 27mm 4Ω 3W |
| Power | USB power module |
 
## Personality
 
Blip's emotional states are generated, not scripted. The VL53L1X provides distance and presence data; the MAX9814 captures ambient sound level. These inputs feed an emotional model running on the ESP32 that produces a continuous state — somewhere along axes like curious/calm, alert/relaxed, pleased/uncertain. That state drives:
 
- **Ear shape** via the MG90S servos (position and speed of movement)
- **Eye expression** via the round TFT displays (pupil size, shape, animation)
- **Sound** via the MAX98357A and speaker (tone, rhythm, duration)
The goal is that Blip feels alive in a small, unobtrusive way — reactive without being annoying, expressive without being overdone.
 
## Physical Design
 
Blip's head is a 3D-printed enclosure designed around an internal chassis. The VL53L1X faces forward, the servos drive the ear/eye assemblies from the front face with the TFT screens mounted on the servo horn axes, the microphone and speaker surface through the top, and the ESP32 and USB power module are accessible from the rear without disassembly.

