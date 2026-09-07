#pragma once

#ifndef SOUND_H
#define SOUND_H

#include <Arduino.h>

namespace Sound {

  enum Event {
    TICK,
    TONE,
    CLICK,
    CURIOUS,
    ALERT,
    PURR,
    SAD,
    CONFUSED,
    EXCITED
  };

  void begin();

  // Request a one-shot sound.
  // This does not queue multiple sounds; it stores only the latest pending event.
  void play(Event event);

  // Held mode repeatedly plays the purr sound until disabled.
  void setHeldMode(bool enabled);
  bool getHeldMode();

  // Clears the latest pending one-shot sound, if any.
  // Does not affect held mode.
  void clearPending();

  void setVolume(float volume);
  float getVolume();

  // Direct sound generators.
  // These are blocking if called directly, so prefer Sound::play(...) from robot logic.
  void tone(float freq, float durationSec);
  void glide(float startFreq, float endFreq, float durationSec);
  void vibrato(float baseFreq, float durationSec, float vibratoDepth, float vibratoRate);
  void purr(float baseFreq, float durationSec = .3f, float vibratoDepth = 20.0f, float vibratoRate = 42.0f, float breathRate = 1.0f);
  void chirp(float startFreq = 800.0f, float endFreq = 1200.0f, float durationSec = 0.15, float vibratoDepth = 25.0f, float vibratoRate = 55.0f);
  void squeak();
  void click();
  void clicks(int count, float pauseSec);
  void curious();
  void alert();
  void sad();
  void confused();
  void excited();
  void tick();
  void random();

}

#endif
