package frc.robot.preferences;

import edu.wpi.first.wpilibj.Preferences;

final class LongPref extends Pref<Long> {

  LongPref(String key, Long defaultValue, boolean reset) {
    super(key, defaultValue, reset);
  }

  @Override
  void write(final String key, final Long value) {
    Preferences.setLong(key, value);
  }

  @Override
  Long read(final String key) {
    return Preferences.getLong(key, defaultValue);
  }

  @Override
  void init(String key, Long value) {
    Preferences.initLong(key, value);
  }
}
