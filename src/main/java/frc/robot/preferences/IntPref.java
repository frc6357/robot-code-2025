package frc.robot.preferences;

import edu.wpi.first.wpilibj.Preferences;

final class IntPref extends Pref<Integer> {

  IntPref(String key, Integer defaultValue, boolean reset) {
    super(key, defaultValue, reset);
  }

  @Override
  void write(final String key, final Integer value) {
    Preferences.setInt(key, value);
  }

  @Override
  Integer read(final String key) {
    return Preferences.getInt(key, defaultValue);
  }

  @Override
  void init(String key, Integer value) {
    Preferences.initInt(key, value);
  }
}