package frc.robot.preferences;

import edu.wpi.first.wpilibj.Preferences;

final class FloatPref extends Pref<Float> {

  FloatPref(String key, Float defaultValue, boolean reset) {
    super(key, defaultValue, reset);
  }

  @Override
  void write(final String key, final Float value) {
    Preferences.setFloat(key, value);
  }

  @Override
  Float read(final String key) {
    return Preferences.getFloat(key, defaultValue);
  }

  @Override
  void init(String key, Float value) {
    Preferences.initFloat(key, value);
  }
}
