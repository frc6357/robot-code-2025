package frc.robot.preferences;

import edu.wpi.first.wpilibj.Preferences;

final class DoublePref extends Pref<Double> {

  DoublePref(String key, Double defaultValue, boolean reset) {
    super(key, defaultValue, reset);
  }

  @Override
  void write(final String key, final Double value) {
    Preferences.setDouble(key, value);
  }

  @Override
  Double read(final String key) {
    return Preferences.getDouble(key, defaultValue);
  }

  @Override
  void init(String key, Double value) {
    Preferences.initDouble(key, value);
  }
}
