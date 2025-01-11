package frc.robot.preferences;

import edu.wpi.first.wpilibj.Preferences;

public class BooleanPref extends Pref<Boolean> {

  BooleanPref(String key, Boolean defaultValue, boolean reset) {
    super(key, defaultValue, reset);
  }

  @Override
  void write(String key, Boolean value) {
    Preferences.setBoolean(key, value);
  }

  @Override
  void init(String key, Boolean value) {
    Preferences.initBoolean(key, value);
  }

  @Override
  Boolean read(String key) {
    return Preferences.getBoolean(key, defaultValue);
  }
}
