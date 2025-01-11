package frc.robot.preferences;

import edu.wpi.first.wpilibj.Preferences;

final class StringPref extends Pref<String> {

  StringPref(String key, String defaultValue, boolean reset) {
    super(key, defaultValue, reset);
  }

  @Override
  void write(final String key, final String value) {
    Preferences.setString(key, value);
  }

  @Override
  String read(final String key) {
    return Preferences.getString(key, defaultValue);
  }

  @Override
  void init(String key, String value) {
    Preferences.initString(key, value);
  }
}
