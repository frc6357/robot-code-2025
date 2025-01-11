package frc.robot.preferences;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * A handle to a driver station preference.
 * 
 * Provides methods to fetch the latest value and then read it.
 */
public abstract class Pref<T> {

  // Probably won't have more than one listener, so lowering default capacity from 10 to 2.
  private List<Consumer<T>> listeners = new ArrayList<>(2);
  private final String key;
  public final T defaultValue;
  private T value;

  Pref(final String key, final T defaultValue, final boolean reset) {
    this.key = key;
    this.defaultValue = defaultValue;

    // Initialize to defaultValue if it doesn't already exist.
    if (reset) {
      write(key, defaultValue);
    } else {
      init(key, defaultValue);
    }

    // Read back the saved preference data.
    this.value = read(key);
  }

  /**
   * Gets the most recently polled value. You must call {@link #poll()} to update
   * the value.
   * 
   * This method provides no thread safety. If a value is updated on driver
   * station at the same time that it is
   * updated here, the value may not be updated until the next call to
   * {@link #poll()}.
   * 
   * @return The latest value polled from driver station.
   */
  public T get() {
    return this.value;
  }

  /**
   * Fetches the latest value from driver station.
   */
  void poll() {
    final var previous = this.value;
    this.value = read(key);

    // Use '.equals' since some types (e.g. string) can't be compared with '!=' or '=='.
    if (!this.value.equals(previous)) {
      // Send the updated info to subscribers.
      listeners.forEach((listener) -> listener.accept(this.value));
    }
  }

  public Pref<T> onChange(Consumer<T> consumer) {
    this.listeners.add(consumer);
    return this;
  }

  void clearListeners() {
    this.listeners.clear();
  }

  abstract void write(final String key, final T value);

  abstract void init(final String key, final T value);

  abstract T read(final String key);

  public static Pref<? extends Object> create(final String key, final Object defaultValue, final boolean reset) {
    if (defaultValue instanceof String) {
      return new StringPref(key, (String) defaultValue, reset);
    } else if (defaultValue instanceof Integer) {
      return new IntPref(key, (Integer) defaultValue, reset);
    } else if (defaultValue instanceof Long) {
      return new LongPref(key, (Long) defaultValue, reset);
    } else if (defaultValue instanceof Float) {
      return new FloatPref(key, (Float) defaultValue, reset);
    } else if (defaultValue instanceof Double) {
      return new DoublePref(key, (Double) defaultValue, reset);
    } else if (defaultValue instanceof Boolean) {
      return new BooleanPref(key, (Boolean) defaultValue, reset);
    } else {
      throw new RuntimeException(
          String.format("Unsupported pref type '%s' for key '%s'.", defaultValue.getClass().getCanonicalName(), key));
    }
  }
}
