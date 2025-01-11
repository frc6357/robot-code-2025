package frc.robot.preferences;

import java.util.HashMap;
import java.util.Map;

/**
 * A wrapper around {@link edu.wpi.first.wpilibj.Preferences} which simplifies
 * their usage.
 * 
 * Example usage:
 * 
 * <pre>
 * Pref<Float> kP = SKPreferences.attach(Constants.Launcher.KpKey, Constants.Launcher.defaultKp)
 *     .onChange(pid::setP);
 * Pref<Float> kI = SKPreferences.attach(Constants.Launcher.KiKey, Constants.Launcher.defaultKi)
 *     .onChange(pid::setI);
 * Pref<Float> kD = SKPreferences.attach(Constants.Launcher.KdKey, Constants.Launcher.defaultKd)
 *     .onChange(pid::setD);
 * 
 * // elsewhere...
 * launcher.setSpeed(pid.calculate(currentSpeed, setpoint));
 * </pre>
 * 
 * In the above example, the PID would adjust its behavior whenever new values
 * were specified in the driver station for
 * any 'Constants.Launcher.KKey' value.
 * 
 * 
 * You may also do something like so:
 * 
 * <pre>
 * Pref<Float> setpoint = SKPreferences.attach(Constants.Launcher.setpointKey, Constants.Launcher.defaultSetpoint)
 * 
 * // elsewhere...
 * launcher.setSpeed(pid.calculate(currentSpeed, setpoint.get()));
 * </pre>
 * 
 * This allows you to read from the preference value directly if you'd rather
 * not use the reactive 'onChange' approach.
 */
public class SKPreferences {

  private static final long UPDATE_INTERVAL_MS = 2000L;
  private long lastUpdate = 0L;

  /**
   * Whether or not to reset values from the code side whenever a new pref is
   * created.
   */
  private boolean resetEnabled = true;

  private Map<String, Pref<? extends Object>> handles = new HashMap<>();

  // Consumers of SKPreferences should not create instances directly.
  private SKPreferences() {
  }

  /** @see {@link SKPreferences#initializeFromCode(boolean)} */
  void enableReset(boolean reset) {
    this.resetEnabled = reset;
  }

  /**
   * Used to implement {@link #refreshIfNeeded()} and {@link #forceRefresh()}.
   */
  private void poll(final boolean force) {
    final long now = System.currentTimeMillis();
    if (!force && (now - lastUpdate) < UPDATE_INTERVAL_MS)
      return;

    for (final String key : handles.keySet()) {
      handles.get(key).poll();
    }

    lastUpdate = now;
  }

  /** @see {@link #attach(String, Object)} */
  @SuppressWarnings("unchecked")
  private synchronized <T> Pref<T> attachPref(final String key, final T defaultValue) {
    final var proposedHandle = (Pref<T>) Pref.create(key, defaultValue, this.resetEnabled);
    if (handles.containsKey(key)) {
      // Make sure the provided handle doesn't conflict with the existing one.
      final var currentHandle = (Pref<T>) handles.get(key);
      if (proposedHandle.getClass() != currentHandle.getClass()) {
        // Trying to overwrite an existing handle with a different handle type.
        throw new RuntimeException(
            String.format(
                "Already registered '%s' for key '%s', cannot register different handle type '%s'.",
                currentHandle.getClass().getCanonicalName(),
                key,
                proposedHandle.getClass().getCanonicalName()));
      } else if (currentHandle.defaultValue != proposedHandle.defaultValue) {
        System.err.println(
            String.format(
                "Ignoring default value %s for key %s, a different default '%s' was set elsewhere.",
                String.valueOf(defaultValue),
                key,
                String.valueOf(currentHandle.defaultValue)));
      }
    } else {
      handles.put(key, proposedHandle);
    }

    return (Pref<T>) handles.get(key);
  }

  private synchronized void detachAll() {
    handles.values().forEach((pref) -> pref.clearListeners());
    handles.clear();
  }

  // Public API is static for simplicity of use.
  // Note for students: Typically, we would use the instance() method outside of
  // this class, as opposed to using
  // static methods which wrap calls to instance(), e.g.
  // `RxPreferences.instance().attach(...)`.

  private static SKPreferences singleton;

  // Utility to make sure the internal singleton is initialized correctly.
  private static synchronized SKPreferences instance() {
    return singleton == null ? (singleton = new SKPreferences()) : singleton;
  }

  /**
   * Decides whether or not preferences treat the pre-existing preferences or the
   * robot as the source of truth.
   * 
   * This method should only be called ONCE before any calls to
   * {@link #attach(String, Object)}.
   * 
   * @param useCode Set to true if the robot should always be initialized with
   *                code-side values. Set to false if you
   *                would like to persist preferences across robot power-cycles or
   *                roborio reboots. The default behavior is to
   *                always reset from code (e.g. useCode=true).
   */
  public static void initializeFromCode(boolean useCode) {
    instance().enableReset(useCode);
  }

  /**
   * This method will read all of the latest preference data from network tables,
   * and is designed to be called by a
   * command that runs periodically as long as the robot is enabled.
   * 
   * It will not trigger a full refresh each invocation, but only if a certain
   * amount of time has elapsed since the
   * last refresh. This is intended to minimize the impact of reading many values
   * from network tables all at once.
   */
  public static void refreshIfNeeded() {
    instance().poll(false);
  }

  /**
   * Immediately reads the latest values from the driver station.
   */
  public static void forceRefresh() {
    instance().poll(true);
  }

  /**
   * Creates a preference which will be periodically updated by the SKPreferences
   * object.
   * 
   * @param <T>          The value-type you want to observe. Supports: String,
   *                     Int, Long, Float, Double, Boolean
   * @param key          The key to observe. Multiple calls will return the same
   *                     {@link Pref}, but only the first one will set
   *                     the default value for the preference.
   * @param defaultValue The value to initialize the preference if it has not
   *                     already been configured.
   * @return A {@link Pref} which will periodically update based on preferences
   *         set on driver station.
   * 
   * @throws RuntimeException If trying to register handles of different types to
   *                          the same key.
   */
  public static <T> Pref<T> attach(String key, T defaultValue) {
    return instance().attachPref(key, defaultValue);
  }

  /**
   * Stops all previously attached preferences from receiving updates.
   * 
   * Only newly attached preferences will receive updates after this is called.
   * 
   * This is mainly intended for unit testing purposes.
   */
  public static void disableAllPrefs() {
    instance().detachAll();
  }
}
