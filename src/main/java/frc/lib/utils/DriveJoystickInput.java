package frc.lib.utils;

public class DriveJoystickInput extends Trio<Double, Double, Double> {
    private boolean slow;
    private boolean fast;
    
    public DriveJoystickInput(Double x, Double y, Double rot, boolean slow, boolean fast) {
        super(x, y, rot);
        this.slow = slow;
        this.fast = fast;
    }

    public double getXJoystick() {
        return super.getFirst();
    }

    public double getYJoystick() {
        return super.getSecond();
    }

    public double getRotationJoystick() {
        return super.getThird();
    }

    public boolean isSlow() {
        return slow;
    }

    public boolean isFast() {
        return fast;
    }

    public static DriveJoystickInput processInput(Double x, Double y, Double rot, boolean slow, boolean fast) {
        return new DriveJoystickInput(x, y, rot, slow, fast);
    }
}
