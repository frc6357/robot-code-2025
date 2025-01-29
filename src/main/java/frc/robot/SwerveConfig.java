package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConfig {
    private static final double kSimLoopPeriod = 0.005;
    //TODO: Update these values with final robot dimensions
    private double robotWidth = Units.inchesToMeters(27);
    private double robotLength = Units.inchesToMeters(27);

    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
}
