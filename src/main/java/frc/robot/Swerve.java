package frc.robot;

//Packages used for the mechanisms/motors for the swerve drivetrain itself
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

//Packages used for positioning, kinematics, odometry
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    public Swerve(SwerveConfig config) {
        //Creates a Swerve Drivetrain using Phoenix6's SwerveDrivetrain class, passing the
        //properties of the swerve drive itself from SwerveConfig into the constructor.
        super(TalonFX::new, TalonFX::new, CANcoder::new,
        config.getDrivetrainConstants(),
        config.getModules());
    }
    
}
