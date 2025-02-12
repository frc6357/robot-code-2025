package frc.robot.subsystems.swerve;

// Packages used for the mechanisms/motors for the swerve drivetrain itself
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
// import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;

// Packages used for Pathplanner
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// Packages used for positioning, kinematics, odometry
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

// Packages used for logging, communications, overall functionality
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;
import frc.robot.utils.Field;
import frc.robot.utils.SK25AutoBuilder;
import frc.robot.utils.Util;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Konstants.AutoConstants.*;
import static frc.robot.Konstants.SwerveConstants.kDeadband;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SK25Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements NTSendable, Subsystem {
    private SwerveConfig config;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private RotationController rotationController;
    private Pigeon2 m_pigeon2;

    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds AutoRequest = new SwerveRequest.ApplyRobotSpeeds();
    
    //Creates publishers for logging
    StructArrayPublisher<SwerveModuleState> currentPublisher = 
    NetworkTableInstance.getDefault().
    getStructArrayTopic("CurrentSwerveStates", SwerveModuleState.struct).publish();

    StructArrayPublisher<SwerveModuleState> targetPublisher = 
    NetworkTableInstance.getDefault().
    getStructArrayTopic("TargetSwerveStates", SwerveModuleState.struct).publish();

    StructPublisher<Rotation2d> odomPublisher = NetworkTableInstance.getDefault().
    getStructTopic("Rotation", Rotation2d.struct).publish();

    /**
     * Constructs a new Swerve drive subsystem.
     *
     * @param config The configuration object containing constants for 
     * the drivetrain and the module configurations.
     */
    public SK25Swerve(SwerveConfig config) {
        // Creates a Swerve Drivetrain using Phoenix6's SwerveDrivetrain class, passing the
        // properties of the swerve drive itself from SwerveConfig into the constructor.
        super(TalonFX::new, TalonFX::new, CANcoder::new,
        config.getDrivetrainConstants(),
        config.getModules());

        this.config = config;
        //setupPathPlanner();
        m_pigeon2 = this.getPigeon2();

        rotationController = new RotationController(config);

        if(Utils.isSimulation()) {startSimThread();}

        SmartDashboard.putData(this);

        fieldCentricDrive = new SwerveRequest.FieldCentric()
                .withDeadband(
                        config.getSpeedAt12Volts().in(MetersPerSecond) * kDeadband)
                .withRotationalDeadband(config.getMaxAngularRate() * kDeadband)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    }

    @Override
    public void periodic()
    {
      this.setOperatorPerspective();
      //SmartDashboard.putNumber("Pigeon", getPigeonHeading().getDegrees());
      currentPublisher.set(this.getState().ModuleStates);
      targetPublisher.set(this.getState().ModuleTargets);
      odomPublisher.set(getOdomHeading());
    }

    //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\

    public void initSendable(NTSendableBuilder builder) {
        SmartDashboard.putData(
                "Swerve Drive",
                new Sendable() {
                    @Override
                    public void initSendable(SendableBuilder builder) {
                        builder.setSmartDashboardType("SwerveDrive");

                        addModuleProperties(builder, "Front Left", 0);
                        addModuleProperties(builder, "Front Right", 1);
                        addModuleProperties(builder, "Back Left", 2);
                        addModuleProperties(builder, "Back Right", 3);

                        builder.addDoubleProperty("Robot Angle", () -> getRotationRadians(), null);
                    }
                });
    }

    private void addModuleProperties(SendableBuilder builder, String moduleName, int moduleNumber) {
        builder.addDoubleProperty(
                moduleName + " Angle",
                () -> getModule(moduleNumber).getCurrentState().angle.getRadians(),
                null);
        builder.addDoubleProperty(
                moduleName + " Velocity",
                () -> getModule(moduleNumber).getCurrentState().speedMetersPerSecond,
                null);
    }

    protected Command resetTurnController() {
        return runOnce(() -> resetRotationController());
    }

    protected Command setTargetHeading(double targetHeading) {
        return runOnce(() -> config.setTargetHeading(targetHeading));
    }

    private final SwerveRequest.FieldCentric fieldCentricDrive;
            
    private final SwerveRequest.RobotCentric robotCentricDrive =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(0).withRotationalDeadband(0);

    /**
   * The primary method for controlling the drivebase.  Takes a xSpeed, ySpeed and a rotation rate, and
   * calculates and commands module states accordingly. Also has field- and robot-relative modes, 
   * which affect how the translation vector is used.
   *
   * @param xSpeed  Velocity of x in m/s
   * @param ySpeed  Velocity of y in m/s
   * @param rot     Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(Double xSpeed, Double ySpeed, Double rot, Boolean fieldRelative)
  {
    if(fieldRelative){
        this.setControl(
        fieldCentricDrive
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rot)
        );
    }
    else{
        this.setControl(
        robotCentricDrive
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rot)
        );
    }
   }

   /**
     * Reset the turn controller and then run the drive command with a angle supplier. This can be
     * used for aiming at a goal or heading locking, etc.
     */
    public void aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        resetTurnController();
        drive(
            velocityX.getAsDouble(),
            velocityY.getAsDouble(),
            calculateRotationController(targetRadians),
            true);
    }

    /**
     * Reset the turn controller, set the target heading to the current heading(end that command
     * immediately), and then run the drive command with the Rotation controller. The rotation
     * controller will only engage if you are driving x or y.
     */
    public void headingLock(DoubleSupplier velocityX, DoubleSupplier velocityY) {
        resetTurnController();
        setTargetHeading(getRotation().getRadians());
        drive(
                velocityX.getAsDouble(),
                velocityY.getAsDouble(),
                rotateToHeadingWhenMoving(
                        velocityX, velocityY, () -> config.getTargetHeading()).getAsDouble(),
                true);
    }

    private DoubleSupplier rotateToHeadingWhenMoving(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier heading) {
        return () -> {
            if (Math.abs(velocityX.getAsDouble()) < 0.5
                    && Math.abs(velocityY.getAsDouble()) < 0.5) {
                return 0;
            } else {
                return calculateRotationController(heading::getAsDouble);
            }
        };
    }

    // Reorient Commands
    public Command reorientForward() {
        return reorientOperatorAngle(0);
    }

    public Command reorientLeft() {
        return reorientOperatorAngle(90);
    }

    public Command reorientBack() {
        return reorientOperatorAngle(180);
    }

    public Command reorientRight() {
        return reorientOperatorAngle(270);
    }

    public Command cardinalReorient() {
        return runOnce(
                () -> {
                    double angleDegrees = getClosestCardinal();
                    reorient(angleDegrees);
                });
    }


    /**
     * The function `getRobotPose` returns the robot's pose after checking and updating it.
     *
     * @return The `getRobotPose` method is returning the robot's current pose after calling the
     *     `seedCheckedPose` method with the current pose as an argument.
     */
    public Pose2d getRobotPose() {
        Pose2d pose = getState().Pose;
        return keepPoseOnField(pose);
    }

    // Keep the robot on the field
    private Pose2d keepPoseOnField(Pose2d pose) {
        double halfRobot = config.getRobotLength() / 2;
        double x = pose.getX();
        double y = pose.getY();

        double newX = Util.limit(x, halfRobot, Field.getFieldLength() - halfRobot);
        double newY = Util.limit(y, halfRobot, Field.getFieldWidth() - halfRobot);

        if (x != newX || y != newY) {
            pose = new Pose2d(new Translation2d(newX, newY), pose.getRotation());
            resetPose(pose);
        }
        return pose;
    }

    public Trigger inXzone(double minXmeter, double maxXmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getX(), () -> minXmeter, () -> maxXmeter));
    }

    public Trigger inYzone(double minYmeter, double maxYmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getY(), () -> minYmeter, () -> maxYmeter));
    }

     /**
     * This method is used to check if the robot is in the X zone of the field flips the values if
     * Red Alliance
     *
     * @param minXmeter
     * @param maxXmeter
     * @return
     */
    public Trigger inXzoneAlliance(double minXmeter, double maxXmeter) {
        return new Trigger(
                () -> Util.inRange(Field.flipXifRed(getRobotPose().getX()), minXmeter, maxXmeter));
    }

    /**
     * This method is used to check if the robot is in the Y zone of the field flips the values if
     * Red Alliance
     *
     * @param minYmeter
     * @param maxYmeter
     * @return
     */
    public Trigger inYzoneAlliance(double minYmeter, double maxYmeter) {
        return new Trigger(
                () -> Util.inRange(Field.flipYifRed(getRobotPose().getY()), minYmeter, maxYmeter));
    }

    // Used to set a control request to the swerve module, ignores disable so commands are
    //continuous.
    Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }
    
    private void setOperatorPerspective() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance()
                    .ifPresent(
                            allianceColor -> {
                                this.setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red
                                                ? config.getRedAlliancePerspectiveRotation()
                                                : config.getBlueAlliancePerspectiveRotation());
                                hasAppliedOperatorPerspective = true;
                            });
        }
    }

    protected void reorient(double angleDegrees) {
        resetPose(
                new Pose2d(
                        getRobotPose().getX(),
                        getRobotPose().getY(),
                        Rotation2d.fromDegrees(angleDegrees)));
    }

    protected Command reorientOperatorAngle(double angleDegrees) {
        return runOnce(
                () -> {
                    double output;
                    output = Field.flipTrueAngleIfRed(angleDegrees);
                    reorient(output);
                });
    }

    protected double getClosestCardinal() {
        double heading = getRotation().getRadians();
        if (heading > -Math.PI / 4 && heading <= Math.PI / 4) {
            return 0;
        } else if (heading > Math.PI / 4 && heading <= 3 * Math.PI / 4) {
            return 90;
        } else if (heading > 3 * Math.PI / 4 || heading <= -3 * Math.PI / 4) {
            return 180;
        } else {
            return 270;
        }
    }

    protected double getClosest45() {
        double angleRadians = getRotation().getRadians();
        double angleDegrees = Math.toDegrees(angleRadians);

        // Normalize the angle to be within 0 to 360 degrees
        angleDegrees = angleDegrees % 360;
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }

        // Round to the nearest multiple of 45 degrees
        double closest45Degrees = Math.round(angleDegrees / 45.0) * 45.0;

        // Convert back to radians and return as a Rotation2d
        return Rotation2d.fromDegrees(closest45Degrees).getRadians();
    }


    public void setFront() {
        reorientOperatorAngle(0);
      }

    //                     //
    // Rotation Controller //
    //                     //
    double getRotationControl(double goalRadians) {
        return rotationController.calculate(goalRadians, getRotationRadians());
    }

    void resetRotationController() {
        rotationController.reset(getRotationRadians());
    }

    Rotation2d getRotation() {
        return getRobotPose().getRotation();
    }

    double getRotationRadians() {
        return getRobotPose().getRotation().getRadians();
    }

    double calculateRotationController(DoubleSupplier targetRadians) {
        return rotationController.calculate(targetRadians.getAsDouble(), getRotationRadians());
    }



  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getPigeonHeading()
  {
    return m_pigeon2.getRotation2d();
  }

  public Rotation2d getOdomHeading()
  {
    return getRobotPose().getRotation();
  }

  

  public boolean leftTilted()
  {
    double roll = this.getPigeon2().getRoll().getValueAsDouble();
    if(roll > kDeadband){
      return true;
    }
    return false;
  }

  public boolean rightTilted()
  {
    double roll = this.getPigeon2().getRoll().getValueAsDouble();
    if(roll < -kDeadband){
      return true;
    }
    return false;
  }
  /**
   * Sets the heading of the robot using a {@link Rotation2d}. CCW positive, not wrapped.
   * 
   * @param angle {@link Rotation2d} to set the robot heading to
   */
  public void setHeading(Rotation2d angle){
    resetPose(new Pose2d(this.getRobotPose().getTranslation(), angle));
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    this.setControl(new SwerveRequest.SwerveDriveBrake());
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return new Rotation2d(this.getPigeon2().getPitch().getValueAsDouble());
  }

    //              //
    // Path Planner //
    //              //
    public void setupPathPlanner() {
        resetPose(
                new Pose2d(
                        Units.feetToMeters(27.0),
                        Units.feetToMeters(27.0 / 2.0),
                        config.getBlueAlliancePerspectiveRotation()));
        double driveBaseRadius = .4; //or something
        for(var moduleLocation : getModuleLocations()) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        RobotConfig robotConfig = null; // Just in case of exception, set to null first
        try {
            robotConfig = RobotConfig.fromGUISettings(); // Takes config from Robot Config on Pathplanner Settings
        }
        catch (Exception e){
            e.printStackTrace(); // Fallback to default config
            throw new RuntimeException(e);
        }
    
    SK25AutoBuilder.configure(
        () -> this.getState().Pose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, driveFeedForwards) -> 
                this.setControl(
                    AutoRequest.withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        kAutoPathConfig,
        robotConfig,
                    // ||  Boolean supplier that controls when the path will be mirrored for the red alliance
                    // ||  This will flip the path being followed to the red side of the field.
                    // \/  THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this // Reference to this subsystem to set requirements
        );
    }
    //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\

    //            //
    // Simulation //
    //            //
    private void startSimThread() { 
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(config.getKSimLoopPeriod());
}
    
}
