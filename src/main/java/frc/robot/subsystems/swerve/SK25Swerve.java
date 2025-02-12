package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Konstants.AutoConstants.kAutoPathConfig;
import static frc.robot.Konstants.SwerveConstants.kBlueAlliancePerspective;
import static frc.robot.Konstants.SwerveConstants.kChassisLength;
import static frc.robot.Konstants.SwerveConstants.kJoystickDeadband;
import static frc.robot.Konstants.SwerveConstants.kMaxAngularRate;
import static frc.robot.Konstants.SwerveConstants.kRedAlliancePerspective;
import static frc.robot.Konstants.SwerveConstants.kSimulationLoopPeriod;
import static frc.robot.Konstants.SwerveConstants.kSpeedAt12VoltsMeterPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// Packages used for the mechanisms/motors for the swerve drivetrain itself
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
// import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
// import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveRequest;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.Field;
import frc.robot.utils.SK25AutoBuilder;
import frc.robot.utils.Util;



public class SK25Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements NTSendable, Subsystem {
    //The constants config object containing all constants for the drivetrain.
    private SwerveConstantsConfigurator config;
    //simulation stuff                
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    //new rotation controller
    private RotationController rotationController;
    //new pigeon gyro/accelerometer
    private Pigeon2 m_pigeon2;

    //Determines if the robot should be facing toward the opposing driverstation at all times.    //TODO: rename constant and method, and move to constants
    private boolean hasAppliedOperatorPerspective = false;

    /**New robot speed applier object which gives the chassisspeeds to the drivetrain object. 
    *Current speeds are zero since constructor is empty.*/
    private final SwerveRequest.ApplyRobotSpeeds AutoRequest = new SwerveRequest.ApplyRobotSpeeds();
    
    //Creates publishers for logging

    //log the current swerve states
    StructArrayPublisher<SwerveModuleState> currentPublisher = 
    NetworkTableInstance.getDefault().
    getStructArrayTopic("CurrentSwerveStates", SwerveModuleState.struct).publish();
    //log the target swerve states
    StructArrayPublisher<SwerveModuleState> targetPublisher = 
    NetworkTableInstance.getDefault().
    getStructArrayTopic("TargetSwerveStates", SwerveModuleState.struct).publish();
    //log the rotation of the robot
    StructPublisher<Rotation2d> odomPublisher = NetworkTableInstance.getDefault().
    getStructTopic("Rotation", Rotation2d.struct).publish();

    /**
     * Constructs a new Swerve drive subsystem.
     *
     * @param config The configuration object containing constants for 
     * the drivetrain and the module configurations.
     */ 
    public SK25Swerve(SwerveConstantsConfigurator config) {
        // Creates a Swerve Drivetrain using Phoenix6's SwerveDrivetrain class, passing the
        // properties of the swerve drive itself from SwerveConfig into the constructor.
        super(TalonFX::new, TalonFX::new, CANcoder::new,
        config.getDrivetrainConstants(),
        config.getModules());

        this.config = config;
        //setupPathPlanner();                  //TODO: enable pathplanner setup at some point
        m_pigeon2 = this.getPigeon2();

        rotationController = new RotationController(config);

        //start simulation when ran
        if(Utils.isSimulation()) {startSimThread();}

        //log this class, which includes all of its data sent by the publishers
        SmartDashboard.putData(this);

        //create a new open loop feild centric drive request
        fieldCentricDrive = new SwerveRequest.FieldCentric()
                .withDeadband(
                        kSpeedAt12VoltsMeterPerSecond.in(MetersPerSecond) * kJoystickDeadband)
                .withRotationalDeadband(kMaxAngularRate * kJoystickDeadband)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    }

    @Override
    public void periodic()
    {
      //periodicaly set the driver/operator perspective (the robot is set to face toward the 
      //opposing driver station)
      this.setOperatorPerspective();
      //SmartDashboard.putNumber("Pigeon", getPigeonHeading().getDegrees());      //TODO: Publish Pigeon data?
      
      //Update the publisher objects with the most recent states and odometry heading
      currentPublisher.set(this.getState().ModuleStates);
      targetPublisher.set(this.getState().ModuleTargets);
      odomPublisher.set(getOdomHeading());
    }

    //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\

    //create new tabs/porperties on the display with the builder object
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

    //assign properties to the modules on the dashboard using the their most recent values
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

    //runs the reset rotation controller method as a command
    protected Command resetTurnController() {
        return runOnce(() -> resetRotationController());
    }

    //run the set target heading method as a command
    protected Command setTargetHeading(double targetHeading) {
        return runOnce(() -> config.setTargetHeading(targetHeading));
    }

    //new feild centric swerve request
    private final SwerveRequest.FieldCentric fieldCentricDrive;
            
    //new robot centric request
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
     * Reset the turn controller and then run the drive command with an angle supplier. This can be
     * used for aiming at a goal or heading locking, etc.
     */
    public void aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        resetTurnController();
        drive(
            velocityX.getAsDouble(),
            velocityY.getAsDouble(),
            //rotate to the target from the current rotation
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
        //set target heading to current heading
        setTargetHeading(getRotation().getRadians());
        drive(
                velocityX.getAsDouble(),
                velocityY.getAsDouble(),
                //rotates to target heading when moving (this example uses the current heading, meaning
                //the rotation value will always try to stay in the same place.
                rotateToHeadingWhenMoving(
                        velocityX, velocityY, () -> config.getTargetHeading()).getAsDouble(),
                true);
    }

    /** Rotates to the specified heading. If not moving faster than 0.5 in the x and y direction, 
     * don't rotate*/
    private DoubleSupplier rotateToHeadingWhenMoving(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier heading) {
        return () -> {
            if (Math.abs(velocityX.getAsDouble()) < 0.5
                    && Math.abs(velocityY.getAsDouble()) < 0.5) {            //TODO: see if 0.5 restriction is too much/little
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

    /**Gets the closest cardinal direction to the current rotation, and uses the reorient method to 
     * reach that rotation.*/
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

    /** Keep the robot on the field using the feild length from Util by checking if the position is off 
     * the feild, then replacing it with the correct position
     * @return The new pose after limiting out of feild possibilities.
     */
    private Pose2d keepPoseOnField(Pose2d pose) {
        double halfRobot = kChassisLength / 2;
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

    /** Gets the trigger containing the boolean of if the x value is within the min and max. */
    public Trigger inXzone(double minXmeter, double maxXmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getX(), () -> minXmeter, () -> maxXmeter));
    }

    /** Gets the trigger containing the boolean of if the y value is within the min and max. */
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

    /** Used to set a control request to the swerve module, ignores disable so commands are
    *continuous.*/
    Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }
    
    /** Stes the default oreintation of the robot to face away from our driverstation toward the
     * opposing alliance's driverstation. This method should be ran periodicaly to ensure that the 
     * perspective value remains unchanged in the event of a sudden error or disable.
     */
    private void setOperatorPerspective() {                    //TODO: see if this clashes with the set oreintation button
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
                                                ? kRedAlliancePerspective
                                                : kBlueAlliancePerspective);
                                hasAppliedOperatorPerspective = true;
                            });
        }
    }

    /** Rotates the robot to the new rotation. */
    protected void reorient(double angleDegrees) {
        resetPose(
                new Pose2d(
                        getRobotPose().getX(),
                        getRobotPose().getY(),
                        Rotation2d.fromDegrees(angleDegrees)));
    }

    /** Rotates the robot position using the reorient method to the new rotation with respect to the 
     * driver/operator position/persepctive. */
    protected Command reorientOperatorAngle(double angleDegrees) {
        return runOnce(
                () -> {
                    double output;
                    output = Field.flipTrueAngleIfRed(angleDegrees);
                    reorient(output);
                });
    }

    /** Gets the closest cardinal direction to the robot rotation.
     * @return The direction to rotate to in degrees.
     */
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

    /** Gets the closets 45 degree angle to the current robot rotation. 
     * @return The closest angle which is a multiple of 45 in degrees.
     */
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


    /** Stes the new front position of the robot by stating the current orientation as the default one. */
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

  

  /** Gets if the robot is facing to the left relative to the forward facing orientation.
   * @return if the robot is left tilted (true) or not (false).
   */
  public boolean leftTilted()
  {
    double roll = this.getPigeon2().getRoll().getValueAsDouble();
    if(roll > kJoystickDeadband){
      return true;
    }
    return false;
  }

  /** Gets if the robot is facing to the right relative to the forward facing orientation.
   * @return if the robot is right tilted (true) or not (false).
   */
  public boolean rightTilted()
  {
    double roll = this.getPigeon2().getRoll().getValueAsDouble();
    if(roll < -kJoystickDeadband){
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
                        kBlueAlliancePerspective));
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
        m_simNotifier.startPeriodic(kSimulationLoopPeriod);
}
    
}
