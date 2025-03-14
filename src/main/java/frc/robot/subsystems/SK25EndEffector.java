package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Konstants.EndEffectorConstants.kArmTolerance;
import static frc.robot.Ports.EndEffectorPorts.kEndEffectorArmMotor;
import static frc.robot.Ports.EndEffectorPorts.kEndEffectorRollerMotor;
import static frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition.kZeroPositionAngle;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;

public class SK25EndEffector extends SubsystemBase
{
    //may not need anymore, external encoder now in use, keep just in case.
    final double motorRatio = 12.5;
    final int gear1Rotation = 1;
    final int gear2Rotation = 1;
    final int degrees = 360;

    SparkFlex armMotor;
    SparkFlex rollerMotor;
    SparkFlexConfig armConfig;

    SparkClosedLoopController mPID;
    double mTargetAngle;
    double mCurrentAngle;

    public RelativeEncoder mEncoder;

    ArmFeedforward  armFeedforward;

    double armTargetAngle;

    public boolean isRunning;

    Pref<Double> armKg = SKPreferences.attach("armKg", 0.05)
        .onChange((newValue) -> {
            armFeedforward = new ArmFeedforward(0, newValue, 0, 0);
        });
        Pref<Double> armAngleDeg = SKPreferences.attach("armAngleDeg", 0.0)
        .onChange((newValue) -> {
            setTargetAngle(newValue);
        });

    //LaserCan laserCanSensor;

    public SK25EndEffector()
    {
        

        
        //initialize the new motor object with its motor ID and type
        rollerMotor = new SparkFlex(kEndEffectorRollerMotor.ID, MotorType.kBrushless);
        armMotor = new SparkFlex(kEndEffectorArmMotor.ID, MotorType.kBrushless);

        armConfig = new SparkFlexConfig();
        
        armConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1.9)
            .i(.0002)
            .d(2.1)
            .outputRange(-.1, .1) //TODO: Add a velocityFF in order to provide a feedforwards to counteract gravity and maintain the arm at a set point
            //.p(0, ClosedLoopSlot.kSlot1)
            //.i(0, ClosedLoopSlot.kSlot1)
            //.d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0/5767, ClosedLoopSlot.kSlot1);
            //.outputRange(-1, 1, ClosedLoopSlot.kSlot1);

            armConfig.closedLoop.maxMotion
            .maxAcceleration(500)
            .maxVelocity(550)
            .allowedClosedLoopError(1);

        armConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30); // TODO: Consider adding a .voltageCompensation(double nominalVoltage) in order to limit maximum volts to the motor

        mPID = armMotor.getClosedLoopController();
        mEncoder = armMotor.getEncoder();
        
        armFeedforward = new ArmFeedforward(0, armKg.get(), 0, 0); // Is this used anywhere?

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        mTargetAngle = 0.0;
        mCurrentAngle = 0.0;

        mEncoder.setPosition(0);

        //laserCanSensor = new LaserCan(kLaserCanEndEffector.ID);
    }

    public void initialize()
    {

    }
        
    
    public void resetEncoder()
    {
        mEncoder.setPosition(0);
    }

    public void setTargetAngle(double angleDegrees)
    {
        mTargetAngle = angleDegrees;
        SmartDashboard.putNumber("EffectorTargetAngle", mTargetAngle);
        SmartDashboard.putNumber("EffectorCurrentAngle", getArmPosition());

        double motorRotations = angleDegrees * motorRatio / degrees;

        //System.out.println("Motor " + motorRotations);
        //System.out.println("Encoder " + mEncoder.getPosition());
        //Come back and change this, need fraction for Encoder Rotations in place of angle
        // double targetAngleRadians = 
        //     Degrees.of(angleDegrees)
        //     .plus(Degrees.of(90))
        //     .in(Radians);
        //System.out.println(targetAngleRadians);
        //double armFF = armFeedforward.calculate(targetAngleRadians, 0);
        mPID.setReference(motorRotations, ControlType.kPosition,ClosedLoopSlot.kSlot0);
    }

    /**
     * Arm position in degrees
     */
    public double getArmPosition()
    {
        //Set conversion factor
        double motorRotations = mEncoder.getPosition();
        double angle = motorRotations / motorRatio * degrees;
        return angle;
    }

    public double getTargetArmPosition()
    {
       return mTargetAngle;

    }

    public boolean isArmAtTargetPosition()
    {
        //double la =  getTargetArmPosition() -getArmPosition();
        //System.out.println(la);
        //System.out.println(getTargetArmPosition());
        //System.out.println(getArmPosition());
        return Math.abs( getTargetArmPosition() -getArmPosition()) < kArmTolerance;
    }

    public void hold()
    { 
        if(isRunning == true)
        {
            mTargetAngle = getArmPosition();
            isRunning = false;
        }
        setTargetAngle(mTargetAngle);   
    }

    public void leave() {
        setTargetAngle(kZeroPositionAngle.angle);
    }

    /*public boolean haveCoral()
    {
        LaserCan.Measurement sensorMeasurement = laserCanSensor.getMeasurement();
        if ((sensorMeasurement != null && sensorMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)) {
            SmartDashboard.putNumber("LaserCan distance", sensorMeasurement.distance_mm);
          if(sensorMeasurement.distance_mm < (kCoralToLaserCanDistance+10))//plus 10 so theres room for error
          {
            return true;
          }
        } 
        return false;
    }
        */
    public void checkPositionUp()
     {
    
        double encoder = mEncoder.getPosition();
        double angle = (encoder * gear2Rotation * degrees) / motorRatio / gear1Rotation;

        if(angle > -10)
        {
            setTargetAngle(-10);
            stopArm();
        }
     }
     public void checkPositionDown()
     {
        double encoder = mEncoder.getPosition();
        double angle = (encoder * gear2Rotation * degrees) / motorRatio / gear1Rotation;

        if(angle < -180)
        {
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            //TODO : FIX THIS IN ENDEFFECTOR!!!!!!!!!!!!!!!!!!!
            
            setTargetAngle(140);
            stopArm();
        }
     }


            
     


     public void runRoller(double rollerspeed)
    {
        rollerMotor.set(rollerspeed);
    }

    public Command runRollerCommand(double rollerSpeed)
    {
        return Commands.sequence(
            Commands.parallel(
                Commands.waitSeconds(0.5),
                new InstantCommand(() -> {
                    rollerMotor.set(rollerSpeed);
                })
            ),
            new InstantCommand(() -> rollerMotor.set(0))
        );
    }

    public void runArm(double armspeed)
    {
        double angleDegrees = getArmPosition();
        double targetAngleRadians = 
            Degrees.of(angleDegrees)
            .plus(Degrees.of(90))
            .in(Radians);
        
        double offset = armKg.get() * Math.cos(targetAngleRadians);
        armMotor.set(armspeed + offset);
    }

    //stops the motor
    public void stopRoller()
    {
        rollerMotor.stopMotor();
    }

    public void stopArm()
    {
        armMotor.stopMotor();
    }

    public void periodic()
    {

        
        /*if (SmartDashboard.getBoolean("Control Mode", false)) 
        {
            double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
            mPID.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        } 
        else 
        {
            double targetPosition = SmartDashboard.getNumber("Target Position", 0);
            mPID.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        SmartDashboard.putNumber("Actual Position", mEncoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", mEncoder.getVelocity());

        if (SmartDashboard.getBoolean("Reset Encoder", false)) 
        {
            SmartDashboard.putBoolean("Reset Encoder", false);
            mEncoder.setPosition(0);
        }
        
        double currentPosition = getArmPosition();
        double armTargetPosition = getTargetArmPosition();
        
        SmartDashboard.putNumber("Current Estimated Position", currentPosition);

        SmartDashboard.putNumber("Arm Target Position", armTargetPosition);
        SmartDashboard.putBoolean("Arm at Setpoint", isArmAtTargetPosition());*/

        SmartDashboard.putNumber("EndEffector Current Angle", mCurrentAngle);
        SmartDashboard.putNumber("EndEffector Target Angle", mTargetAngle);
        SmartDashboard.putBoolean("Is EndEffector at Target", isArmAtTargetPosition());
        
    }

    public void testPeriodic(){}
    public void testInit(){}

}