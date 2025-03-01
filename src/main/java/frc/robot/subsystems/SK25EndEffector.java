package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Konstants.EndEffectorConstants.*;
import static frc.robot.Ports.EndEffectorPorts.kEndEffectorArmMotor;
import static frc.robot.Ports.EndEffectorPorts.kEndEffectorRollerMotor;
//import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;
//import static frc.robot.Konstants.EndEffectorConstants.kCoralToLaserCanDistance;
///import static frc.robot.Ports.EndEffectorPorts.kLaserCanEndEffector;

//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkMax;

public class SK25EndEffector extends SubsystemBase
{
    //TODO: Check these ratios
    final int motorRatio = 25;
    final int gear1Rotation = 1;
    final int gear2Rotation = 1;
    final int degrees = 360;

    // SparkMax armMotor;
    SparkMax rollerMotor;
    TalonFX armMotor;

    /* TalonFX Arm motor variables */
    Slot0Configs armSlot0;
    TalonFXConfiguration armConfig;
    MotionMagicConfigs armMotionConfig;
    CurrentLimitsConfigs armCurrentLimits;
    final MotionMagicVoltage armMotorRequest = new MotionMagicVoltage(0);

    double armTargetAngle;

    ArmFeedforward  armFeedforward;


    public boolean isRunning;

    Pref<Double> armKg = SKPreferences.attach("armKg", 0.1)
        .onChange((newValue) -> {
            armFeedforward = new ArmFeedforward(0, newValue, 0, 0);
        });

    //LaserCan laserCanSensor;

    public SK25EndEffector()
    {
        //initialize the new motor object with its motor ID and type
        rollerMotor = new SparkMax(kEndEffectorRollerMotor.ID, MotorType.kBrushless);
        armMotor = new TalonFX(kEndEffectorArmMotor.ID);

        armMotor.setNeutralMode(NeutralModeValue.Brake);

        armCurrentLimits = kArmCurrentLimitsConfigs;

        armSlot0 = new Slot0Configs()
            .withKP(kArmP)
            .withKI(kArmI)
            .withKD(kArmD)
            .withGravityType(GravityTypeValue.Arm_Cosine) // TODO: Apply offset/tune encoder to know what perfectly horizontal is
            .withKV(kArmV); // TODO: Use kV or kG??
        
        armMotionConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(kArmCruiseVel) // rot/sec
            .withMotionMagicAcceleration(kArmTargetAccel) // rot/sec^2
            .withMotionMagicJerk(kArmTargetJerk); // rot/sec^3
        
        armConfig = new TalonFXConfiguration()
            .withSlot0(armSlot0)
            .withMotionMagic(armMotionConfig)
            .withCurrentLimits(armCurrentLimits);

        armMotor.getConfigurator().apply(armConfig);
        
        armFeedforward = new ArmFeedforward(0, armKg.get(), 0, 0); // Is this used anywhere?

        armTargetAngle = 0.0;

        //laserCanSensor = new LaserCan(kLaserCanEndEffector.ID);
    }

        
    
    // public void resetEncoder()
    // {
    //     
    // }

    public void setTargetAngle(EndEffectorPosition pos)
    {
        setTargetAngle(pos.angle);
    }

    public void setTargetAngle(double angleDegrees)
    {
        armTargetAngle = angleDegrees;

        Angle targetAngle = Degrees.of(angleDegrees); 
        double motorRotations =  targetAngle.in(Rotations)* motorRatio;

        //System.out.println("Motor " + motorRotations);
        //System.out.println("Encoder " + mEncoder.getPosition());
        //TODO: Come back and change this, need fraction for Encoder Rotations in place of angle
        // double targetAngleRadians = 
        //     Degrees.of(angleDegrees)
        //     .plus(Degrees.of(90))
        //     .in(Radians);

        armMotor.setControl(armMotorRequest.withPosition(motorRotations));
    }

    /**
     * Arm position in degrees
     */
    public double getArmPosition()
    {
        //Set conversion factor
        double motorRotations = armMotor.getPosition().getValueAsDouble();
        double angle = motorRotations / motorRatio * degrees;
        return angle;
    }

    public double getTargetArmPosition()
    {
       return armTargetAngle;

    }

    public boolean isArmAtTargetPosition()
    {
        return Math.abs( getTargetArmPosition() -getArmPosition()) < kArmTolerance;
    }

    public void hold()
    { 
        if(isRunning == true)
        {
            armTargetAngle = getArmPosition(); // Sets the target position to wherever it is at that moment
            isRunning = false;
        }
        setTargetAngle(armTargetAngle);   
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
    
        double motorAngle = armMotor.getPosition().getValueAsDouble();
        double angle = (motorAngle * gear2Rotation * degrees) / motorRatio / gear1Rotation;

        if(angle < 10)
        {
            setTargetAngle(10);
            stopArm();
        }
     }
     public void checkPositionDown()
     {
        double motorAngle = armMotor.getPosition().getValueAsDouble();
        double angle = (motorAngle * gear2Rotation * degrees) / motorRatio / gear1Rotation;

        if(angle > 140)
        {
            setTargetAngle(140);
            stopArm();
        }
     }


            
     


     public void runRoller(double rollerspeed)
    {
        rollerMotor.set(rollerspeed);
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
        
    }

    public void testPeriodic(){}
    public void testInit(){}

}