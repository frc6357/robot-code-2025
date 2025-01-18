package frc.robot.subsystems;

public class SwerveManager {

    SwerveWheel frontRightWheel;
    SwerveWheel frontLeftWheel;
    SwerveWheel backRightWheel;
    SwerveWheel backLeftWheel;

    SwerveWheel[] wheels = {frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel};

    public SwerveManager(SwerveWheel frontRightWheel, SwerveWheel frontLeftWheel, SwerveWheel backRightWheel, SwerveWheel backLeftWheel)
    {
        this.frontRightWheel = frontRightWheel;
        this.frontLeftWheel = frontLeftWheel;
        this.backRightWheel = backRightWheel;
        this.backLeftWheel = backLeftWheel;
    }

    public void translate(double direction, double speed)
    {
        for (SwerveWheel i : wheels)
        {
            i.setDirection(direction);
            i.setSpeed(speed);
        }
    }

    public void rotate(double speed)
    {
        frontRightWheel.setDirection(135.0);
        frontLeftWheel.setDirection(45.0);
        backRightWheel.setDirection(-45.0);
        backLeftWheel.setDirection(-135.0);

        for (SwerveWheel i : wheels)
        {
            i.setSpeed(speed);
        }
    }

    private static double closestAngle(double targetAngle, double comparisonAngle)
    {
        // get direction
        //double dir = closestAngle(b, 360.0) - closestAngle(a, 360.0);
        double dir = targetAngle - comparisonAngle;

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0)
        {
                dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
    }

    public void translateAndRotate(double direction, double translateSpeed, double rotateSpeed)
    {
        double rotateAngle = rotateSpeed * 45.0;

         // if the left front wheel is in the front
        if (closestAngle(direction, 135.0) >= 90.0)
        {
            frontLeftWheel.setDirection(direction + rotateAngle);
        }
        // if it's in the back
        else
        {
            frontLeftWheel.setDirection(direction - rotateAngle);
        }
        // if the left back wheel is in the front
        if (closestAngle(direction, 225.0) > 90.0)
        {
            backLeftWheel.setDirection(direction + rotateAngle);
        }
        // if it's in the back
        else
        {
            backLeftWheel.setDirection(direction - rotateAngle);
        }
        // if the right front wheel is in the front
        if (closestAngle(direction, 45.0) > 90.0)
        {
            frontRightWheel.setDirection(direction + rotateAngle);
        }
        // if it's in the back
        else
        {
            frontRightWheel.setDirection(direction - rotateAngle);
        }
        // if the right back wheel is in the front
        if (closestAngle(direction, 315.0) >= 90.0)
        {
            backRightWheel.setDirection(direction + rotateAngle);
        }
        // if it's in the back
        else
        {
            backRightWheel.setDirection(direction - rotateAngle);
        }

        for (SwerveWheel i : wheels)
        {
            i.setSpeed(translateSpeed);
        }
    }

    public void setSwerve(double direction, double translateSpeed, double rotateSpeed)
    {
        if(( translateSpeed == 0.0) && (rotateSpeed != 0.0))
        {
            rotate(rotateSpeed);
        }
        else
        {
            translateAndRotate(direction, translateSpeed, rotateSpeed);
        }
    }
}
