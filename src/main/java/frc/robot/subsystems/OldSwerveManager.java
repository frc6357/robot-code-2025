// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Radians;

// import edu.wpi.first.units.measure.Angle;

// public class OldSwerveManager {

//     //declare swerve wheels
//     OldSwerveWheel frontRightWheel;
//     OldSwerveWheel frontLeftWheel;
//     OldSwerveWheel backRightWheel;
//     OldSwerveWheel backLeftWheel;

//     /**
//      * Creates a new SwerveManager which provides methods to translate and rotate the robot.
//      * @param frontRightWheel The front right wheel object to be used.
//      * @param frontLeftWheel The front left wheel object to be used.
//      * @param backRightWheel The back right wheel object to be used.
//      * @param backLeftWheel The back leftwheel object to be used.
//      */
//     public OldSwerveManager(OldSwerveWheel frontRightWheel, OldSwerveWheel frontLeftWheel, OldSwerveWheel backRightWheel, OldSwerveWheel backLeftWheel)
//     {
//         //initialize wheels
//         this.frontRightWheel = frontRightWheel;
//         this.frontLeftWheel = frontLeftWheel;
//         this.backRightWheel = backRightWheel;
//         this.backLeftWheel = backLeftWheel;
//     }

//     /**
//      * Translates the robot in a direction using polar corrdiantes.
//      * @param direction The direction to translate from the front as a degree value from 0.0 to 360.0.
//      * @param speed The speed of the translation for the wheel as a percentage value from 0.0 to 1.0.
//      */
//     public void translate(Angle direction, double speed)
//     {
//         //sets the directoin of each turn motor
//         frontRightWheel.setDirection(direction);
//         frontLeftWheel.setDirection(direction);
//         backRightWheel.setDirection(direction);
//         backLeftWheel.setDirection(direction);

//         //sets the speed of each drive motor
//         frontRightWheel.setSpeed(speed);
//         frontLeftWheel.setSpeed(speed);
//         backRightWheel.setSpeed(speed);
//         backLeftWheel.setSpeed(speed);
//     }

//     /**
//      * Rotates the robot in place.
//      * @param speed The speed at which to rotate as a percentage value from 0.0 to 1.0.
//      */
//     public void rotate(double speed)
//     {
//         //set the direction for each wheel using angle values
//         frontRightWheel.setDirection(Degrees.of(45.0));
//         frontLeftWheel.setDirection(Degrees.of(-45.0));
//         backRightWheel.setDirection(Degrees.of(-45.0));
//         backLeftWheel.setDirection(Degrees.of(45.0));

//         //sets the speed of each drive wheel
//         frontRightWheel.setSpeed(speed);
//         frontLeftWheel.setSpeed(speed);
//         backRightWheel.setSpeed(speed);
//         backLeftWheel.setSpeed(speed);
//     }

//     /**
//      * Determines the closet angle to the target angle from the compared angle by converting angles between 180.0 and 360.0 to their respective negative Unit Circle values. .
//      * @param targetAngle The target angle to rotate toward.
//      * @param comparisonAngle The angle used as comparison to the target angle.
//      * @return The closest angle to rotate toward between -180.0 and 180.0.
//      */
    

//     /**
//      * Translates and rotates the robot at the same time by pointing the wheels in the hardcoded angle directions to do so.
//      * @param direction The direction of the translation between 0.0 and 360.0.
//      * @param translateSpeed The speed of translation as a percentage from 0.0 to 1.0.
//      * @param rotateSpeed The speed of rotation as a percentage speed.
//      */
//     public void translateAndRotate(double direction, double translateSpeed, double rotateSpeed)
//     {

//         //rotation direction plus rotation speed
//         Angle rotationTarget = Radians.of(direction);//.plus(Degrees.of(rotateSpeed * 45.0));
//         frontRightWheel.setDirection(rotationTarget);
//         frontLeftWheel.setDirection(rotationTarget);
//         backRightWheel.setDirection(rotationTarget);
//         backLeftWheel.setDirection(rotationTarget);




//         //scale the double speed value to an angle value
//         //double rotateAngle = rotateSpeed * 45.0;

//          // if the left front wheel is in the front
//         //if (closestAngle(direction, 135.0) >= 90.0)
//         //{
//             //frontLeftWheel.setDirection(direction + rotateAngle);
//         //}
//         // if it's in the back
//         //else
//         //{
//             //frontLeftWheel.setDirection(direction - rotateAngle);
//         //}
//         // if the left back wheel is in the front
//         //if (closestAngle(direction, 225.0) > 90.0)
//         //{
//             //backLeftWheel.setDirection(direction + rotateAngle);
//         //}
//         // if it's in the back
//         //else
//         //{
//             //backLeftWheel.setDirection(direction - rotateAngle);
//         //}
//         // if the right front wheel is in the front
//         //if (closestAngle(direction, 45.0) > 90.0)
//         //{
//             //frontRightWheel.setDirection(direction + rotateAngle);
//         //}
//         // if it's in the back
//         //else
//         //{
//             //frontRightWheel.setDirection(direction - rotateAngle);
//         //}
//         // if the right back wheel is in the front
//         //if (closestAngle(direction, 315.0) >= 90.0)
//         //{
//             //backRightWheel.setDirection(direction + rotateAngle);
//         //}
//         // if it's in the back
//         //else
//         //{
//             //backRightWheel.setDirection(direction - rotateAngle);
//         //}

//         //set the drive motors' speeds
//         frontRightWheel.setSpeed(translateSpeed);
//         frontLeftWheel.setSpeed(translateSpeed);
//         backRightWheel.setSpeed(translateSpeed);
//         backLeftWheel.setSpeed(translateSpeed);
//     }

    
//     /**
//      * Higher method to handle all movement options dictated by the above translation and rotation methods.
//      * @param direction The direction to translate in from 0.0 to 360.0.
//      * @param translateSpeed The speed to translate at as a percentage from 0.0 to 1.0.
//      * @param rotateSpeed The speed to rotate at as a percentage from 0.0 to 1.0.
//      */
//     public void setSwerve(double direction, double translateSpeed, double rotateSpeed)
//     {
//         //if the robot is not translating but is rotating.
//         if(( translateSpeed == 0.0) && (rotateSpeed != 0.0))
//         {
//             //rotate at rotation speed.
//             rotate(rotateSpeed);
//         }
//         else
//         {
//             //translate and rotate with respective speeds toward direction.
//             translateAndRotate(direction, translateSpeed, rotateSpeed);
//         }
//     }
// }
