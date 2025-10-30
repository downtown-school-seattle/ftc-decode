package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Auto Controller v0.1", group = "team code")
public class AutoController extends OpMode {
    
    double xLocEstimate = 0; // Overall xloc best guess
    // 0 = back 1 = front

    double yLocEstimate = 0; // Overall yloc best guess
    // 0 = left 1 = right

    double currentX;
    double previousX;
    double currentY;
    double previousY;
    double currentAngle;
    double previousAngle;

    static final double ENCODER_PER_MM = (537.7*19.2)/(104*Math.PI);

    // % of full rotation per second
    static final double ROTATE_SPEED = 0.5;

    // mm
    static final double BRAKE_THRESHOLD = 10;
    static final double ROTATE_THRESHOLD = 10;
    static final double DECELERATION_DISTANCE_X = 450; //This is a placeholder value.
    // It is the distance in mm for the robot to decelerate from full speed. #TODO change this to a real value.

    static final double DECELERATION_DISTANCE_Y = 300; //This is a placeholder value.
    // It is the distance in mm for the robot to decelerate from full speed. #TODO change this to a real value.

    static final double DECELERATION_DISTANCE_ANGULAR = Math.PI/6; //This is a placeholder value.
    // It is the distance in radians for the robot to decelerate from full speed. #TODO change this to a real value.
    //sec
    static final double DECELERATION_TIME = 5; //This is a placeholder value.
    // It is the time in seconds for the robot to decelerate from full speed. #TODO change this to a real value.
    static final double MAX_MM_PER_SECOND_X = 1000; //TODO change this to a real value.
    static final double MAX_MM_PER_SECOND_Y = 800; //TODO change this to a real value.
    static final double MAX_MM_PER_SECOND_RADIANS = 2; //TODO change this to a real value.

    static final double minPowerToMoveRobot = 0.2; //This is a placeholder value.TODO change this to a real value


    // TODO: why do we need this again?
    // The robot isn't quite a square, so seperate worldXSize and worldYSize
    // Seem to be needed. (anything else assumes the robot is a point)
    // However, what happens if the robot is rotated? The world size would have to change
    // Far easier is to approximate the robot as a sphere
    // Only cost is 0.1m on one side of the map that can't be accessed
    static final double WORLD_SIZE = ((12. * 12.) / 25.4) - 630.5813904643873;

//    double xwidth = 440;
//    double ywidth = 451.7;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    IMU imu;
    GoBildaPinpointDriver pinpoint;

    double targetX = 100;
    double targetY = 0;
    double targetRadians = 0;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        imu = hardwareMap.get(IMU.class, "imu");
        initializeIMU();

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        configurePinpoint();

        telemetry.addLine("Make sure to rotate the robot to face the wall with the goals on them.");
        telemetry.update();
    }

    @Override
    public void loop() {
        pinpoint.update();
        currentAngle = pinpoint.getHeading(AngleUnit.RADIANS);
        currentX = pinpoint.getPosX(DistanceUnit.MM);
        currentY = pinpoint.getPosY(DistanceUnit.MM);
        double xDistance = targetX - currentX;
        double yDistance = targetY - currentY;
        double rotDistance = targetRadians - currentAngle;

        move(xDistance, yDistance, rotDistance, 10, Math.PI/12);

        telemetry.addData("currentAngle", currentAngle);
        telemetry.addData("currentX", currentX);
        telemetry.addData("currentY", currentY);

        telemetry.addData("rotDistance", rotDistance);
        telemetry.addData("xDistance", xDistance);
        telemetry.addData("yDistance", yDistance);

        telemetry.addData("targetRadians", targetRadians);
        telemetry.addData("targetX", targetX);
        telemetry.addData("targetY", targetY);

        telemetry.addData("XVelocity", getVelocityX());
        telemetry.addData("YVelocity", getVelocityY());
        telemetry.addData("AngularVelocity", getAngularVelocity());
        telemetry.update();

        previousAngle = currentAngle;
        previousX = currentX;
        previousY = currentY;
    }
    public boolean move(double xDistance, double yDistance, double rotDistance, double distanceAccuracy,
                     double rotAccuracy){
        if (Math.abs(xDistance) + Math.abs(yDistance) < distanceAccuracy * 2 && Math.abs(rotDistance) < rotAccuracy) {
            // TODO: update targets
            return true;
        } else {
//            driveFieldRelative(calculatePowerX(getVelocityX(), xDistance),
//                    calculatePowerY(getVelocityY(), yDistance),
//                    calculatePowerAngular(getAngularVelocity(), rotDistance));
//            driveFieldRelative(-calculatePowerX(getVelocityX(), xDistance), 0, 0);
            frontLeftDrive.setPower(-calculatePowerX(getVelocityX(), xDistance));
            frontRightDrive.setPower(-calculatePowerX(getVelocityX(), xDistance));
            backLeftDrive.setPower(-calculatePowerX(getVelocityX(), xDistance));
            backRightDrive.setPower(-calculatePowerX(getVelocityX(), xDistance));

            return false;
        }
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public double calculatePowerX (double velocityX, double distanceToTargetX){
        double XPower;
        double shouldHavePower = (DECELERATION_DISTANCE_X/distanceToTargetX)*(velocityX/MAX_MM_PER_SECOND_X);
        if (Math.abs(shouldHavePower) < 1){
            if(distanceToTargetX > 0){
                XPower = 1;
            } else {
                XPower = -1;
            }
        } else {
            XPower = 0;
        }
        return XPower;
    }

    public double calculatePowerY (double velocityY, double distanceToTargetY){
        double YPower;
        double shouldHavePower = (DECELERATION_DISTANCE_Y/distanceToTargetY)*(velocityY/MAX_MM_PER_SECOND_Y);
        if (Math.abs(shouldHavePower) < 1){
            if(distanceToTargetY > 0){
                YPower = 1;
            } else {
                YPower = -1;
            }
        } else {
            YPower = 0;
        }
        return YPower;
    }

    public double calculatePowerAngular (double velocityAngular, double distanceToTargetRadians){
        double angularPower;
        double shouldHavePower = (DECELERATION_DISTANCE_ANGULAR/distanceToTargetRadians)*
                (velocityAngular/MAX_MM_PER_SECOND_RADIANS);
        if (Math.abs(shouldHavePower) < 1){
            if(distanceToTargetRadians > 0){
                angularPower = 1;
            } else {
                angularPower = -1;
            }
        } else {
            angularPower = 0;
        }
        return angularPower;
    }


    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    void initializeIMU() {
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public double getVelocityX(){
        return currentX-previousX;
    }
    public double getVelocityY(){
        return currentY-previousY;
    }
    public double getAngularVelocity(){
        return currentAngle-previousAngle;
    }
    // Pinpoint is centered on the rotation
    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-22.0, -172.0, DistanceUnit.MM);

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}
