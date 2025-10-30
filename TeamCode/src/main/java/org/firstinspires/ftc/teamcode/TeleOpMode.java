package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Timer;
import java.util.concurrent.TimeUnit;

@TeleOp
public class TeleOpMode extends LinearOpMode {
    enum DriveMode {
        FIELD_RELATIVE,
        ROBOT_RELATIVE,
    }

    double currentX;
    double previousX;
    double currentY;
    double previousY;
    double currentAngle;
    double previousAngle;


    public static double ENCODER_PER_MM = (537.7*19.2)/((104)*Math.PI);

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    IMU imu;

    DriveMode driveMode = DriveMode.FIELD_RELATIVE;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    GoBildaPinpointDriver pinpoint;
    double loopsPerSecond;
    double loopTime;
    double previousTime;


    @Override
    public void runOpMode() {
        initAprilTagProcessor();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        initializeIMU();

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        configurePinpoint();

        while (!isStarted()) {
            telemetry.addData("Drive Mode", driveMode.name());
            telemetry.update();

            if (gamepad1.left_bumper) {
                driveMode = DriveMode.FIELD_RELATIVE;
            } else if (gamepad1.right_bumper) {
                driveMode = DriveMode.ROBOT_RELATIVE;
            }
        }

        while (opModeIsActive()) {

            double currentTime = System.currentTimeMillis();
            pinpoint.update();
            currentAngle = pinpoint.getHeading(AngleUnit.RADIANS);
            currentX = pinpoint.getPosX(DistanceUnit.MM);
            currentY = pinpoint.getPosY(DistanceUnit.MM);

            telemetryAprilTag();

            loopTime = currentTime-previousTime;

            telemetry.addData("currentTime", currentTime);
            telemetry.addData("previousTime", currentTime);

            telemetry.addData("pinpointX", pinpoint.getPosX(DistanceUnit.MM));

            telemetry.addData("loopTime", loopTime);
            loopsPerSecond = 1/(loopTime/1000);


            telemetry.addData("Status", "Running");
            telemetry.addData("Front left position", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front right position", frontRightDrive.getCurrentPosition());
            telemetry.addData("Back left position", backLeftDrive.getCurrentPosition());
            telemetry.addData("Back right position", backRightDrive.getCurrentPosition());
            telemetry.addData("loopsPerSecond", loopsPerSecond);

            telemetry.addData("XVelocity", getVelocityX()*loopsPerSecond);
            telemetry.addData("YVelocity", getVelocityY()*loopsPerSecond);
            telemetry.addData("AngularVelocity", getAngularVelocity()*loopsPerSecond);

            telemetry.update();

            pinpoint.update();
            currentAngle = pinpoint.getHeading(AngleUnit.RADIANS);
            currentX = pinpoint.getPosX(DistanceUnit.MM);
            currentY = pinpoint.getPosY(DistanceUnit.MM);

            double speedCap = 1;
            if (gamepad1.b) {
                speedCap = 0.5;
            }

            switch (driveMode) {
                case FIELD_RELATIVE:
                    driveFieldRelative(
                        -gamepad1.left_stick_y * speedCap,
                        gamepad1.left_stick_x * speedCap,
                        gamepad1.right_stick_x * speedCap
                    );
                    if (gamepad1.a){
                        imu.resetYaw();
                    }
                    break;
                case ROBOT_RELATIVE:
                    drive(
                            -gamepad1.left_stick_y * speedCap,
                            gamepad1.left_stick_x * speedCap,
                            gamepad1.right_stick_x * speedCap
                    );
                    break;
            }
            previousAngle = currentAngle;
            previousX = currentX;
            previousY = currentY;
            previousTime = currentTime;
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

    public void initAprilTagProcessor() {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() { //code I stole from the example... if it doesn't work, it's not my fault - Hendrix

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
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

    public double getVelocityX(){
        return currentX-previousX;
    }
    public double getVelocityY(){
        return currentY-previousY;
    }
    public double getAngularVelocity(){
        return currentAngle-previousAngle;
    }

}
