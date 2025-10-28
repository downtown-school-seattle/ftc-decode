package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.function.ObjLongConsumer;

@Autonomous(name = "Auto Controller", group = "team code")
public class AutoController extends LinearOpMode {
    enum AllianceColor {
        RED,
        BLUE
    }

    enum Obelisk {
        PPG(914.4),
        PGP(1524),
        GPP(2133.6);

        public final double fieldPosition;

        Obelisk(double fieldPosition) {
            this.fieldPosition = fieldPosition;
        }
    }

    TeleOpMode.AllianceColor allianceColor = TeleOpMode.AllianceColor.RED;
    TeleOpMode.Obelisk obelisk = TeleOpMode.Obelisk.PPG;

    static final double ENCODER_PER_MM = (537.7*19.2)/(104*Math.PI);

    // % of full rotation per second
    static final double ROTATE_SPEED = 0.5;

    // mm
    static final double BRAKE_THRESHOLD = 10;
    static final double ROTATE_THRESHOLD = Math.toRadians(5);
    // Distance in mm it takes the robot to stop
    static final double FORWARD_RAMP_DISTANCE = 600;
    // Distance in radians it takes the robot to stop
    static final double TURN_RAMP_DISTANCE = Math.toRadians(20);


//    double xwidth = 440;
//    double ywidth = 451.7;

    Obelisk detectedObelisk = Obelisk.PPG;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    IMU imu;
    GoBildaPinpointDriver pinpoint;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        goToTarget(0, 0, -Math.toRadians(-15));
        shoot(3);

        double xTarget = detectedObelisk.fieldPosition;
        goToTarget(xTarget, 0, Math.toRadians(90));
        goToTarget(xTarget, -254, Math.toRadians(90));
        goToTarget(xTarget, 0, Math.toRadians(90));

        goToTarget(0, 0, -Math.toRadians(-15));
        shoot(3);
    }

    public void initRobot() {
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
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        configurePinpoint();

        telemetry.addLine("Make sure to rotate the robot to face the wall with the goals on them.");
        telemetry.addData("stop dist", FORWARD_RAMP_DISTANCE);
        telemetry.update();

    }

    public void shoot(int balls) {
        // TODO: Shooting mechanism.
        sleep(1000L * balls);
    }

    public void goToTarget(double xTarget, double yTarget, double rotTarget) {
        double xDistance;
        double yDistance;
        double rotDistance;

        do {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            xDistance = xTarget - pose.getX(DistanceUnit.MM);
            yDistance = yTarget - pose.getY(DistanceUnit.MM);
            rotDistance = AngleUnit.normalizeRadians(rotTarget - pose.getHeading(AngleUnit.RADIANS));

            telemetry.addData("x pos", pose.getX(DistanceUnit.MM));
            telemetry.addData("y pos", pose.getY(DistanceUnit.MM));
            telemetry.addData("heading (yaw)", pose.getHeading(AngleUnit.RADIANS));
            telemetry.addData("X dist", xDistance);
            telemetry.addData("Y dist", yDistance);
            telemetry.addData("rot dist", rotDistance);
            telemetry.update();

            if (Math.abs(xDistance) < BRAKE_THRESHOLD) {
                xDistance = 0;
            }
            if (Math.abs(yDistance) < BRAKE_THRESHOLD) {
                yDistance = 0;
            }
            if (Math.abs(rotDistance) < ROTATE_THRESHOLD) {
                rotDistance = 0;
            }

            if (xDistance == 0 && yDistance == 0 && rotDistance == 0) {
                telemetry.addLine("stopping");
            } else {
                driveFieldRelative(
                        powerModulate(xDistance, FORWARD_RAMP_DISTANCE),
//                    Math.signum(xDistance) * 0.3,
                        powerModulate(-yDistance, FORWARD_RAMP_DISTANCE), // GoBuilda uses an inverted y-axis.
//                    Math.signum(yDistance) * -0.3,
                        powerModulate(-rotDistance, TURN_RAMP_DISTANCE), // Drive function expects CW
//                    Math.signum(-rotDistance),
//                    0,
                        pose
                );
            }
        } while (opModeIsActive() && xDistance != 0 && yDistance != 0 && rotDistance != 0);
    }

    private double powerModulate(double distance, double stopDistance) {
        return Math.signum(distance) * Math.min(Math.abs(distance) / stopDistance, 1);
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate, Pose2D pose) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - pose.getHeading(AngleUnit.RADIANS));

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

    public void initAprilTagProcessor() {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 20 && allianceColor == TeleOpMode.AllianceColor.BLUE) {
                telemetry.addData("BLUE April Tag", "Detected");
            } else if (detection.id == 21) {
                obelisk = TeleOpMode.Obelisk.PPG;
                telemetry.addData("Obelisk", "PPG");
            } else if (detection.id == 22) {
                obelisk = TeleOpMode.Obelisk.PGP;
                telemetry.addData("Obelisk", "PGP");
            } else if (detection.id == 23) {
                obelisk = TeleOpMode.Obelisk.GPP;
                telemetry.addData("Obelisk", "GPP");
            } else if (detection.id == 24 && allianceColor == TeleOpMode.AllianceColor.RED) {
                telemetry.addData("RED April Tag", "Detected");
            } else {
                break;
            }
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");
        }
    }
}
