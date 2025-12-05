package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public abstract class AutoController extends RobotController {
    public static final double SHOOTING_ARM_LAUNCH_BALL_2 = 0.4;
    public static final double SHOOTING_ARM_LAUNCH_BALL_1 = 0.7;

    public static final int LAUNCH_BALL_PITCH = -1300;

    static final double ENCODER_PER_MM = (537.7*19.2)/(104*Math.PI);

    // % of full rotation per second
    static final double ROTATE_SPEED = 0.5;

    // mm
    static final double BRAKE_THRESHOLD = 10;
    static final double ROTATE_THRESHOLD = Math.toRadians(5);
    // Distance in mm it takes the robot to stop
    static final double FORWARD_POWER_RAMP = 400;
    // Distance in radians it takes the robot to stop
    static final double ROTATION_POWER_RAMP = Math.toRadians(40);
    // Minimum power given to the robot.
    static final double MIN_POWER = 0.2;


//    double xwidth = 440;
//    double ywidth = 451.7;

    Obelisk detectedObelisk = Obelisk.PPG;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    long waitTime;

    abstract AllianceColor getAllianceColor();

    @Override
    public void runOpMode() {
        initAprilTagProcessor();

        initRobot();
        telemetry.addLine("Make sure to rotate the robot to face the wall with the goals on them.");
        telemetry.update();

        while (!this.isStarted()) {
            telemetry.addLine("Delay for " + waitTime + "ms (Bumpers to change)");
            telemetry.update();

            if (gamepad1.leftBumperWasPressed()) waitTime += 1000;
            else if (gamepad1.rightBumperWasPressed()) waitTime -= 1000;
        }

//        goToTarget(500, 0, 90);

        sleep(waitTime);

        goToShootingPos();
        shoot(3);
        if (getAllianceColor() == AllianceColor.RED) drive(1, 0, -0.4);
        else drive(1, 0, 0.4);
        sleep(800);
        stopDrive();

//        sleep(waitTime);
//
//        goToShootingPos();
//        shoot(3);
//
//        Pose2D pose = pinpoint.getPosition();
//        double angle;
//        switch (getAllianceColor()) {
//            case RED:
//                angle = 45;
//                break;
//            case BLUE:
//                angle = -45;
//                break;
//            default:
//                throw new Error("Unreachable! no alliance handler defined for auto-init");
//        }
//        goToTarget(pose.getX(DistanceUnit.MM), pose.getY(DistanceUnit.MM), angle);
//
//        pinpoint.resetPosAndIMU();
//        goToTarget(-500, 0, 90);
    }

    abstract public void goToShootingPos();

    public void shoot(int balls) {
        if (balls > 3) throw new Error("The robot can't hold more than 3 balls.");

        rampPitch.setTargetPosition(LAUNCH_BALL_PITCH);
        leftIntake.setPower(-0.5);
        rightIntake.setPower(0.5);
        sleep(3000);
        leftIntake.setPower(0.9);
        rightIntake.setPower(-0.9);

        sleep(2000);
        int i = 0;
        while (opModeIsActive()) {
            // main loop


            sleep(2000);
            double shootingArmPos;
            switch (balls - i) {
                case 1: shootingArmPos = SHOOTING_ARM_POS_ACTIVE; break;
                case 2: shootingArmPos = SHOOTING_ARM_LAUNCH_BALL_2; break;
                case 3: shootingArmPos = SHOOTING_ARM_LAUNCH_BALL_1; break;
                default: throw new Error("unreachable code: tried to shoot more than 3 balls.");
            }

            shootingArm.setPosition(shootingArmPos);

            i++;
            if (i >= balls){
                sleep(1000);
                break;
            }
        }
        shootingArm.setPosition(SHOOTING_ARM_POS_DORMANT);
        rampPitch.setTargetPosition(0);
    }

    public void moveForward(double xTarget) {
        pinpoint.update();
        xTarget += pinpoint.getPosX(DistanceUnit.MM);

        double xDistance;
        do {
            pinpoint.update();
            xDistance = xTarget - pinpoint.getPosX(DistanceUnit.MM);
            telemetry.addData("x position", pinpoint.getPosX(DistanceUnit.MM));
            telemetry.addData("x distance", xDistance);
            telemetry.update();

            drive(powerModulate(xDistance, FORWARD_POWER_RAMP), 0, 0);
        } while (Math.abs(xDistance) > BRAKE_THRESHOLD && opModeIsActive());
    }

    public void turnBy(double rotTarget) {
        rotTarget += AngleUnit.normalizeRadians(pinpoint.getHeading(AngleUnit.RADIANS));
        double rotDistance;

        do {
            pinpoint.update();

            rotDistance = AngleUnit.normalizeRadians(rotTarget - pinpoint.getHeading(AngleUnit.RADIANS));
            telemetry.addData("turn distance", rotDistance);
            telemetry.update();

            if (Math.abs(rotDistance) < ROTATE_THRESHOLD) {
                rotDistance = 0;
            }

            drive(0, 0, powerModulate(-rotDistance, ROTATION_POWER_RAMP));
        } while (opModeIsActive() && rotDistance != 0);
    }

    public void goToTarget(double xTarget, double yTarget, double rotTarget) {
        double xDistance;
        double yDistance;
        double rotDistance;

        do {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            telemetryAprilTag();

            xDistance = xTarget - pose.getX(DistanceUnit.MM);
            yDistance = yTarget - pose.getY(DistanceUnit.MM);
            rotDistance = AngleUnit.normalizeRadians(rotTarget - pose.getHeading(AngleUnit.RADIANS));

            telemetry.addData("x pos", pose.getX(DistanceUnit.MM));
            telemetry.addData("y pos", pose.getY(DistanceUnit.MM));
            telemetry.addData("heading (yaw)", pose.getHeading(AngleUnit.RADIANS));
            telemetry.addData("X dist", xDistance);
            telemetry.addData("Y dist", yDistance);
            telemetry.addData("rot dist", rotDistance);

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
                        powerModulate(xDistance, FORWARD_POWER_RAMP),
//                    Math.signum(xDistance) * 0.3,
                        powerModulate(-yDistance, FORWARD_POWER_RAMP), // GoBuilda uses an inverted y-axis.
//                    Math.signum(yDistance) * -0.3,
                        powerModulate(-rotDistance, ROTATION_POWER_RAMP), // Drive function expects CW
//                    Math.signum(-rotDistance),
//                    0,
                        pose
                );
            }

            telemetry.update();
        } while (opModeIsActive() && (xDistance != 0 || yDistance != 0 || rotDistance != 0));
    }

    private double powerModulate(double distance, double stopDistance) {
        return Math.signum(distance) * Math.max(Math.min(Math.abs(distance) / stopDistance, 1), MIN_POWER);
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

    public void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
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
            if (detection.id == 23) {
                detectedObelisk = Obelisk.PPG;
                telemetry.addData("Obelisk", "PPG");
            } else if (detection.id == 22) {
                detectedObelisk = Obelisk.PGP;
                telemetry.addData("Obelisk", "PGP");
            } else if (detection.id == 21) {
                detectedObelisk = Obelisk.GPP;
                telemetry.addData("Obelisk", "GPP");
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
