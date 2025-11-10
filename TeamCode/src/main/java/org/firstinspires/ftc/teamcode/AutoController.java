package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public abstract class AutoController extends RobotController {
    public static final double SHOOTING_ARM_LAUNCH_BALL_3 = 0.2;
    public static final double SHOOTING_ARM_LAUNCH_BALL_2 = 0.2;
    public static final double SHOOTING_ARM_LAUNCH_BALL_1 = 0.4;

    public static final int LAUNCH_BALL_PITCH = -2800;


    static final double ENCODER_PER_MM = (537.7*19.2)/(104*Math.PI);

    // % of full rotation per second
    static final double ROTATE_SPEED = 0.5;

    // mm
    static final double BRAKE_THRESHOLD = 10;
    static final double ROTATE_THRESHOLD = Math.toRadians(5);
    // Distance in mm it takes the robot to stop
    static final double FORWARD_POWER_RAMP = 800;
    // Distance in radians it takes the robot to stop
    static final double ROTATION_POWER_RAMP = Math.toRadians(40);


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
            telemetry.addLine("Delay for " + waitTime + "ms (L Stick to change)");
            telemetry.update();

            waitTime += (long) Math.floor(gamepad1.left_stick_y * 10);
        }

        sleep(waitTime);

//        moveForward(-1400);
        moveForward(400);
//        stopDrive();
//
//        shoot(2);
//
//        if (getAllianceColor() == AllianceColor.BLUE) {
//            turnTo(Math.toRadians(-45));
//        } else {
//            turnTo(Math.toRadians(45));
//        }
//
//        moveForward(500);
//        stopDrive();

//        shoot(3);
//
//        double xTarget = detectedObelisk.patternPickUpLocation;
//        double yTarget = 254;
//
//        if (getAllianceColor() == AllianceColor.BLUE) {
//            yTarget *= -1;
//        }
//
//        goToTarget(xTarget, 0, Math.toRadians(90));
//        goToTarget(xTarget, yTarget, Math.toRadians(90));
//        goToTarget(xTarget, 0, Math.toRadians(90));
//
//        goToTarget(0, 0, -Math.toRadians(-15));
//        shoot(3);
    }


    public void shoot(int balls) {
        if (balls > 3) throw new Error("The robot can't hold more than 3 balls.");

        rampPitch.setTargetPosition(LAUNCH_BALL_PITCH);
        sleep(3000);
        leftIntake.setPower(1);
        rightIntake.setPower(-1);
        sleep(1000);

        int i = 0;
        while(true) {
            // main loop


            sleep(1000);
            double shootingArmPos;
            if (i==0){
                shootingArmPos = SHOOTING_ARM_LAUNCH_BALL_1;
            } else if (i==1) {
                shootingArmPos = SHOOTING_ARM_LAUNCH_BALL_2;
            } else {
                shootingArmPos = SHOOTING_ARM_LAUNCH_BALL_3;
            }

            shootingArm.setPosition(shootingArmPos);

            i++;
            if (i >= balls){
                sleep(1000);
                break;
            }
        }
        shootingArm.setPosition(SHOOTING_ARM_POS_DORMANT);
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
        } while (Math.abs(xDistance) > BRAKE_THRESHOLD);
    }

    public void turnTo(double rotTarget) {
        rotTarget += imu.getRobotYawPitchRollAngles().getYaw();
        double rotDistance;
        do {
            pinpoint.update();
            rotDistance = AngleUnit.normalizeRadians(rotTarget - imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("turn distance", rotDistance);
            telemetry.update();

            drive(0, 0, powerModulate(rotDistance, ROTATION_POWER_RAMP));
        } while (Math.abs(rotDistance) > ROTATE_THRESHOLD);
    }

    public void goToTarget(double xTarget, double yTarget, double rotTarget) {
        double xDistance;
        double yDistance = 0;
        double rotDistance;

        do {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            xDistance = xTarget - pose.getX(DistanceUnit.MM);
//            yDistance = yTarget - pose.getY(DistanceUnit.MM); // The y-pod is not working.
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
//                        powerModulate(-yDistance, FORWARD_RAMP_DISTANCE), // GoBuilda uses an inverted y-axis.
                        0, // The y pod is not working.
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
            if (detection.id == 21) {
                detectedObelisk = Obelisk.PPG;
                telemetry.addData("Obelisk", "PPG");
            } else if (detection.id == 22) {
                detectedObelisk = Obelisk.PGP;
                telemetry.addData("Obelisk", "PGP");
            } else if (detection.id == 23) {
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
