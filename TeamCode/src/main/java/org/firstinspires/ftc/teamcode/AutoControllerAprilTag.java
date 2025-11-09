package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public abstract class AutoControllerAprilTag extends RobotController {

    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera

    private int DESIRED_TAG_ID;     // Choose the tag you want to approach or set to -1 for ANY tag.


    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    public static final double SHOOTING_ARM_POS_DORMANT = 0.8;
    public static final double SHOOTING_ARM_LAUNCH_BALL_3 = 0.2;
    public static final double SHOOTING_ARM_LAUNCH_BALL_2 = 0.4;
    public static final double SHOOTING_ARM_LAUNCH_BALL_1 = 0.6;

    public static final int LAUNCH_BALL_PITCH = 150;


    static final double ENCODER_PER_MM = (537.7*19.2)/(104*Math.PI);

    // % of full rotation per second
    static final double ROTATE_SPEED = 0.5;

    // mm
    static final double BRAKE_THRESHOLD = 10;
    static final double ROTATE_THRESHOLD = Math.toRadians(5);
    // Distance in mm it takes the robot to stop
    static final double FORWARD_POWER_RAMP = 200;
    // Distance in radians it takes the robot to stop
    static final double ROTATION_POWER_RAMP = Math.toRadians(20);


//    double xwidth = 440;
//    double ywidth = 451.7;

    Obelisk detectedObelisk = Obelisk.PPG;

    private AprilTagProcessor aprilTagProcessor;

    abstract AllianceColor getAllianceColor();

    @Override
    public void runOpMode() {
        if (getAllianceColor() == AllianceColor.BLUE) {
            DESIRED_TAG_ID = 20;
        } else {
            DESIRED_TAG_ID = 24;
        }
        initRobot();

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();



        telemetry.addLine("Make sure to rotate the robot to face the wall with the goals on them.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                if (rangeError < 1 && yawError < 1 && headingError < 1){
                    shoot(3);
                }


                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                moveRobot(drive, strafe, turn);
                sleep(10);
            }
        }



//        moveForward(500);
//        turnTo(45);
//
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


    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }




    public void shoot(int balls) {
        // TODO: Shooting mechanism.
        rampPitch.setTargetPosition(LAUNCH_BALL_PITCH);
        sleep(1000);
        leftIntake.setPower(1);
        rightIntake.setPower(1);
        int i = 0;
        while(true){
            // main loop


            sleep(500);
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
                sleep(500);
                break;
            }
        }
        shootingArm.setPosition(SHOOTING_ARM_POS_DORMANT);
    }

    public void moveForward(double xTarget) {
        xTarget += pinpoint.getPosX(DistanceUnit.MM);

        double xDistance;
        do {
            xDistance = Math.abs(xTarget - pinpoint.getPosX(DistanceUnit.MM));
            drive(powerModulate(xDistance, FORWARD_POWER_RAMP), 0, 0);
        } while (xDistance > BRAKE_THRESHOLD);
    }

    public void turnTo(double rotTarget) {
        double rotDistance;
        do {
            rotDistance = Math.abs(rotTarget - imu.getRobotYawPitchRollAngles().getYaw());
            drive(0, 0, powerModulate(rotDistance, ROTATION_POWER_RAMP));
        } while (rotDistance > ROTATE_THRESHOLD);
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


    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
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
