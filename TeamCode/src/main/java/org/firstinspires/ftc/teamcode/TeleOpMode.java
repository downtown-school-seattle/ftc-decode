package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class TeleOpMode extends RobotController {
    public static final double RAMP_GEAR_RATIO = 1.0 / 64.0;
    public static final double SHOOTING_ARM_POS_DORMANT = 0.8;
    public static final double SHOOTING_ARM_POS_ACTIVE = 0.2;
    public static final double INTAKE_POWER = 0.3;
    public static final double SHOOT_POWER = 1;
    public static final double RAMP_MAX = 0;
    public static final double RAMP_MIN = -44;
    public static final double RAMP_SPEED = 2;

    enum DriveMode {
        FIELD_RELATIVE,
        ROBOT_RELATIVE,
        DONT_MOVE
    }

    AllianceColor allianceColor = AllianceColor.RED;
    Obelisk obelisk = Obelisk.PPG;

    public static double ENCODER_PER_MM = (537.7*19.2)/((104)*Math.PI);

    DriveMode driveMode = DriveMode.FIELD_RELATIVE;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    MechOption mechOption = MechOption.INTAKE_MECH;

    double rampPos = 0;

    @Override
    public void runOpMode() {
        initAprilTagProcessor();

        initRobot();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStarted()) {
            telemetry.addData("Drive Mode", driveMode.name());
            telemetry.update();

            if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.back) {
                driveMode = DriveMode.DONT_MOVE;
            } else if (gamepad1.left_bumper) {
                driveMode = DriveMode.FIELD_RELATIVE;
            } else if (gamepad1.right_bumper) {
                driveMode = DriveMode.ROBOT_RELATIVE;
            }
        }

        switchMechanism(mechOption);
        leftIntake.setPower(INTAKE_POWER);
        rightIntake.setPower(INTAKE_POWER);

        while (opModeIsActive()) {
            telemetryAprilTag();

            telemetry.addData("Status", "Running");
            telemetry.addData("Front left position", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front right position", frontRightDrive.getCurrentPosition());
            telemetry.addData("Left spinny", leftIntake.getDirection());
            telemetry.addData("Right spinny", rightIntake.getDirection());
            telemetry.addData("Back left position", backLeftDrive.getCurrentPosition());
            telemetry.addData("Back right position", backRightDrive.getCurrentPosition());
            telemetry.addData("Ramp lift position", rampPitch.getCurrentPosition());
            telemetry.addData("Ramp lift target", rampPos);
            telemetry.addData("Shooty target", shootingArm.getPosition());

            telemetry.update();

            double speedCap = 1;
            if (gamepad1.b) {
                speedCap = 0.5;
            }

            updateRampPitch();

            if (gamepad1.leftBumperWasPressed()) {
                if (mechOption == MechOption.INTAKE_MECH) {
                    mechOption = MechOption.SHOOTING_MECH;
                } else {
                    mechOption = MechOption.INTAKE_MECH;
                }
                switchMechanism(mechOption);
            }

            if (gamepad1.rightBumperWasPressed() || gamepad1.aWasPressed()) {
                shootingArm.setPosition(SHOOTING_ARM_POS_ACTIVE);
            } else if (gamepad1.rightBumperWasReleased() || gamepad1.aWasReleased()) {
                shootingArm.setPosition(SHOOTING_ARM_POS_DORMANT);
            }


            switch (driveMode) {
                case FIELD_RELATIVE:
                    driveFieldRelative(
                        -gamepad1.left_stick_y * speedCap,
                        gamepad1.left_stick_x * speedCap,
                        gamepad1.right_stick_x * speedCap
                    );
                    if (gamepad1.backWasPressed()){
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
                case DONT_MOVE:
                    // Dont move :)
                    break;
                default:
                    throw new RuntimeException("Unreachable code! You need to add a drive mode handler.");
            }
        }
    }

    public void updateRampPitch() {
        rampPos += (int) gamepad1.right_stick_y * RAMP_SPEED;
        rampPos = Math.max(Math.min(rampPos, RAMP_MAX), RAMP_MIN);

        rampPitch.setTargetPosition((int) rampPos);
    }

    public void switchMechanism(MechOption current) {
        double power;
        switch (current){
            case SHOOTING_MECH:
                leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
//                rampPitch.setDirection(DcMotorSimple.Direction.REVERSE);
//                rampPitch.setPower(power);
                power = SHOOT_POWER;
                break;
            case INTAKE_MECH:
                leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
                rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
//                rampPitch.setDirection(DcMotorSimple.Direction.FORWARD);
//                rampPitch.setPower(power);
                power = INTAKE_POWER;
                break;
            default:
                throw new RuntimeException("Unreachable! You need to add a switchMechanism handler.");
        }

        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                pinpoint.getHeading(AngleUnit.RADIANS));

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
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

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
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 20 && allianceColor == AllianceColor.BLUE) {
                telemetry.addData("BLUE April Tag", "Detected");
            } else if (detection.id == 21) {
                obelisk = Obelisk.PPG;
                telemetry.addData("Obelisk", "PPG");
            } else if (detection.id == 22) {
                obelisk = Obelisk.PGP;
                telemetry.addData("Obelisk", "PGP");
            } else if (detection.id == 23) {
                obelisk = Obelisk.GPP;
                telemetry.addData("Obelisk", "GPP");
            } else if (detection.id == 24 && allianceColor == AllianceColor.RED) {
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
