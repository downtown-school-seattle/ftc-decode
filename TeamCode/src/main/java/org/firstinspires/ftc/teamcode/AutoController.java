package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Auto Controller v0.1", group = "team code")
public class AutoController extends OpMode {
    
    double xLocEstimate = 0; // Overall xloc best guess
    // 0 = back 1 = front

    double yLocEstimate = 0; // Overall yloc best guess
    // 0 = left 1 = right

    static final double ENCODER_PER_MM = (537.7*19.2)/(104*Math.PI);

    // % of full rotation per second
    static final double ROTATE_SPEED = 0.5;

    // mm
    static final double BRAKE_THRESHOLD = 10;
    static final double ROTATE_THRESHOLD = 10;


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
    GoBildaPinpointDriver pinpointDriver;

    double targetX = 100;
    double targetY = 0;
    double targetRadians = 0;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        imu = hardwareMap.get(IMU.class, "imu");
        initializeIMU();

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pinpointDriver.resetPosAndIMU();

        telemetry.addLine("Make sure to rotate the robot to face the wall with the goals on them.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double xDistance = targetX - pinpointDriver.getPosY(DistanceUnit.MM);
        double yDistance = targetY - pinpointDriver.getPosX(DistanceUnit.MM);
        double rotDistance = targetRadians - pinpointDriver.getHeading(AngleUnit.DEGREES);

        if (Math.abs(xDistance) + Math.abs(yDistance) < BRAKE_THRESHOLD * 2 && Math.abs(rotDistance) < ROTATE_THRESHOLD) {
            // TODO: update targets
        } else {
            driveFieldRelative(Math.signum(yDistance), Math.signum(xDistance), Math.signum(rotDistance));
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
}
