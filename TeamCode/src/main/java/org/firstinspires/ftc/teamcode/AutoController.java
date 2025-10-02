package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Controller v0.1", group = "team code")
public class AutoController extends LinearOpMode {
    
    double xLocEstimate = 0; // Overall xloc best guess
    // 0 = back 1 = front

    double yLocEstimate = 0; // Overall yloc best guess
    // 0 = left 1 = right
    
    // % of world per second
    static final double speed = 1; // TODO: calculate
    
    // % of full rotation per second
    static final double rotateSpeed = 0.5;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    IMU imu;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        imu = hardwareMap.get(IMU.class, "imu");
        initializeIMU();

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                frontLeftDrive.getCurrentPosition(),
                frontRightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // telemetry.addData("Path", "Complete");
        // telemetry.update();
        // sleep(1000);  // pause to display final telemetry message.
        driveToPosition(0.5, 0.5);
    }

    private void driveToPosition(double xloc, double yloc) {
        driveForward(xloc - xLocEstimate);
        // TODO: Use yloc
    }

    private void driveForward(double xdist) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        double timeToGo = xdist / speed;
        setSpeeds(1, 1, 1, 1);
        while (runtime.seconds() < timeToGo) {}
        setSpeeds(0, 0, 0, 0);
    }

    // angles are in % of full rotation
    private void rotateAngle(double angle) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        double timeToGo = angle / rotateSpeed;
        setSpeeds(1, -1, -1, 1);
        while (runtime.seconds() < timeToGo) {}
        setSpeeds(0, 0, 0, 0);
    }

    private void setSpeeds(double fl, double fr, double bl, double br) {
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backRightDrive.setPower(br);
        backLeftDrive.setPower(bl);
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
