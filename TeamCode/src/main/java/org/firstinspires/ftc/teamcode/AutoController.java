package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auto Controller v0.1", group = "team code")
public class AutoController extends OpMode {
    
    double xLocEstimate = 0; // Overall xloc best guess
    // 0 = back 1 = front

    double yLocEstimate = 0; // Overall yloc best guess
    // 0 = left 1 = right
    
    // % of world per second
    const double speed = 1; // TODO: calculate
    
    // % of full rotation per second
    const double rotateSpeed = 0.5;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    @Override
    public void init() {
        // Initialize the drive system variables.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                          leftDrive.getCurrentPosition(),
                          rightDrive.getCurrentPosition());
        telemetry.update();

        // // Wait for the game to start (driver presses START)
        // waitForStart();

        // // Step through each leg of the path,
        // // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        // encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        // encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        // telemetry.addData("Path", "Complete");
        // telemetry.update();
        // sleep(1000);  // pause to display final telemetry message.
        driveToPosition(0.5, 0.5);
    }

    private void driveToPosition(double xloc, double yloc) {
        driveForward(xloc - xLocEstimate);

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
        ElapsedTime runtime = new ElapsedTime.
        runtime.reset();
        double timeToGO = angle / rotateSpeed;
        set speeds(1, -1, -1, 1);
        while (runtime.seconds() < timeToGo) {}
        setSpeeds(0, 0, 0, 0);
    }

    private void setSpeeds(double fl, double fr, double bl, double br) {
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backRightDrive.setPower(br);
        backLeftDrive.setPower(bl);
    }

    @Override
    public void loop() {

    }


}
