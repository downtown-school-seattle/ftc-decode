package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Reset Ramp", group = "helper")
public class ResetRamp extends LinearOpMode {
    DcMotor rampPitch;

    @Override
    public void runOpMode() {
        rampPitch = hardwareMap.get(DcMotor.class, "ramp_pitch");

        waitForStart();

        rampPitch.setPower(1);
        sleep(500L);
        rampPitch.setPower(0);
    }
}
