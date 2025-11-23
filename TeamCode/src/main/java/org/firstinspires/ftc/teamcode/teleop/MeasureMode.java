package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Measure mode", group = "helper")
public class MeasureMode extends TeleOpMode {

    @Override
    public void controller(double delta) {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        telemetry.addData("x pos", pose.getX(DistanceUnit.MM));
        telemetry.addData("y pos", pose.getY(DistanceUnit.MM));
        telemetry.addData("heading (yaw)", pose.getHeading(AngleUnit.RADIANS));
        super.controller(delta);
    }

}
