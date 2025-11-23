package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotController;

@TeleOp(name = "Blue TeleOp", group = "Robot")
@Disabled
public class BlueTeleOp extends TeleOpMode {
    public BlueTeleOp() {
        this.allianceColor = RobotController.AllianceColor.BLUE;
    }
}