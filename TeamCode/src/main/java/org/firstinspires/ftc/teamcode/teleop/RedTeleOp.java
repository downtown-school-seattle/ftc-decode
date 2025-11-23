package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotController;

@TeleOp(name = "Red TeleOp", group = "Robot")
@Disabled
public class RedTeleOp extends TeleOpMode {
    public RedTeleOp() {
        this.allianceColor = RobotController.AllianceColor.RED;
    }
}