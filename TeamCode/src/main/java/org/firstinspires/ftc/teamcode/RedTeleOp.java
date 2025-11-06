package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red TeleOp", group = "Robot")
@Disabled
public class RedTeleOp extends TeleOpMode {
    public RedTeleOp() {
        this.allianceColor = AllianceColor.RED;
    }
}