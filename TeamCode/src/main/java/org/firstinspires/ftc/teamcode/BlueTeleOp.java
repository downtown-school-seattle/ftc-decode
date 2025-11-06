package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue TeleOp", group = "Robot")
@Disabled
public class BlueTeleOp extends TeleOpMode {
    public BlueTeleOp() {
        this.allianceColor = AllianceColor.BLUE;
    }
}