package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Auto", group = "team code")
public class BlueAutoController extends AutoControllerAprilTag {
    @Override
    AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
}
