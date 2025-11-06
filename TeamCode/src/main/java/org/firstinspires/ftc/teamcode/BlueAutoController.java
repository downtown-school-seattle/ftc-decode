package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Auto", group = "team code")
public class BlueAutoController extends AutoController {
    @Override
    AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
}
