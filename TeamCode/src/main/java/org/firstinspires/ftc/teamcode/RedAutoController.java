package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Auto", group = "team code")
public class RedAutoController extends AutoController {
    @Override
    AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}
