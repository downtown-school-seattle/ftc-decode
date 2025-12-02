package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Near Goal Auto", group = "team code")
public class RedNearAutoController extends AutoController {
    @Override
    AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }

    @Override
    public void goToShootingPos() {
        drive(-1, 0, 0);
        sleep(500);
        stopDrive();
    }
}
