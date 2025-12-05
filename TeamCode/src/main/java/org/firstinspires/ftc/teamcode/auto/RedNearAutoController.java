package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Near", group = "team code")
public class RedNearAutoController extends AutoController {
    @Override
    AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }

    @Override
    public void goToShootingPos() {
        drive(-0.5, 0, 0);
        sleep(1200);
        stopDrive();
    }
}
