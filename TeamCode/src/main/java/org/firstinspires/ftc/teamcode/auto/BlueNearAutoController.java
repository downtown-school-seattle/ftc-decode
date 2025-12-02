package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Near Goal Auto", group = "team code")
public class BlueNearAutoController extends AutoController {
    @Override
    AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }

    @Override
    public void goToShootingPos() {
        drive(-.7, 0, 0);
        sleep(400);
        stopDrive();
    }
}
