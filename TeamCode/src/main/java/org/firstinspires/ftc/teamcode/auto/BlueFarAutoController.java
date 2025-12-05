package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Far", group = "team code")
public class BlueFarAutoController extends AutoController {
    @Override
    AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }

    @Override
    public void goToShootingPos() {
        drive(.5, 0, 0);
        sleep(2200);
        drive(0, 0, -0.5);
        sleep(150);
        drive(0.5, 0, 0);
        sleep(200);
        stopDrive();
    }
}
