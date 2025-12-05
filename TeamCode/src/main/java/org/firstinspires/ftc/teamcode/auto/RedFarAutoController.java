package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Far", group = "team code")
public class RedFarAutoController extends AutoController {
    @Override
    AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }

    @Override
    public void goToShootingPos() {
        drive(.5, 0, 0);
        sleep(2200);
        drive(0, 0, 0.5);
        sleep(150);
        drive(0.5, 0, 0);
        sleep(200);
        stopDrive();
    }
}
