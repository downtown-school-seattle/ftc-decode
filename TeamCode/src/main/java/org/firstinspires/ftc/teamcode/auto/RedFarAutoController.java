package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Near Audience Auto", group = "team code")
public class RedFarAutoController extends AutoController {
    @Override
    AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }

    @Override
    public void goToShootingPos() {
        drive(1, 0, 0);
        sleep(1000);
        drive(0, 0, 0.5);
        sleep(150);
        stopDrive();
    }
}
