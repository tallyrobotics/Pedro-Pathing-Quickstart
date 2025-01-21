package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(group = "Auto JTracking")
public class JATest extends LinearOpMode {
    JArm armPID;

    @Override
    public void runOpMode() {
        armPID = new JArm(this, hardwareMap);
        armPID.run();

        while (opModeIsActive()) {
            armPID.setTarget(0.95);
            sleep(2000);
            armPID.setTarget(1.20);
            sleep(2000);
            armPID.setTarget(0.70);
            while (opModeIsActive()) {
                sleep(1000);
            }
        }
    }
}
