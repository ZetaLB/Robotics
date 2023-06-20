package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Line Driving", group = "labs")
public class lineDriving extends LineFollower {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        calibrateColorSensor();
        sleep(200);
        System.out.println("max" + robot.maxBrightness);
        System.out.println("min " + robot.minBrightness);
        drivePID(.25);

    }
}