package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Pattern Driving", group = "labs")
public class PatternDriving extends AutoCommon {

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        waitForStart();
        drive(.5,50);
        
        finalLab(.15);
        // test turnIMU()
        /*turnIMU(0.5, 90);
        sleep(1000);   // let robot settle before taking a final heading
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.5, -90);
        sleep(1000);
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.5, 45);
        sleep(1000);
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.5, -45);
        sleep(1000);
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.5, -175);
        sleep(1000);   // let robot settle before taking a final heading
        System.out.println("TURNIMU: " + getHeading());

        turnIMU(0.5, 175);
        sleep(1000);   // let robot settle before taking a final heading
        System.out.println("TURNIMU: " + getHeading()); */

        //driveIMU(.5,400);
        //driveUntilTouch(.5);

        //driveToCalibrateLightSensor();
        //sleep(200);
        //int dis = countingLines();
        //sleep(200);
        //int cm = robot.convertTicksToDistance(dis) + 20;
        //drive(.5, -cm);
        //waitForStart();


        //drive(.5,150);
        //sleep(200);
        //drive(.5,-50);
        //sleep(200);
        //turn(.5,90);
        ///sleep(200);
        ///drive(.5,100);
        //sleep(200);
        //turn(.5,-70);
        //sleep(200);
        //drive(.5,-106.42);
        //sleep(200);
        //turn(.5,250);
        //sleep(200);
        //drive(.5,63.6);
        //sleep(200);
        //turn(.5,90);
        //driveForTime(0.5,4200);
        //driveForTime(-0.5, 1400);
        //turnForTime(0.5,1000);
        //driveForTime(.5,2800);
        //turnForTime(-.5,850);
        //driveForTime(-.5,3000);
        //turnForTime(.5,2700);
        //driveForTime(.5, 1800);
        //turnForTime(0.5,1000);
    }
}
