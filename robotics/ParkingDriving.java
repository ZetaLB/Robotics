package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Parking Driving", group = "labs")
public class ParkingDriving extends AutoCommon {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        //while (opModeIsActive()) {
            //tester();
        //}
        white(.3);
        sleep(200);
        parell(.3);
    }



    public void tester(){
        telemetry.addData("distance:", robot.distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
    public void white(double power){
        double start = getHeading();
        driveToCalibrateLightSensor();
        double ticks = robot.convertDistanceToTicks(20);
        int cur = 0;
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);
        while (cur == 0 && opModeIsActive()){
            if( robot.colorSensor.alpha() > (robot.maxBrightness - 1000)) {
                robot.motorLeft.setPower(0);
                robot.motorRight.setPower(0);
                cur = 1;
            }
        }
        sleep(200);
        turnIMU(.5,90);
    }
    public void parell(double power){
        double distance = robot.distanceSensor.getDistance(DistanceUnit.CM);
        int cur = 0;
        double start = robot.motorLeft.getCurrentPosition();
        double finish = 0;
        double curStart = 0;
        double curEnd = 0;
        double curMid = 0;
        double bestStart = 0;
        double bestEnd =0;
        double bestMid = 10000;
        int ind = 0;

        //int fail = 0;
        robot.resetDriveEncoders();
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);
        while (cur != 2 && opModeIsActive()){
            telemetry.addData("dis:", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("end:", bestEnd);
            telemetry.addData("mid:", bestMid);
            telemetry.update();
            if(robot.distanceSensor.getDistance(DistanceUnit.CM) > distance + 10 && cur == 0){
                start = robot.motorLeft.getCurrentPosition();
                curStart = start;
                cur = 1;
            }
            if (robot.distanceSensor.getDistance(DistanceUnit.CM) < distance + 10 && robot.distanceSensor.getDistance(DistanceUnit.CM) > distance - 10 && cur == 1){
                finish = robot.motorLeft.getCurrentPosition();
                curEnd = finish;
                curMid = (curEnd - curStart) / 2;
                telemetry.addData("end:", curEnd);
                telemetry.addData("mid:", curMid);
                telemetry.update();
                if (curMid >= robot.convertDistanceToTicks(22)){
                    telemetry.addData("end:", curEnd);
                    telemetry.addData("mid:", curMid);
                    telemetry.update();
                    if (curMid < bestMid){
                        bestMid = curMid;
                        bestStart = curStart;
                        bestEnd = curEnd;
                    }
                }
                //robot.motorLeft.setPower(0);
                //robot.motorRight.setPower(0);
                cur = 0;
            }
            if( robot.colorSensor.alpha() > (robot.maxBrightness - 1000)) {
                robot.motorLeft.setPower(0);
                robot.motorRight.setPower(0);
                //fail = 1;
                cur = 2;
            }

        }
        double back = robot.motorLeft.getCurrentPosition() - bestEnd;
        telemetry.addData("end:", curEnd);
        telemetry.addData("mid:", curMid);
        telemetry.update();
        sleep(200);
        double cmEnd = robot.convertTicksToDistance(back);
        robot.resetDriveEncoders();
        bestMid = robot.convertTicksToDistance(bestMid);
        drive(-power,-(cmEnd+5));
        telemetry.addData("end:", cmEnd);
        telemetry.addData("mid:", curMid);
        telemetry.update();
        sleep(200);
        if (bestMid == 10000) {
            turnIMU(power, -90);
            drive(-power, -40);
            //sleep(200);
            cur = 0;
            robot.resetDriveEncoders();
            robot.motorLeft.setPower(-power);
            robot.motorRight.setPower(-power);
            while (cur == 0 && opModeIsActive()) {
                if (robot.colorSensor.alpha() > (robot.maxBrightness - 1000)) {
                    robot.motorLeft.setPower(0);
                    robot.motorRight.setPower(0);
                    cur = 1;
                }
                robot.motorLeft.setPower(0);
                robot.motorRight.setPower(0);
            }
        }
        else {
            drive(-power,-(bestMid+5));
            sleep(200);
            robot.resetDriveEncoders();
            turnIMU(power, -90);
            sleep(200);
            telemetry.addData("dis:", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            double dis = robot.distanceSensor.getDistance(DistanceUnit.CM);
            if (dis > 150){
                dis = 100;
            }
            drive(power,(dis));
            sleep(200);
            turnIMU(power, 90);
            sleep(200);
        }
    }
}