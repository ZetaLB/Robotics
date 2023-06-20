package edu.elon.robotics;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LineFollower extends AutoCommon {
    
    public void calibrateColorSensor() {
        robot.resetDriveEncoders();
        resetAngle();

        robot.resetDriveEncoders();

        robot.motorLeft.setPower(-.3);
        robot.motorRight.setPower(.3);

        int min = 1000000000;
        int max = 0;

        while (Math.abs(getHeading()) < 45 && opModeIsActive()) {
            int cur = robot.colorSensor.alpha();
            if (cur < min){
                min = cur;
            }
            if (cur > max){
                max = cur;
            }
            System.out.println(getHeading());
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);

        resetAngle();

        robot.motorRight.setPower(-.3);
        robot.motorLeft.setPower(.3);
        while (Math.abs(getHeading()) < 90 && opModeIsActive()) {
            int cur = robot.colorSensor.alpha();
            if (cur < min){
                min = cur;
            }
            if (cur > max){
                max = cur;
            }
            System.out.println(getHeading());
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);

        resetAngle();

        robot.motorRight.setPower(.3);
        robot.motorLeft.setPower(-.3);
        while (Math.abs(getHeading()) < 45 && opModeIsActive()) {
            int cur = robot.colorSensor.alpha();
            if (cur < min){
                min = cur;
            }
            if (cur > max){
                max = cur;
            }
            System.out.println(getHeading());
        }

        //robot.resetDriveEncoders();
        robot.maxBrightness = max;
        robot.minBrightness = min;

        robot.midBrightness = (max + min) / 2;

        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);

        System.out.println("max: " + max);
        System.out.println("set max: " + robot.maxBrightness);
        System.out.println("min: " + min);
        System.out.println("set min: " + robot.minBrightness);
    }

    public void pController(double power){
        robot.resetDriveEncoders();

        double absPower = Math.abs(power);

        robot.resetDriveEncoders();

        robot.motorLeft.setPower(absPower);
        robot.motorRight.setPower(absPower);
        double error;
        double kp = 0.00001433;
        double powerOffset;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()){
            error = robot.midBrightness - robot.colorSensor.alpha();
            //System.out.println(error);
            /*if (error < 150 && error > -150){
                error = 0;
            }*/
            powerOffset = (kp*error);
            robot.motorLeft.setPower(absPower + powerOffset);
            robot.motorRight.setPower(absPower - powerOffset);

            double value = robot.colorSensor.alpha();
            System.out.println("P-CONTROL: " + timer.milliseconds() + ", " + value);
        }
        robot.stopDriveMotors();
    }

    public void drivePID (double power) {
        double kp = .000085;
        double kc = .0000266;
        double ki = 0.00001269;
        double kd = 0.0005596;
        double error;
        double powerOffset;
        double sumError = 0.0;
        double diffError = 0.0;
        double prevError = 0.0;
        robot.resetDriveEncoders();

        double absPower = Math.abs(power);

        robot.resetDriveEncoders();

        robot.motorLeft.setPower(absPower);
        robot.motorRight.setPower(absPower);
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()){
            loopTimer.reset();
            error = robot.midBrightness - robot.colorSensor.alpha();

            sumError = (0.8*(sumError)) + error;
            diffError = error -prevError;

            powerOffset = (kp*error) + (ki*sumError)+ (kd*diffError);
            robot.motorLeft.setPower(absPower + powerOffset);
            robot.motorRight.setPower(absPower - powerOffset);
            sleep(30 - Math.round(loopTimer.milliseconds()));
            prevError = error;
        }
        robot.stopDriveMotors();
    }
}
