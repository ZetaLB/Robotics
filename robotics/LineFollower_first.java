package edu.elon.robotics;

public class LineFollower_first extends AutoCommon {
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
        double error = 0.0;
        double kp = 200;
        while (opModeIsActive()){
            error =robot.colorSensor.alpha() -robot.midBrightness;
            if (error != 0){
                robot.motorLeft.setPower(absPower - (error/kp));
                robot.motorRight.setPower(absPower + (error/kp));
            }
            /*else if(error < 0){
                robot.motorRight.setPower(absPower + (Math.abs(error * kp)));
                robot.motorLeft.setPower(absPower - (Math.abs(error *kp)));
            }*/
            else {
                robot.motorLeft.setPower(absPower);
                robot.motorRight.setPower(absPower);
            }

        }
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
    }
}
