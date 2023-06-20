package edu.elon.robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutoCommon extends LinearOpMode {
    private double initialHeading;
    protected RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
    }

    protected void driveForTime(double power, long milliseconds) {
        //start the drive motors
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);

        //wait for some time
        sleep(milliseconds);

        //start the drive motors
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);

    }

    protected void turnForTime(double power, long milliseconds) {
        //start the drive motors
        robot.motorLeft.setPower(-power);
        robot.motorRight.setPower(power);

        //wait for some time
        sleep(milliseconds);

        //start the drive motors
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
    }

    protected void drive(double power, double cm) {
        double posCM = Math.abs(cm);
        int ticks = robot.convertDistanceToTicks(posCM);
        robot.resetDriveEncoders();

        double absPower = Math.abs(power);

        if (cm < 0) {
            absPower = absPower * -1;
        }

        robot.resetDriveEncoders();

        robot.motorLeft.setPower(absPower);
        robot.motorRight.setPower(absPower);

        while (Math.abs(robot.motorLeft.getCurrentPosition()) <= ticks && opModeIsActive()) {
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);

    }

    protected void turn(double power, double degrees) {
        double posDegrees = Math.abs(degrees);
        int ticks = robot.covertDegreesToTicks(posDegrees);
        robot.resetDriveEncoders();

        telemetry.addData("ticks", ticks);
        telemetry.update();

        double absPower = Math.abs(power);

        if (degrees < 0) {
            absPower = absPower * -1;
        }

        robot.resetDriveEncoders();

        robot.motorLeft.setPower(-absPower);
        robot.motorRight.setPower(absPower);

        while (Math.abs(robot.motorLeft.getCurrentPosition()) <= ticks && opModeIsActive()) {
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
    }

    public final double ANGLE_OVERSHOOT = 7;
    public final double TURN_ENDING_POWER = 0.25;
    public final double SLOW_DOWN_DEGREES = 15;

    protected void turnIMU(double power, double degrees) {
        resetAngle();
        double smallPower = 0.0;
        double smalldegrees = 0.0;
        double posDegrees = Math.abs(degrees);
        double absPower = Math.abs(power);
        double ogabsPower = Math.abs(power);
        if (degrees < 0) {
            absPower = absPower * -1;
            //smalldegrees = degrees +SLOW_DOWN_DEGREES;
            smallPower = (ogabsPower - TURN_ENDING_POWER) * -1;
        } else {
            //smalldegrees = degrees -SLOW_DOWN_DEGREES;
            smallPower = (ogabsPower - TURN_ENDING_POWER);
        }
        smalldegrees = (posDegrees - ANGLE_OVERSHOOT) - SLOW_DOWN_DEGREES;

        robot.resetDriveEncoders();

        robot.motorLeft.setPower(-absPower);
        robot.motorRight.setPower(absPower);
        while (Math.abs(getHeading()) < (posDegrees - ANGLE_OVERSHOOT) && opModeIsActive()) {
            if (Math.abs(getHeading()) >= smalldegrees) {
                robot.motorLeft.setPower(-(smallPower));
                robot.motorRight.setPower(smallPower);
            }
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
    }

    protected void driveIMU(double power, double cm) {
        double posCM = Math.abs(cm);
        int ticks = robot.convertDistanceToTicks(posCM);
        robot.resetDriveEncoders();

        double absPower = Math.abs(power);

        if (cm < 0) {
            absPower = absPower * -1;
        }

        robot.resetDriveEncoders();

        robot.motorLeft.setPower(absPower);
        robot.motorRight.setPower(absPower);
        double degree = 0.0;
        double kp = 200;
        while (Math.abs(robot.motorLeft.getCurrentPosition()) <= ticks && opModeIsActive()) {
            degree = getHeading();
            if (degree > 0) {
                robot.motorLeft.setPower(absPower + (degree / kp));
                robot.motorRight.setPower(absPower - (degree / kp));
            } else if (degree < 0) {
                robot.motorRight.setPower(absPower + (Math.abs(degree) / kp));
                robot.motorLeft.setPower(absPower - (Math.abs(degree) / kp));
            } else {
                robot.motorLeft.setPower(absPower);
                robot.motorRight.setPower(absPower);
            }
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
    }

    public void resetAngle() {
        // take a magnetometer reading
        Orientation lastAngles = robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES);

        // set initialHeading to the current angle (-180 to 180)
        initialHeading = lastAngles.firstAngle;
    }

    public double getHeading() {
        // get the current angle
        double firstAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        // adjust that angle by the initial to get a heading
        double heading = firstAngle - initialHeading;

        // pull the heading back between -180 to 180
        if (heading < -180)
            heading += 360;
        else if (heading > 180)
            heading -= 360;

        return heading;
    }

    public void driveUntilTouch(double power) {
        ;
        robot.resetDriveEncoders();

        double absPower = Math.abs(power);


        boolean stop = false;
        while (stop == false && opModeIsActive()) {
            if (robot.touchSensor.getState() == true) {
                telemetry.addData("Touch Sensor ", "Is Not Pressed");
                robot.motorLeft.setPower(absPower);
                robot.motorRight.setPower(absPower);
            } else {
                telemetry.addData("Touch Sensor ", "Is Pressed");
                robot.stopDriveMotors();
                stop = true;
            }
            telemetry.update();
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
    }

    public void driveToCalibrateLightSensor() {
        int ticks = robot.convertDistanceToTicks(20);
        robot.resetDriveEncoders();

        double absPower = .5;

        robot.resetDriveEncoders();

        robot.motorLeft.setPower(absPower);
        robot.motorRight.setPower(absPower);

        int min = 1000000000;
        int max = 0;

        while (Math.abs(robot.motorLeft.getCurrentPosition()) <= ticks && opModeIsActive()) {
            int cur = robot.colorSensor.alpha();
            if (cur < min) {
                min = cur;
            }
            if (cur > max) {
                max = cur;
            }
        }
        robot.maxBrightness = max;
        robot.minBrightness = min;

        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);

        System.out.println("max: " + max);
        System.out.println("set max: " + robot.maxBrightness);
        System.out.println("min: " + min);
        System.out.println("set min: " + robot.minBrightness);

    }

    public int countingLines() {
        robot.resetDriveEncoders();

        final int NOT_ON_LINE = 0;
        final int ON_LINE = 1;

        double absPower = .25;
        int lines = 0;
        boolean stop = false;
        int previous = 0;
        int cur = 0;
        while (stop == false && opModeIsActive()) {
            if (robot.colorSensor.alpha() > (robot.maxBrightness - 1000)) {
                cur = 1;
            }
            if (robot.colorSensor.alpha() < (robot.minBrightness + 200)) {
                cur = 0;
            }

            if (cur == 1 && previous == 0) {
                lines = lines + 1;
                System.out.println(lines);
                previous = 1;
            }
            if (cur == 0 && previous == 1) {
                previous = 0;
            }

            if (robot.touchSensor.getState() == true) {
                telemetry.addData("Touch Sensor ", "Is Not Pressed");
                robot.motorLeft.setPower(absPower);
                robot.motorRight.setPower(absPower);
            } else {
                telemetry.addData("Touch Sensor ", "Is Pressed");
                robot.stopDriveMotors();
                stop = true;
            }
            telemetry.update();
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
        System.out.println(lines);
        return robot.motorLeft.getCurrentPosition();
    }
    public void drivePID (double power) {
        double kp = .000085;
        double kc = .0000266;
        double ki = 0.00001269;
        double kd = 0.00005596;
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

        while (opModeIsActive() && robot.touchSensor.getState() == true){
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
        if(robot.touchSensor.getState() == false) {
            robot.stopDriveMotors();
        }
    }
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

    public void finalLab(double power) {
        double dis = robot.distanceSensor.getDistance(DistanceUnit.CM);
        double powerOffset = .001;
        double newArmHeight = robot.convertDistanceToTicks(40);
        double lowerArmHeight = robot.convertDistanceToTicks(27);
        double finalHeight = robot.convertDistanceToTicks(40);
        boolean stop = false;
//
        while (opModeIsActive()) {
            int red = robot.colorSensor.red();
            int blue = robot.colorSensor.blue();
            int green = robot.colorSensor.green();
            int white = robot.colorSensor.alpha();
            telemetry.addData("Alpha", robot.colorSensor.alpha());
            telemetry.addData("Red", robot.colorSensor.red());
            telemetry.addData("Blue", robot.colorSensor.blue());
            telemetry.addData("Green", robot.colorSensor.green());
            telemetry.update();
            robot.motorRight.setPower(power);
            robot.motorLeft.setPower(power);
            //detect red and turn right
            if (red > 300 && blue < 150 && green < 150) {
                turnIMU(.3, -90);
            }
//            //detect blue and turn left
            else if (blue > green && green > red && red < 300) {
                turnIMU(.3, 95);
            }
//            //detect orange and follow a wall CHECK NUMBERS
            else if (red > 1200 && green > 200 && blue < 500) {
                telemetry.addData("this is", "orange");
                telemetry.addData("distance", dis);
                telemetry.update();
                sleep(300);
                drivePIDWall(.15);
                //pControllerWall(.2);


            } else if (white > 3000) {
                telemetry.addData("this is", "white");
                telemetry.update();
                if (robot.touchSensor.getState() == true) {
                    telemetry.addData("following the line", "true");
                    telemetry.update();
                    telemetry.addData("true or false?", "true");
                    telemetry.update();
                    sleep(200);
                    System.out.println("max" + robot.maxBrightness);
                    System.out.println("min " + robot.minBrightness);
                    calibrateColorSensor();
                    drivePID(.15);
                //} else {
                    telemetry.addData("hit the wall", "true");
                    telemetry.update();
                    robot.motorRight.setPower(0);
                    robot.motorLeft.setPower(0);
                    robot.resetDriveEncoders();
                    sleep(300);

                    drive(.5,-42);
                    telemetry.addData("drove back", "true");
                    telemetry.update();
                    sleep(200);
                    turnIMU(.5,157);
                    telemetry.addData("turning 150", "true");
                    telemetry.update();
                    sleep(200);
                    robot.motorArm.setPower(robot.ARM_POWER_UP);
                    while(opModeIsActive() && robot.motorArm.getCurrentPosition() < newArmHeight) {
                    }
                    telemetry.addData("arm up", "true");
                    telemetry.update();
                    robot.motorArm.setPower(0);
                    drive(.5,-15);
                    telemetry.addData("heading to the light", "true");
                    telemetry.update();
                    robot.motorArm.setPower(robot.ARM_POWER_DOWN);
                    while(opModeIsActive() && robot.motorArm.getCurrentPosition() > lowerArmHeight) {
                    }
                    telemetry.addData("arm down", "true");
                    telemetry.update();
                    robot.motorArm.setPower(0);
                    sleep(200);
                    robot.motorArm.setPower(robot.ARM_POWER_UP);
                    while(opModeIsActive() && robot.motorArm.getCurrentPosition() < finalHeight) {
                    }
                    telemetry.addData("arm up", "true");
                    telemetry.update();
                    robot.motorArm.setPower(0);
                    robot.motorRight.setPower(0);
                    robot.motorLeft.setPower(0);
                    telemetry.addData("stop", "true");
                    telemetry.update();
                }
            }
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
    }
    public void drivePIDWall (double power) {
//        double kp = .000085;
//        double kc = .0000266;
//        double ki = 0.00001269;
//        double kd = 0.005596;
        int red = robot.colorSensor.red();
        int blue = robot.colorSensor.blue();
        int green = robot.colorSensor.green();

        double kp = .006;
        double kc = .0000266;
        double ki = 0;
        double kd = 0;
        //double kd = 0.0005596;
        double error;
        double powerOffset;
        double sumError = 0.0;
        double diffError = 0.0;
        double prevError = 0.0;
        boolean blu = false;
        robot.resetDriveEncoders();

        double absPower = Math.abs(power);

        robot.resetDriveEncoders();

        robot.motorLeft.setPower(absPower);
        robot.motorRight.setPower(absPower);
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()&& !blu){
            loopTimer.reset();
            error = (robot.distanceSensor.getDistance(DistanceUnit.CM)) - 60;
            telemetry.addData("error", error);
            telemetry.update();

            sumError = (0.8*(sumError)) + error;
            diffError = error -prevError;
            //telemetry.addData("leftpoweroffset", leftpowerOffset);
            powerOffset = (kp*error) + (ki*sumError)+ (kd*diffError);
            telemetry.addData("distance",robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("error", error);
            telemetry.addData("poweroffset", powerOffset);
            telemetry.update();
            robot.motorLeft.setPower(absPower + powerOffset);
            robot.motorRight.setPower(absPower - powerOffset);
            sleep(70 - Math.round(loopTimer.milliseconds()));
            prevError = error;
            telemetry.addData("Red", robot.colorSensor.red());
            telemetry.addData("Blue", robot.colorSensor.blue());
            telemetry.addData("Green", robot.colorSensor.green());
            telemetry.update();
            if (blue > green && green > red && red < 300) {
                blu = true;
                robot.stopDriveMotors();
            }
        }
        turnIMU(.3, 95);
        robot.stopDriveMotors();

    }
    public void pControllerWall(double power){
        int red = robot.colorSensor.red();
        int blue = robot.colorSensor.blue();
        int green = robot.colorSensor.green();
        robot.resetDriveEncoders();

        double absPower = Math.abs(power);

        robot.resetDriveEncoders();

        robot.motorLeft.setPower(absPower);
        robot.motorRight.setPower(absPower);
        double error;
        double kp = 0.00005433;
        double powerOffset;
        boolean blu = false;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && !blu){
            error = (robot.distanceSensor.getDistance(DistanceUnit.CM)) - robot.convertDistanceToTicks(40);
            //System.out.println(error);
            /*if (error < 150 && error > -150){
                error = 0;
            }*/
            powerOffset = (kp*error);
            double leftpowerOffset = (kp*error);
            if (error > 0){
                leftpowerOffset = leftpowerOffset*10;
            }
            telemetry.addData("distance",robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("error", error);
            telemetry.addData("poweroffset", powerOffset);
            telemetry.addData("leftpoweroffset", leftpowerOffset);
            telemetry.update();
            robot.motorLeft.setPower(absPower + leftpowerOffset);
            robot.motorRight.setPower(absPower - powerOffset);

            double value = robot.colorSensor.alpha();
            System.out.println("P-CONTROL: " + timer.milliseconds() + ", " + value);
            if (blue > green && green > red && red < 300) {
                blu = true;
            }

        }
        turnIMU(.3, 95);
        robot.stopDriveMotors();
    }
}





