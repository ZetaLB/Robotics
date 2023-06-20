package edu.elon.robotics;
/**
 * A simple teleop OpMode for drive the PushBot.
 * + Arm Controls.
 *
 * @author J. Hollingsworth
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import edu.elon.robotics.RobotHardware;

@TeleOp(name = "Drive the Arm Robot", group = "TeleOp")
public class ArmBot extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);

        // move the arm to a known location
        initializeArm();

        waitForStart();

        while (opModeIsActive()) {
            stickDriving();
            controlArm();
            controlHand();
            if (gamepad1.right_bumper) {
                automation();
            }

            /* telemetry */
            showDriveMotorEncoders();
            telemetry.update();
        }
    }

    private void controlArm() {
        double armPower = 0.0;

        // drive the arm up/down
        if (gamepad1.dpad_up && robot.motorArm.getCurrentPosition() < robot.ARM_MAX_HEIGHT) {
            armPower = robot.ARM_POWER_UP;
        } else if (gamepad1.dpad_down && robot.touchSensorArm.getState()) {
            armPower = robot.ARM_POWER_DOWN;
        }

        // reset the encoder when we touch the button
        if (!robot.touchSensorArm.getState()) {
            robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // set the arm power
        robot.motorArm.setPower(armPower);

        // show information to the user
        telemetry.addData("arm power", armPower);
        telemetry.addData("arm ticks", robot.motorArm.getCurrentPosition());
    }

    // starting positions
    private double gripperPos = RobotHardware.GRIPPER_FULLY_OPEN;
    private double wristPos = RobotHardware.WRIST_PICKUP_POS;

    // used for toggling
    private boolean wasAPressed = false;
    private boolean wasBPressed = false;
    private boolean wasXPressed = false;
    private boolean wasYPressed = false;

    private void controlHand() {
        // open/close the gripper
        if (gamepad1.b && !wasBPressed) {
            gripperPos += robot.GRIPPER_INCREMENT;
        } else if (gamepad1.x && !wasXPressed) {
            gripperPos -= robot.GRIPPER_INCREMENT;
        }

        // move the wrist up/down
        if (gamepad1.y && !wasYPressed) {
            wristPos += robot.WRIST_INCREMENT;
        } else if (gamepad1.a && !wasAPressed) {
            wristPos -= robot.WRIST_INCREMENT;
        }

        // remember button presses
        wasAPressed = gamepad1.a;
        wasBPressed = gamepad1.b;
        wasXPressed = gamepad1.x;
        wasYPressed = gamepad1.y;

        // limit the servo to possible gripper positions
        gripperPos = Range.clip(gripperPos, robot.GRIPPER_FULLY_CLOSED, robot.GRIPPER_FULLY_OPEN);
        wristPos = Range.clip(wristPos, robot.WRIST_FULLY_DOWN, robot.WRIST_FULLY_UP);

        // set the servo positions
        robot.servoGripper.setPosition(gripperPos);
        robot.servoWrist.setPosition(wristPos);

        // telemetry
        telemetry.addData("gripper pos", gripperPos);
        telemetry.addData("wrist pos", wristPos);
        telemetry.addData("Alpha",robot.colorSensor.alpha());
        telemetry.addData("Red",robot.colorSensor.red());
        telemetry.addData("Blue",robot.colorSensor.blue());
        telemetry.addData("Green",robot.colorSensor.green());
    }

    private void automation(){
        double open = robot.GRIPPER_FULLY_OPEN;
        double pickup = robot.WRIST_PICKUP_POS;
        double up = robot.WRIST_FULLY_UP;
        final double REDANGLE =90;
        final double REDHEIGHT =50;
        final double REDDISTANCE = 10;

        final double BLUEANGLE =-50;
        final double BLUEHEIGHT = 15;
        final double BLUEDISTANCE = 4;

        int color = 0; //red is 1, blue is 2
        double angle = 0.0;
        double height = 0.0;
        double distance = 0.0;
        robot.servoGripper.setPosition(robot.GRIPPER_FULLY_CLOSED);
        robot.servoWrist.setPosition(robot.WRIST_FULLY_UP);
        sleep(400);
        if (robot.colorSensor.red() > robot.colorSensor.blue()){
            color = 1;
            angle = REDANGLE;
            height = REDHEIGHT;
            distance = REDDISTANCE;
        }
        else{
            color = 2;
            angle = BLUEANGLE;
            height = BLUEHEIGHT;
            distance = BLUEDISTANCE;
        }
        robot.servoWrist.setPosition(pickup);
        double downHeight = 0.0;
        downHeight = robot.ARM_MAX_HEIGHT - robot.convertDistanceToTicks(height);
        double newHeight = robot.convertDistanceToTicks(height);


        robot.motorArm.setPower(robot.ARM_POWER_UP);
        while(opModeIsActive() && robot.motorArm.getCurrentPosition() < robot.ARM_MAX_HEIGHT) {
        }

        robot.resetDriveEncoders();

        robot.motorArm.setPower(0);
        robot.motorArm.setPower(robot.ARM_POWER_DOWN);

        while(opModeIsActive() && robot.motorArm.getCurrentPosition() > newHeight) {
        }
        robot.motorArm.setPower(0);

        if (color == 1){
            robot.servoWrist.setPosition(.10);
        }

        turn(.3,angle);
        sleep(100);
        drive(.3,-distance);

        robot.servoGripper.setPosition(open);
        if (color == 1){
            robot.servoWrist.setPosition(robot.WRIST_PICKUP_POS);
        }
        sleep(200);
        drive(.3,distance);
        sleep(100);
        turn(.3, -angle);

        robot.motorArm.setPower(robot.ARM_INIT_POWER);
        while (robot.touchSensorArm.getState()) {
            // do nothing -- waiting for a button press
        }
        robot.motorArm.setPower(0);

        gamepad1.right_bumper = false;
    }
    private void initializeArm() {

        // move the servo to known good positions
        robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
        robot.servoWrist.setPosition(robot.WRIST_PICKUP_POS);

        // let the servos get to their position before moving the arm
        sleep(500);

        // tell the user that the arm is initializing
        telemetry.addData("ARM", "is initializing");
        telemetry.update();

        // initialize the arm
        robot.motorArm.setPower(robot.ARM_INIT_POWER);
        while (robot.touchSensorArm.getState()) {
            // do nothing -- waiting for a button press
        }
        robot.motorArm.setPower(0);

        // reset the encoder
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // tell the user we are done
        telemetry.addData("ARM", "initialization complete");
        telemetry.update();
    }

    /************************************************************************
     * Drive the robot using the left/right joysticks.
     ************************************************************************/
    private void stickDriving() {
        // simple tank drive controls
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        // create dead zones in case the sticks have drift
        if (Math.abs(gamepad1.left_stick_y) < robot.STICK_THRESHOLD) {
            drive = 0;
        }

        if (Math.abs(gamepad1.right_stick_x) < robot.STICK_THRESHOLD) {
            turn = 0;
        }

        robot.startMove(drive, turn, robot.DRIVE_SPEED_NORMAL);
    }

    /************************************************************************
     * Telemetry - messages that show up on the Driver Station
     ************************************************************************/
    private void showDriveMotorEncoders() {
        telemetry.addData(" motorLeft: ", robot.motorLeft.getCurrentPosition());
        telemetry.addData("motorRight: ", robot.motorRight.getCurrentPosition());
    }
    protected void turn(double power, double degrees){
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

        while (Math.abs(robot.motorLeft.getCurrentPosition()) <= ticks && opModeIsActive()){
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
    }

    protected void drive(double power, double cm){
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

        while (Math.abs(robot.motorLeft.getCurrentPosition()) <= ticks && opModeIsActive()){
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);

    }
}