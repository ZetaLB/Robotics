package edu.elon.robotics;

/**
 * Define the robot hardware, some useful constants, and a couple
 * of useful methods.
 *
 * @author J. Hollingsworth
 */

import android.text.method.Touch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class RobotHardware {

    private final HardwareMap hardwareMap;

    // drive motors
    public DcMotor motorLeft;
    public DcMotor motorRight;

    // useful constants
    public final double STICK_THRESHOLD = 0.2;
    public final double DRIVE_SPEED_NORMAL = 0.8;
    public final double WHEEL_DIAMETER = 10.16;
    public final double TICKS_PER_ROTATION = 1120;
    public final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public final double TICKS_PER_CM = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
    public final double WHEEL_CIRCLE_DIAMETER = 40;
    public final double WHEEL_CIRCLE_CIRCUMFRENCE = Math.PI * WHEEL_CIRCLE_DIAMETER;
    public final double TICKS_PER_TURN_CM = TICKS_PER_ROTATION / WHEEL_CIRCLE_CIRCUMFRENCE;
    public BNO055IMU imu;
    public boolean calibratedIMU;

    // sensors
    public DigitalChannel touchSensor;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    //color
    public int maxBrightness;
    public int minBrightness;
    public int midBrightness;

    //arm
    public DcMotor motorArm;
    public Servo servoGripper;
    public Servo servoWrist;
    public DigitalChannel touchSensorArm;

    //Arm
    // constants for controlling the arm
    public final double ARM_INIT_POWER = -0.15; // init speed of the motor
    public final double ARM_POWER_UP = 0.3;     // up speed of the motor
    public final double ARM_POWER_DOWN = -0.6;  // down speed of the motor
    public final int ARM_MAX_HEIGHT = 2311;     // encoder ticks of upper limit

    // constants for controlling the wrist
    public static final double WRIST_PICKUP_POS = 0.69; // parallel to ground
    public final double WRIST_FULLY_DOWN = 0.06;        // minimum position
    public final double WRIST_FULLY_UP = 0.87;          // maximum position
    public final double WRIST_INCREMENT = 0.05;        // delta value

    // constants for controlling the gripper
    public static final double GRIPPER_FULLY_OPEN = 0.64; // gripper open
    public final double GRIPPER_FULLY_CLOSED = 0.13;     // gripper closed
    public final double GRIPPER_INCREMENT = 0.05;        // delta value


    public RobotHardware(HardwareMap hardwareMap) {
        // remember the robot configuration
        this.hardwareMap = hardwareMap;

        initializeIMU();
        if (!calibratedIMU) {
            System.out.println("ELON_ROBOTICS: IMU IS NOT CALIBRATED");
        }
        // configure the drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopDriveMotors();
        resetDriveEncoders();
        //sensor
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        servoGripper = hardwareMap.servo.get("servoGripper");
        servoGripper.setPosition(.14);
        servoWrist = hardwareMap.servo.get("servoWrist");
        servoWrist.setPosition(.69);

        touchSensorArm = hardwareMap.get(DigitalChannel.class, "touchSensorArm");
        touchSensorArm.setMode(DigitalChannel.Mode.INPUT);

        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    public void startMove(double drive, double turn, double modifier) {
        double leftPower = (drive + turn) * modifier;
        double rightPower = (drive - turn) * modifier;

        double max = Math.max(leftPower, rightPower);

        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        motorLeft.setPower(Range.clip(leftPower, -1, 1));
        motorRight.setPower(Range.clip(rightPower, -1, 1));
    }

    public void stopDriveMotors() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public void resetDriveEncoders() {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int convertDistanceToTicks(double cm) {
        double ticks = cm * TICKS_PER_CM;
        int tick = (int)ticks;
        return tick;
    }

    public int convertTicksToDistance(double ticks) {
        double cms = ticks / TICKS_PER_CM;
        int cm = (int)cms;
        return cm;
    }
    public int covertDegreesToTicks(double degrees){
        double arc_length = degrees/360.0 * WHEEL_CIRCLE_CIRCUMFRENCE;
        double ticks = convertDistanceToTicks(arc_length);
        int length = (int)ticks;
        return(length);
    }

    public void initializeIMU() {
        //------------------------------------------------------------
        // IMU - BNO055
        // Set up the parameters with which we will use our IMU.
        // + 9 degrees of freedom
        // + use of calibration file (see calibration program)
        //------------------------------------------------------------
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // see the calibration sample opmode
        parameters.calibrationDataFile = "AdafruitImuCalibration.json";
        parameters.loggingEnabled      = false;
        //parameters.loggingTag          = "IMU";
        //parameters.mode                = BNO055IMU.SensorMode.NDOF;

        parameters.accelerationIntegrationAlgorithm = null;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Read the IMU configuration from the data file saved during calibration.
        // Using try/catch allows us to be specific about the error instead of
        // just showing a NullPointer exception that could come from anywhere in the program.
        calibratedIMU = true;
        try {
            File file = AppUtil.getInstance().getSettingsFile(parameters.calibrationDataFile);
            String strCalibrationData = ReadWriteFile.readFile(file);
            BNO055IMU.CalibrationData calibrationData = BNO055IMU.CalibrationData.deserialize(strCalibrationData);
            imu.writeCalibrationData(calibrationData);
        }
        catch (Exception e) {
            calibratedIMU = false;
        }
    }

}
