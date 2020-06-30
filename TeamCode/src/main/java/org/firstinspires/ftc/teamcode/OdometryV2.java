package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


// Odometry based navigation system for 2021. Uses X odometer, Y odometer, and REV IMU
// for navigation. Uses IMU for motion profiling.
public abstract class OdometryV2 extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    // DC Motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    //Servo Motors

    // Navigation Variables
    private double fieldX;
    private double fieldY;
    private double fieldT;
    private double initialT;

    // Motion Variables
    private double xVel;
    private double yVel;

    //IMU Configuration
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    //Odometer Variables
    private final double pulseToInch = .0032639031;
    // total reading of odometers in inches
    private double xInch;
    private double yInch;
    // last reading of odometers
    private double lastX = 0;
    private double lastY = 0;

    public void setConfig()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backRight.setDirection(DcMotor.Direction.REVERSE);
       backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!imu.isGyroCalibrated())
        {
        }
    }
    public void updatePosition() //updates current position and true motion
    {
        //Position tracking
       //xInch = motor1.getCurrentPosition()*pulseToInch;   these motors whose encoder ports we use for odometers
       //yInch = motor2.getCurrentPosition()*pulseToInch;

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        fieldT = initialT + Math.toRadians(angles.firstAngle);

        fieldX += (xInch-lastX)*Math.cos(fieldT) -(yInch-lastY)*Math.sin(fieldT);
        fieldY += (xInch-lastX)*Math.sin(fieldT) +(yInch-lastY)*Math.cos(fieldT);

        lastX = xInch;
        lastY = yInch;

        // motion tracking

        Velocity motion = imu.getVelocity();
        xVel = (motion.xVeloc)*Math.cos(fieldT)-(motion.yVeloc*Math.sin(fieldT)); // uses velocity values with respect to
        yVel = (motion.xVeloc)*Math.sin(fieldT)+(motion.yVeloc*Math.cos(fieldT)); // the REV hubs to track field velocity
    }



}
