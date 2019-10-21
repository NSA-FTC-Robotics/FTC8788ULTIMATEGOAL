package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Main DC", group="Iterative Opmode")
//@Disabled
public class DC_Code1920 extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private double dampener = 1;
    private Servo leftCollector;
    private Servo rightCollector;

// tester
    private int pulseLeftX ;
    private int pulseRightX ;
    private int pulseRightY ;
    private double inchLeftX;
    private double inchRightX;
    private double inchRightY;
    private final double pulseToInch = .0032639031;
    private double lastRY = 0;
    private double lastRX = 0;
    private double lastLX = 0;
    private double diffRY = 0;
    private double diffRX = 0;
    private double diffLX = 0;
    private double dX = 0;
    private double dY = 0;
    private double dT = 0;
    private double fieldX = 72;
    private double fieldY = 72;
    private double fieldT = 0;

    private double towerHeight = 1;
    private boolean upPressed; //checks if the up/down button is unpressed before running method code again
    private boolean downPressed;


    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    public void init()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);

        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftCollector = hardwareMap.get(Servo.class, "left_collector");
        leftCollector.setPosition(0);

        rightCollector = hardwareMap.get(Servo.class, "right_collector");
        rightCollector.setPosition(1);

        leftWheel = hardwareMap.get(DcMotor.class, "Intake1");
        rightWheel = hardwareMap.get(DcMotor.class, "Intake2");



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated())
        {

        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

    }
    @Override
    public void start()
    {
        runtime.reset();
    }

    public void loop() {
        telemetry.addData("Running", " :)");
        telemetry.update();
        telemetry.clear();
        telemetry.addData("Running", ";)");
        telemetry.update();
        telemetry.clear();

        /*dampener = 1-(0.7*(gamepad1.left_trigger));
        if (gamepad1.dpad_up) {
            frontLeft.setPower(1*dampener);
            frontRight.setPower(1*dampener);
            backLeft.setPower(1*dampener);
            backRight.setPower(1*dampener);
        } else if (gamepad1.dpad_down) {
            frontLeft.setPower(-1*dampener);
            frontRight.setPower(-1*dampener);
            backLeft.setPower(-1*dampener);
            backRight.setPower(-1*dampener);
        } else if (gamepad1.dpad_left) {
            frontLeft.setPower(-1*dampener);
            frontRight.setPower(1*dampener);
            backLeft.setPower(1*dampener);
            backRight.setPower(-1*dampener);
        } else if (gamepad1.dpad_right) {
            frontLeft.setPower(1*dampener);
            frontRight.setPower(-1*dampener);
            backLeft.setPower(-1*dampener);
            backRight.setPower(1*dampener);
        } else {
            frontLeft.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_x))*dampener);
            frontRight.setPower(((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (gamepad1.right_stick_x))*dampener);
            backLeft.setPower(((-gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_x))*dampener);
            backRight.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x) - (gamepad1.right_stick_x))*dampener);


        }

         */

        strafe(Math.hypot(gamepad1.left_stick_x,gamepad1.left_stick_y), getLeftStickAngle()-getRobotAngle());

    if(gamepad1.right_bumper)
    {
        rightCollector.setPosition(0.8);
    }
    else rightCollector.setPosition(1);
    if (gamepad1.left_bumper)
    {
        leftCollector.setPosition(0.2);
    }
    else leftCollector.setPosition(0);


    if(gamepad1.y)
    {
        leftWheel.setPower(1);
        rightWheel.setPower(-1);
    }
    if(gamepad1.b)
        {
            leftWheel.setPower(-1);
            rightWheel.setPower(11);
        }
    if(gamepad1.x)
        {
            leftWheel.setPower(0);
            rightWheel.setPower(0);
        }
        pulseRightY = frontRight.getCurrentPosition();
        pulseRightX = backRight.getCurrentPosition();
        pulseLeftX = frontLeft.getCurrentPosition();

        inchRightY = pulseRightY * pulseToInch;
        inchRightX = pulseRightX * pulseToInch * -1;
        inchLeftX = pulseLeftX * pulseToInch * -1;

        diffRY = inchRightY-lastRY;
        diffRX= inchRightX-lastRX;
        diffLX = inchLeftX-lastLX;

        dX =(diffLX+ diffRX)/2;
        dY = diffRY  + 16*dT/(2*Math.PI);
        dT = (diffLX-diffRX)/14.5;


        fieldX += (dX * Math.cos(fieldT) - dY * Math.sin(fieldT));
        fieldY += (dX *Math.sin(fieldT) + dY * Math.cos(fieldT));
        fieldT += dT;               // angle of robot

        lastRY = inchRightY;
        lastRX = inchRightX;
        lastLX = inchLeftX;

        if (fieldT >= 2*Math.PI) fieldT -= 2*Math.PI;
        else if (fieldT<0) fieldT += 2*Math.PI;


        if (!gamepad2.dpad_up)
        {
            upPressed = true;
        }
        if (!gamepad2.dpad_down)
        {
            downPressed = true;
        }

        if (gamepad2.dpad_up)
        {
            incrementTower();
        }
        if (gamepad2.dpad_down)
        {
            decrementTower();
        }
        if (gamepad2.x)
        {
            towerHeight = 1.0;
        }


       /* telemetry.addData("x coordinate: ", fieldX);
        telemetry.addData("y coordinate: ", fieldY);
        telemetry.addData("t coordinate: ", Math.toDegrees(fieldT)  );
        telemetry.update();
        telemetry.clear();*/


        telemetry.addData("heading: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)  );
        telemetry.addData("left stick angle", getLeftStickAngle());
        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", -1 * gamepad1.left_stick_y);
        telemetry.addData("GO ANGLE", getLeftStickAngle()-getRobotAngle());
        telemetry.addData("Z", getRobotAngle());

        telemetry.addData("towerHeight", towerHeight + " Inches");


    }

    public double getLeftStickAngle()
    {
        double x = gamepad1.left_stick_x;
        double y = -1 * gamepad1.left_stick_y;
        if(x > 0 && y > 0)
        {
            return Math.toDegrees(Math.atan(x/y));
        }
        else if (x > 0 && y < 0)
        {
            return 180 - Math.toDegrees(Math.atan(x/-y));
        }
        else if (x < 0 && y < 0)
        {
            return 180 + Math.toDegrees(Math.atan(-x/-y));
        }
        else if (x < 0 && y > 0)
        {
            return 360 - Math.toDegrees(Math.atan(-x/y));
        }
        else if (x == 0 && y > 0)
        {
            return 0;
        }
        else if (x == 0 && y < 0)
        {
            return 180;
        }
        else if (x > 0 && y == 0)
        {
            return 90;
        }
        else if (x < 0 && y == 0)
        {
            return 270;
        }
        return 0;
    }

    public double getRobotAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return -1 * angles.firstAngle;
    }

    public void incrementTower()
    {
        towerHeight += 4;
        upPressed = false;
    }

    public void decrementTower()
    {
        if (towerHeight >= 5)
        {
            towerHeight -= 4;
        }
    }

    public void strafe(double power, double direction)
    {
        direction = 90 - direction;
        direction = Math.toRadians(direction);

        double x = Math.cos(direction);
        double y =  Math.sin(direction);

        frontLeft.setPower(((y + x) * power)+gamepad1.right_stick_x);
        frontRight.setPower(((y - x) * power)-gamepad1.right_stick_x);
        backLeft.setPower(((y - x) * power)+gamepad1.right_stick_x);
        backRight.setPower(((y + x) * power)-gamepad1.right_stick_x);

        telemetry.addData("cos:" , x);
        telemetry.addData("sin:", y);
        telemetry.update();
        telemetry.clear();
    }






    @Override
    public void stop() { }
}