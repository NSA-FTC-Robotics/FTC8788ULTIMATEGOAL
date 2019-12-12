package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private DcMotor passiveWinch; //port 2
    private DcMotor activeWinch; //port 3
    private Servo leftCollector;
    private Servo rightCollector;
    private Servo outake;
    private Servo orienter;
    private Servo grabber;
    private Servo encoderlift;
    private Servo capstonePlacer;

    private double towerHeight = 0; // tracks the height of the tower the robot is working on
    private double dampener = 1; // slows the robot down on command
    private boolean upPressed; //checks if the up/down button is unpressed before running method code again
    private boolean downPressed;
    private boolean apressed;
    private double speed;
    private double driveangle;
    private boolean fieldCentric;
    private boolean winchMode;
    private boolean clawposition;

    private final double ticksPerLevel = 342.2467;


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

        leftWheel = hardwareMap.get(DcMotor.class, "Intake1");
        leftWheel.setDirection(DcMotor.Direction.FORWARD);

        rightWheel = hardwareMap.get(DcMotor.class, "Intake2");
        rightWheel.setDirection(DcMotor.Direction.FORWARD);

        passiveWinch = hardwareMap.get(DcMotor.class, "passiveWinch");
        passiveWinch.setDirection(DcMotor.Direction.FORWARD);

        activeWinch = hardwareMap.get(DcMotor.class, "activeWinch");
        activeWinch.setDirection(DcMotor.Direction.FORWARD);
        activeWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftCollector = hardwareMap.get(Servo.class, "left_collector");
        //leftCollector.setPosition(1);

        rightCollector = hardwareMap.get(Servo.class, "right_collector");
        //rightCollector.setPosition(0);

        outake = hardwareMap.get(Servo.class, "outake");
        outake.setPosition(0.85);

        orienter = hardwareMap.get(Servo.class, "orienter");
        orienter.setPosition(0.18);

        grabber = hardwareMap.get(Servo.class, "grabber");
        grabber.setPosition(0);

        encoderlift = hardwareMap.get(Servo.class, "encoderlift");
        encoderlift.setPosition(0.5);

        capstonePlacer = hardwareMap.get(Servo.class, "capstonePlacer");
        capstonePlacer.setPosition(1);

        fieldCentric = false;
        apressed = false;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

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

        dampener = 1 - (0.5 * (gamepad1.left_trigger));
        driveangle = (Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4);
        speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("DriveAngle", driveangle);

        if (fieldCentric)
        {
            frontLeft.setPower((Math.cos((driveangle-getRobotAngle())%(2*Math.PI)) * dampener * speed)+gamepad1.right_stick_x);
            frontRight.setPower((Math.sin((driveangle-getRobotAngle())%(2*Math.PI)) * dampener * speed)-gamepad1.right_stick_x);
            backLeft.setPower((Math.sin((driveangle-getRobotAngle())%(2*Math.PI)) * dampener * speed)+gamepad1.right_stick_x);
            backRight.setPower((Math.cos((driveangle-getRobotAngle())%(2*Math.PI)) * dampener * speed)-gamepad1.right_stick_x);
        }
        else
        {
            frontLeft.setPower(Math.cos(driveangle)*dampener*speed+gamepad1.right_stick_x*dampener);
            frontRight.setPower(Math.sin(driveangle)*dampener*speed-gamepad1.right_stick_x*dampener);
            backLeft.setPower(Math.sin(driveangle)*dampener*speed+gamepad1.right_stick_x*dampener);
            backRight.setPower(Math.cos(driveangle)*dampener*speed-gamepad1.right_stick_x*dampener);
        }
       // strafe(Math.hypot(gamepad1.left_stick_x,gamepad1.left_stick_y), getLeftStickAngle()-getRobotAngle());

    if(gamepad1.right_bumper)
    {
        //collector in
        leftCollector.setPosition(0.75);
        rightCollector.setPosition(0.17);

    }
    else
    {
        //collector open
        leftCollector.setPosition(0.6);
        rightCollector.setPosition(0.4);
    }

    if(gamepad1.a&&gamepad1.left_bumper)
    {
        capstonePlacer.setPosition(0.62);
    }
    else capstonePlacer.setPosition(1);
    if(gamepad1.y)
    {
        leftWheel.setPower(1);
        rightWheel.setPower(-1);
    }
    if(gamepad1.b)
        {
            leftWheel.setPower(-1);
            rightWheel.setPower(1);
        }
    if(gamepad1.right_bumper)
    {
        leftWheel.setPower(-0.3);
        rightWheel.setPower(0.3);
    }
    if(gamepad1.x)
        {
            leftWheel.setPower(0);
            rightWheel.setPower(0);
        }
    if(gamepad1.a && !apressed)
    {
        if(fieldCentric)
        {
            fieldCentric = false;
        }
        else fieldCentric = true;
        apressed = true;
    }
    if(!gamepad1.a)apressed=false;

//Gamepad 2



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
        if (gamepad2.dpad_right)
        {
            //target = tower height
            winchMode = true;
        }
        if (gamepad2.dpad_left)
        {
            //target = 1
            winchMode = true;
        }

        //if(Math.abs(gamepad2.right_stick_y)>0.05)

            winchMode = false;
            activeWinch.setPower(-gamepad2.right_stick_y*0.5*(1-0.5*(gamepad2.left_trigger)));
            passiveWinch.setPower(-gamepad2.right_stick_y*0.5*(1-0.5*(gamepad2.left_trigger)));

       /* if(winchMode)
        {
            activeWinch.setTargetPosition((int)((towerHeight)*ticksPerLevel));
            double winchpower = (0.5*((activeWinch.getTargetPosition()-activeWinch.getCurrentPosition())*0.01+0.2));

            if(activeWinch.getCurrentPosition()<activeWinch.getTargetPosition())
            {
                activeWinch.setPower(winchpower);
                passiveWinch.setPower(winchpower);
            }
            if(activeWinch.getCurrentPosition()>activeWinch.getTargetPosition())
            {
                activeWinch.setPower(-winchpower);
                passiveWinch.setPower(-winchpower);
            }
        }

        */
        if (gamepad2.x)
        {
            towerHeight = 0;
            winchMode = true;
        }


        if(gamepad2.right_bumper) {
            outake.setPosition(0.15);
            orienter.setPosition(0.34);
            clawposition = true;
        }
        if(gamepad2.y) // alternate scoring position
        {
           outake.setPosition(0.25);
            orienter.setPosition(0.6);
        }

        if(gamepad2.left_bumper)
        {
            outake.setPosition(0.9);
            orienter.setPosition(0.2);
            clawposition = false;
        }
        if(Math.hypot(gamepad2.right_stick_y,gamepad2.right_stick_x)>0.001&&!clawposition&&!gamepad2.left_bumper)
        {
            outake.setPosition(0.8);
            orienter.setPosition(0.18);
        }

        if (gamepad2.a)
            grabber.setPosition(0.3);
        if (gamepad2.b)
            grabber.setPosition(0);




     /*   telemetry.addData("heading: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)  );
        telemetry.addData("left stick angle", getLeftStickAngle());
        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", -1 * gamepad1.left_stick_y);
        telemetry.addData("GO ANGLE", getLeftStickAngle()-getRobotAngle());
        telemetry.addData("Z", getRobotAngle());
*/
        telemetry.addData("towerHeight:", towerHeight + " Inches");
        if(fieldCentric)telemetry.addData( "Mode:","Field-Centric");
        else telemetry.addData( "Mode:","Robo-Centric");


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
        towerHeight += 1;
        upPressed = false;
    }

    public void decrementTower()
    {
        if (towerHeight >= 1)
        {
            towerHeight -= 1;
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