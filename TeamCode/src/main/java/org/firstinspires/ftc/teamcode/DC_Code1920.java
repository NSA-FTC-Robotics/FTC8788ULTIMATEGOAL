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
    private Servo outake;
    private Servo orienter;
    private Servo grabber;
    private Servo encoderlift;
    private Servo leftclaw;
    private Servo rightclaw;

    private double towerHeight = 0; // tracks the height of the tower the robot is working on
    private double dampener = 1; // slows the robot down on command
    private boolean upPressed; //checks if the up/down button is unpressed before running method code again
    private boolean downPressed;
    private boolean apressed;
    private double speed;
    private double driveangle;
    private double normal;
    private boolean fieldCentric;
    private boolean winchMode;
    private boolean clawposition;
    private double targetHeight;
    private boolean stoneangle;

    //outake positions

    private double out = 0.72;
    private double in = 0.02;
    private double side = 0.19;
    private double rightout = 0.86;
    private double leftout = 0.58;

    //orienter positions

    private double parallel = 0.14; //0.1
    private double perpendicular = .455;
    private double sideangle = 0.278;
    private double rightoutangle = 0.28;
    private double leftoutangle = 0;


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
        passiveWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        activeWinch = hardwareMap.get(DcMotor.class, "activeWinch");
        activeWinch.setDirection(DcMotor.Direction.FORWARD);
        activeWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        activeWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        activeWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        outake = hardwareMap.get(Servo.class, "outake");

        orienter = hardwareMap.get(Servo.class, "orienter");

        grabber = hardwareMap.get(Servo.class, "grabber");
        grabber.setPosition(0);

        encoderlift = hardwareMap.get(Servo.class, "encoderlift");
        encoderlift.setPosition(0.5);

        leftclaw = hardwareMap.get(Servo.class,"leftclaw");
        leftclaw.setPosition(0);

        rightclaw = hardwareMap.get(Servo.class,"rightclaw");
        rightclaw.setPosition(1);

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

        dampener = 1 - (0.65 * (gamepad1.left_trigger));
        driveangle = (Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4);
        speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("DriveAngle", driveangle);
        if(Math.sin(driveangle)>Math.cos(driveangle)) normal = 1/Math.abs((Math.sin(driveangle)));
        else normal = 1/Math.abs((Math.cos(driveangle)));


            frontLeft.setPower(Math.cos(driveangle)*normal*dampener*speed+gamepad1.right_stick_x*dampener);
            frontRight.setPower(Math.sin(driveangle)*normal*dampener*speed-gamepad1.right_stick_x*dampener);
            backLeft.setPower(Math.sin(driveangle)*normal*dampener*speed+gamepad1.right_stick_x*dampener);
            backRight.setPower(Math.cos(driveangle)*normal*dampener*speed-gamepad1.right_stick_x*dampener);

       // strafe(Math.hypot(gamepad1.left_stick_x,gamepad1.left_stick_y), getLeftStickAngle()-getRobotAngle());

    if(gamepad1.y)
    {
        leftWheel.setPower(1);
        rightWheel.setPower(-1);
    }
    if(gamepad1.b)
        {
            leftWheel.setPower(-0.8);
            rightWheel.setPower(0.8);
        }
    if(gamepad1.x)
        {
            leftWheel.setPower(0);
            rightWheel.setPower(0);
        }
   if(gamepad1.a)
   {
       leftWheel.setPower(0.3);
       rightWheel.setPower(-0.3);
   }


//Gamepad 2




        if(gamepad2.left_stick_x>0.05&&!clawposition)
        {
            outake.setPosition(side);
            orienter.setPosition(sideangle);
        }

        if(Math.abs(gamepad2.right_stick_y)>0.05) winchMode = false;
       // if(gamepad2.left_trigger>0.05||gamepad2.right_trigger>0.05|| gamepad2.left_stick_x>0.05) winchMode = true;
           if (winchMode)
           {
               if(gamepad2.left_trigger>0.05&&!clawposition)
               {
                   targetHeight = 500;
                   outake.setPosition(0.02);
                   orienter.setPosition(0.03);
               }
               else if (gamepad2.right_trigger>0.05&&!clawposition)
               {
                   targetHeight = 0;
                   outake.setPosition(0.02);
                   orienter.setPosition(0.03);
               }
               else if(gamepad2.left_stick_x>0.05)
               {
                    outake.setPosition(side);
                    orienter.setPosition(sideangle);}
               int currentHeight = activeWinch.getCurrentPosition();
               if (Math.abs(targetHeight-currentHeight)<5)
               {
                   activeWinch.setPower(0);
                   passiveWinch.setPower(0);
               }
               else if (targetHeight-currentHeight < 1000 && targetHeight-currentHeight > 0)
               {
                   activeWinch.setPower(-1*(0.9*((Math.abs(targetHeight-currentHeight))/1000)+0.1));
                   passiveWinch.setPower(-1*(0.9*((Math.abs(targetHeight-currentHeight))/1000)+0.1));
               }
               else if (targetHeight-currentHeight > 1000 && targetHeight-currentHeight < 0)
               {
                   activeWinch.setPower(1*(0.7*((Math.abs(targetHeight-currentHeight))/1000)+0.3));
                   passiveWinch.setPower(1*(0.7*((Math.abs(targetHeight-currentHeight))/1000)+0.3));
               }
               else if (targetHeight > currentHeight)
               {
                   activeWinch.setPower(-1);
                   passiveWinch.setPower(-1);
               }
               else if (targetHeight < currentHeight)
               {
                   activeWinch.setPower(1);
                   passiveWinch.setPower(1);
               }
               else
               {
                   telemetry.addData("error", 69);
                   telemetry.update();
               }
           }
           else {
               activeWinch.setPower(gamepad2.right_stick_y * (1 - 0.5 * gamepad2.left_trigger));
               passiveWinch.setPower(gamepad2.right_stick_y * (1 - 0.5 * gamepad2.left_trigger));
           }





        if(gamepad2.right_bumper) {
            orienter.setPosition(perpendicular);
            outake.setPosition(out);
            clawposition = true;
            stoneangle = false;
        }
        if(gamepad2.y) // alternate scoring position
        {

            orienter.setPosition(parallel);
            if(clawposition)
                stoneangle = true;
        }

        if(gamepad2.left_bumper)
        {
            orienter.setPosition(perpendicular);
            outake.setPosition(in);
            clawposition = false;
            stoneangle = false;
        }
        if(Math.hypot(gamepad2.right_stick_y,gamepad2.right_stick_x)>0.001&&!clawposition&&!gamepad2.left_bumper&&!(gamepad2.left_stick_x>0.05))
        {
            outake.setPosition(in);
            orienter.setPosition(parallel);
        }

        //Arm manuverability

        if(clawposition)
        {


                if(outake.getPosition()>leftout)
                {
                    if(gamepad2.left_stick_x<0)
                    outake.setPosition(outake.getPosition()+0.005*gamepad2.left_stick_x);
                }
                if(orienter.getPosition()>leftoutangle)
                {
                    if(gamepad2.left_stick_x<0)
                    orienter.setPosition(orienter.getPosition()+0.005*gamepad2.left_stick_x);
                }
                if (outake.getPosition()<rightout)
                {
                    if(gamepad2.left_stick_x>0)
                    outake.setPosition(outake.getPosition()+0.005*gamepad2.left_stick_x);
                }
                if(orienter.getPosition()<rightoutangle)
                {
                    if(gamepad2.left_stick_x>0)
                    orienter.setPosition(orienter.getPosition()+0.005*gamepad2.left_stick_x);
                }
                if(gamepad2.y)
                {
                    outake.setPosition(out);
                    orienter.setPosition(parallel);
                }


        }



        if (gamepad2.a)
            grabber.setPosition(0.3);
        if (gamepad2.b)
            grabber.setPosition(0);



        if (gamepad2.dpad_down)
        {
            leftclaw.setPosition(0.67);
            rightclaw.setPosition(0.35);
        }
        if (gamepad2.dpad_up)
        {
            leftclaw.setPosition(0);
            rightclaw.setPosition(1);
        }




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

        return Math.toRadians(-1 * angles.firstAngle);
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