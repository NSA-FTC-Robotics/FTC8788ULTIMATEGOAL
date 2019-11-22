package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorController;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@SuppressWarnings("FieldCanBeLocal")
//@Disabled
public abstract class SimpleAutonomous extends LinearOpMode
{
    /////////////////////////////////
    //                             //
    // note: all angle parameters  //
    //             are in degrees  //
    //                             //
    /////////////////////////////////

/*
                          😜
                       👊/||\_
                       _/¯  ¯\_

                        👋 😳
                           || \_
                        _/¯  ¯\_


 */

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor intake1; //port 0
    private DcMotor intake2; //port 1
    private DcMotor passiveWinch; //port 2
    private DcMotor activeWinch; //port 3

    private Servo leftCollector;
    private Servo rightCollector;
    private Servo outake;
    private Servo orienter;
    private Servo grabber;
    private Servo encoderlift;


    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    private static final double  TicksPerInches = 42.78084870310147;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AfJzjJr/////AAABmemsjo2zJENej1y8r9qhkaEKCiKoOr5q7Czz5gge3XrThPKl0Pp2eFeQ7ZF96JItNtNrBy9wsk3MHJ1i+DM9TWPwvKNOazd3qZGHFIB4GQg/CN9nsklTwCo20PfgvRftG90CMk7rCBHkDn2qpO+mWz5imgF4G96IDZANpndyrl/zgMgCI+YlEoGMHe8tH8ZH/yWerD6WGcLsfDfGThG495J8qB0DXdp300peDPQCSipTTTsmvUWU/j1a4/JzN6rXW7AkvWpbw5WI+rgSof1VLUaMkjnHthlOILXF6oE1FQeG8qSkTEYbsD0VKmuMw3q6kyZMlF4NLDZHR1xgL4Ho+r/F0siUFi2XwuLlODCSP+q+";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    // initializes motors
    public void setConfig()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1 = hardwareMap.get(DcMotor.class, "Intake1");
        intake1.setDirection(DcMotor.Direction.FORWARD);

        intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        intake2.setDirection(DcMotor.Direction.FORWARD);

        passiveWinch = hardwareMap.get(DcMotor.class, "passiveWinch");
        passiveWinch.setDirection(DcMotor.Direction.FORWARD);

        activeWinch = hardwareMap.get(DcMotor.class, "activeWinch");
        activeWinch.setDirection(DcMotor.Direction.FORWARD);
        //activeWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftCollector = hardwareMap.get(Servo.class, "left_collector");
        leftCollector.setPosition(1);

        rightCollector = hardwareMap.get(Servo.class, "right_collector");
        rightCollector.setPosition(0);

        outake = hardwareMap.get(Servo.class, "outake");
        outake.setPosition(0.8);

        orienter = hardwareMap.get(Servo.class, "orienter");
        orienter.setPosition(0.2);

        grabber = hardwareMap.get(Servo.class, "grabber");
        grabber.setPosition(0);

        encoderlift = hardwareMap.get(Servo.class, "encoderlift");
        encoderlift.setPosition(0.7); // needs testing


    }

    // sets initial coordinates of robot

    public void openCollector()
    {
        rightCollector.setPosition(0.4);
        sleep(50);
        leftCollector.setPosition(0.6);
    }
    public void intakeCollector()
    {
        rightCollector.setPosition(.25);
        leftCollector.setPosition(0.75);
    }

    public double getRobotAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return -1 * angles.firstAngle;
    }


    public void strafe(double power, double direction)
    {
        direction = Math.toRadians(direction)- Math.PI/4;

        frontLeft.setPower(Math.cos(direction));
        frontRight.setPower(Math.sin(direction));
        backLeft.setPower(Math.sin(direction));
        backRight.setPower(Math.cos(direction));
    }

    // turns the robots to targetTheta angle
    public void setTheta(double targetTheta, double power)     //degrees input
    {
        double t = Math.toRadians(getRobotAngle());
        while(Math.abs(targetTheta-t)>0.2)
        {
           if(Math.abs(targetTheta-t)<10)
           {
               power = power/2;
           }
            if ((targetTheta < t) && (targetTheta <= t - 180))
            {
                frontLeft.setPower(power);
                frontRight.setPower(-power);
                backLeft.setPower(power);
                backRight.setPower(-power);
            }
            else if ((targetTheta < t) && (targetTheta > t - 180))
            {
                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(-power);
                backRight.setPower(power);
            }
            else if ((targetTheta > t) && (targetTheta <= t + 180))
            {
                frontLeft.setPower(power);
                frontRight.setPower(-power);
                backLeft.setPower(power);
                backRight.setPower(-power);
            }
            else
                {
                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(-power);
                backRight.setPower(power);
            }
        }
    }
    public void translate(double x, double y, double power)
    {
        double flpower = power;
        double frpower = power;
        double blpower = power;
        double brpower = power;
        double direction = Math.atan2(y,x)-Math.PI/4;
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int targetPosition = (int)(Math.hypot(x,y) * TicksPerInches);
        frontLeft.setTargetPosition((int)(targetPosition*Math.cos(direction)));
        frontRight.setTargetPosition((int)(targetPosition*Math.sin(direction)));
        backLeft.setTargetPosition((int)(targetPosition*Math.sin(direction)));
        backRight.setTargetPosition((int)(targetPosition*Math.cos(direction)));

        while(frontLeft.isBusy()||frontRight.isBusy()||backRight.isBusy()||backLeft.isBusy())
        {
            if(frontLeft.isBusy())
            {
                if(Math.abs(frontLeft.getCurrentPosition()-targetPosition)<1000) flpower = (0.85 *(Math.abs(frontLeft.getCurrentPosition()-targetPosition))/1000) +0.15;
                frontLeft.setPower(flpower);
            }
            else frontLeft.setPower(0);
            if(frontRight.isBusy())
            {
                if(Math.abs(frontRight.getCurrentPosition()-targetPosition)<1000) flpower = (0.85 *(Math.abs(frontRight.getCurrentPosition()-targetPosition))/1000) +0.15;
                frontRight.setPower(flpower);
            }
            else frontRight.setPower(0);
            if(backLeft.isBusy())
            {
                if(Math.abs(backLeft.getCurrentPosition()-targetPosition)<1000) flpower = (0.85 *(Math.abs(backLeft.getCurrentPosition()-targetPosition))/1000) +0.15;
                backLeft.setPower(flpower);
            }
            else backLeft.setPower(0);
            if(backRight.isBusy())
            {
                if(Math.abs(backRight.getCurrentPosition()-targetPosition)<1000) flpower = (0.85 *(Math.abs(backRight.getCurrentPosition()-targetPosition))/1000) +0.15;
                backRight.setPower(flpower);
            }
            else backRight.setPower(0);
        }
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}