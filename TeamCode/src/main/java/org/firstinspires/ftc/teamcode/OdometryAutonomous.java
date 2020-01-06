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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@SuppressWarnings("FieldCanBeLocal")
//@Disabled
public abstract class OdometryAutonomous extends LinearOpMode
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

    // original test odometry variables (ignore)
    private int pulseLeftX;
    private int pulseRightX;        // intake 2
    private int pulseRightY;
    private double inchLeftX;
    private double inchRightX;
    private double inchRightY;
    private final double pulseToInch = .0032639031;
    private double lastRY = 0;
    private double lastRX = 0;
    private double lastLX = 0;
    private  double diffRY = 0;
    private double diffRX = 0;
    private double diffLX = 0;
    private double dX = 0;
    private double dY = 0;
    private double dT = 0;

    private double fieldX;          // robot's current x coord
    private double fieldY;          // robot's current y coord
    private double fieldT;          // robot's current angle

    // ta = turn aspect
    private double flta = 0;
    private double frta = 0;
    private double brta = 0;
    private double blta = 0;

    // ma = motion aspect
    private double flma = 0;
    private double frma = 0;
    private double brma = 0;
    private double blma = 0;

    private double lastTime = 0;
    private double diffTime = 0;
    private double vX;
    private double vY;
    private double speed;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";





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
        //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setDirection(DcMotor.Direction.FORWARD);
       // backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backRight.setDirection(DcMotor.Direction.REVERSE);
       // backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        outake = hardwareMap.get(Servo.class, "outake");
       // outake.setPosition(0.8);

        orienter = hardwareMap.get(Servo.class, "orienter");
        //orienter.setPosition(0.2);

        grabber = hardwareMap.get(Servo.class, "grabber");
       // grabber.setPosition(0);

        encoderlift = hardwareMap.get(Servo.class, "encoderlift");
        encoderlift.setPosition(0.7); // needs testing



        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        passiveWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        passiveWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // sets initial coordinates of robot

    public void openCollector()
    {
        rightCollector.setPosition(0.45);
        sleep(50);
        leftCollector.setPosition(0.55);
    }
    public void intakeCollector()
    {
        leftCollector.setPosition(0.75);
        rightCollector.setPosition(0.17);
    }

    public void initCoords(double x, double y, double t)
    {
         fieldX = x;
         fieldY = y;
         fieldT = Math.toRadians(t);
    }

    public double getFieldX()
    {
        return fieldX;
    }
    public double getFieldY()
    {
        return fieldY;
    }
    public double getFieldT()
    {
        return fieldT;
    }

    // constantly updates robot's current x/y point
    public void updateposition()
    {
        pulseRightY = passiveWinch.getCurrentPosition(); //passiveWich
        pulseRightX = intake2.getCurrentPosition(); //intake 2
        pulseLeftX = intake1.getCurrentPosition(); //intake1

        inchRightY = pulseRightY * pulseToInch *-1;
        inchRightX = pulseRightX * pulseToInch * -1;
        inchLeftX = pulseLeftX * pulseToInch ;

        diffRY = inchRightY-lastRY;
        diffRX= inchRightX-lastRX;
        diffLX = inchLeftX-lastLX;

        diffTime = getRuntime()-lastTime;

        dX =(diffLX+ diffRX)/2;
        dY = diffRY  + 16*dT/(2*Math.PI);
        dT = (diffLX-diffRX)/14.5;

        vX = dX/diffTime;
        vY = dY/diffTime;

        speed = Math.hypot(vX,vY);

        fieldX += (dX * Math.cos(fieldT) - dY * Math.sin(fieldT));
        fieldY += (dX *Math.sin(fieldT) + dY * Math.cos(fieldT));
        fieldT += dT;

        lastRY = inchRightY;
        lastRX = inchRightX;
        lastLX = inchLeftX;

        lastTime = getRuntime();

        if (fieldT >= 2*Math.PI) fieldT -= 2*Math.PI;
        else if (fieldT<0) fieldT += 2*Math.PI;

        telemetry.addData("x coordinate: ", fieldX);
        telemetry.addData("y coordinate: ", fieldY);
        telemetry.addData("t coordinate: ", Math.toDegrees(fieldT)  );
        telemetry.update();
        telemetry.clear();
    }

    // self explanatory
    public void strafe(double power, double direction)
    {
        direction = 90 - direction;
        direction = Math.toRadians(direction);

        double x = Math.cos(direction);
        double y =  Math.sin(direction);

        frontLeft.setPower(y + x);
        frontRight.setPower(y - x);
        backLeft.setPower(y - x);
        backRight.setPower(y + x);

        telemetry.addData("cos:" , x);
        telemetry.addData("sin:", y);
        telemetry.update();
        telemetry.clear();
    }

    // turns the robots to targetTheta angle
    public void setTheta(double targetTheta, double power)     //degrees input
    {
        double num = 15;
        double  t = Math.toDegrees(fieldT);
        if(Math.abs(t-targetTheta)>1 && !isStopRequested())
        {
            while (Math.abs(t - targetTheta) > .02 && !isStopRequested()) {
                updateposition();
                t = Math.toDegrees(fieldT);

                if (Math.abs(t - targetTheta) < num) {
                    power = power / 1.2;
                    num = num / 2;
                }
                if ((targetTheta < t) && (targetTheta <= t - 180)) {
                    frontLeft.setPower(power);
                    frontRight.setPower(-power);
                    backLeft.setPower(power);
                    backRight.setPower(-power);
                } else if ((targetTheta < t) && (targetTheta > t - 180)) {
                    frontLeft.setPower(-power);
                    frontRight.setPower(power);
                    backLeft.setPower(-power);
                    backRight.setPower(power);
                } else if ((targetTheta > t) && (targetTheta <= t + 180)) {
                    frontLeft.setPower(power);
                    frontRight.setPower(-power);
                    backLeft.setPower(power);
                    backRight.setPower(-power);
                } else {
                    frontLeft.setPower(-power);
                    frontRight.setPower(power);
                    backLeft.setPower(-power);
                    backRight.setPower(power);
                }
            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void forward(double power)
    {
        frontLeft.setPower(0.5*power);
        frontRight.setPower(0.5*power);
        backLeft.setPower(0.5*power);
        backRight.setPower(0.5*power);
        sleep(200);
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    public void backward(double power)
    {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
    }

    public void zeroPower()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // send in x/y point, points you in angle direction to go
    public double target(double x, double y) //returns degrees
    {
        double dx = x-fieldX;
        double dy = y-fieldY;
        double targetTheta = Math.atan2(dy,dx);
        if (targetTheta >= 2*Math.PI) targetTheta -= 2*Math.PI;
        else if (targetTheta<0) targetTheta += 2*Math.PI;
        targetTheta=Math.toDegrees(targetTheta);
        return (targetTheta);
    }

    // turns robot to certain angle while moving (turning component of mvmt)
    public void alterTheta(double targetTheta)
    {
       double p=0.4;
       if (Math.abs(targetTheta - Math.toDegrees(fieldT))<5)
           p = 0.2;
       else if (Math.abs(targetTheta - Math.toDegrees(fieldT))<1.5)
           p= 0;
        if ((targetTheta < Math.toDegrees(fieldT)) && (targetTheta <= Math.toDegrees(fieldT) - 180)) {
            flta = p;
            frta = -p;
            blta = p;
            brta = -p;
        } else if ((targetTheta < Math.toDegrees(fieldT)) && (targetTheta > Math.toDegrees(fieldT) - 180)) {
            flta = -p;
            frta = p;
            blta = -p;
            brta = p;
        } else if ((targetTheta > Math.toDegrees(fieldT)) && (targetTheta <= Math.toDegrees(fieldT) + 180)) {
            flta = p;
            frta = -p;
            blta = p;
            brta = -p;
        } else {
            flta = -p;
            frta = p;
            blta = -p;
            brta = p;
        }
    }

    // strafes robot to fix current path to x/y point (strafe component of mvmt)
    public void alterTragectory(double direction)
    {
        direction = 90 - direction;
        direction = Math.toRadians(direction);
        direction = fieldT+direction;
        double x = Math.cos(direction);
        double y =  Math.sin(direction);

       flma = y+x ;
       frma = y-x ;
       blma = y-x ;
       brma = y+x ;
    }

    // tells robot to drive to given (x,y) point
    public void driveTo (double targetX, double targetY, double power)
    {

        double distance =0;
        double da = 1;

            distance = Math.hypot((targetX-fieldX),(targetY-fieldY));
            while (distance >1 && !isStopRequested())
            {
                while (distance >.9 && !isStopRequested()) {
                    distance = Math.hypot((targetX - fieldX), (targetY - fieldY));
                    if (distance < 30) da = 0.4;
                    if (distance < 10)
                    {
                        //da = 0.25;
                        if( power>0.5)
                            power=0.4;
                    }
                    updateposition();
                    alterTheta(target(targetX, targetY));
                    alterTragectory(target(targetX, targetY));
                    frontLeft.setPower((flma * power * da) + flta * da);
                    frontRight.setPower((frma * power * da) + frta * da);
                    backLeft.setPower((blma * power * da) + blta * da);
                    backRight.setPower((brma * power * da) + brta * da);
                    updateposition();
                }
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                updateposition();
                distance = Math.hypot((targetX - fieldX), (targetY - fieldY));
            }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
            updateposition();
    }

    // same as driveTo but the robot faces inputed direction once reaches target coordinate
    public void driveToVector (double targetX, double targetY, double power, double endDirection)
    {
        double distance;
        double da = 1;
        //double sd = Math.hypot((targetX-fieldX),(targetY-fieldY));

        distance = Math.hypot((targetX-fieldX),(targetY-fieldY));
        while (distance >1 && !isStopRequested())
        {
            while (distance >.9 && !isStopRequested()) {
                distance = Math.hypot((targetX - fieldX), (targetY - fieldY));
                if(distance < 20)
                da = Math.abs(((distance / 20)  * .8) + .2);

                updateposition();
                alterTheta(endDirection);
                alterTragectory(target(targetX, targetY));
                frontLeft.setPower((flma * power * da) + (flta * power * da));
                frontRight.setPower((frma * power * da) + (frta * power * da));
                backLeft.setPower((blma * power * da) + (blta * power * da));
                backRight.setPower((brma * power * da) + (brta * power * da));
                updateposition();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            updateposition();
            distance = Math.hypot((targetX - fieldX), (targetY - fieldY));
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        updateposition();
    }

    // ensures robot drives through a certain (x,y) point, does not stop at point
    public void waypoint(double targetX, double targetY, double power, double precision)
    {
        double distance =0;
        distance = Math.hypot((targetX-fieldX),(targetY-fieldY));
       while (distance > precision && !isStopRequested())
       {
           updateposition();
           alterTheta(target(targetX, targetY));
           alterTragectory(target(targetX, targetY));
           frontLeft.setPower((flma * power) + flta);
           frontRight.setPower((frma * power) + frta);
           backLeft.setPower((blma * power) + blta);
           backRight.setPower((brma * power) + brta);
           updateposition();
           distance = Math.hypot((targetX-fieldX),(targetY-fieldY));
       }
    }

    // ensures robot drives through a certain (x,y) point, does not stop at point, but finishes in angle direction
    public void waypointVector(double targetX, double targetY, double power, double precision, double endDirection)
    {

        double distance =0;
        distance = Math.hypot((targetX-fieldX),(targetY-fieldY));
        while (distance > precision && !isStopRequested())
        {
            updateposition();
            alterTheta(endDirection);
            alterTragectory(target(targetX, targetY));
            frontLeft.setPower((flma * power) + flta);
            frontRight.setPower((frma * power) + frta);
            backLeft.setPower((blma * power) + blta);
            backRight.setPower((brma * power) + brta);
            updateposition();
            distance = Math.hypot((targetX-fieldX),(targetY-fieldY));
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
    public void setPower0()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void suction ()
    {
        intake1.setPower(-1);
        intake2.setPower(1);
    }

    public void stopCollector()
    {
        intake1.setPower(0);
        intake2.setPower(0);
    }
    public void spit()
    {
        intake1.setPower(0.5);
        intake2.setPower(-0.5);
    }
    public  void leftspin()
    {
        intake2.setPower(0.5);
    }

}