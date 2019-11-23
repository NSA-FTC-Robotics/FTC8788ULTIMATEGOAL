package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Incremental Test")
//@Disabled
public class IncrementalReadingTest extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotor passiveWinch;
    private DcMotor activeWinch;
    private int pulseLeftX;
    private int pulseRightX;
    private int pulseRightY;
    private double inchLeftX;
    private double inchRightX;
    private double inchRightY;
    private final double pulseToInch = .0032639031;
    private double x;
    private double y;
    private double t;                               //radians
    private double fieldX;
    private double fieldY;
    private double fieldT;
    @Override
    public void runOpMode()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        //backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        activeWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        passiveWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        passiveWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (opModeIsActive())
        {
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            double lastRY = 0;
            double lastRX = 0;
            double lastLX = 0;
            double diffRY = 0;
            double diffRX = 0;
            double diffLX = 0;
            double dX = 0;
            double dY = 0;
            double dT = 0;
           for(int i = 0; 1<1000; i++)
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

               dX =(diffLX+ diffRX)/2;
               dY = diffRY  + 16*dT/(2*Math.PI);
               dT = (diffLX-diffRX)/14.5;


               fieldX += (dX * Math.cos(fieldT) - dY * Math.sin(fieldT));
               fieldY += (dX *Math.sin(fieldT) + dY * Math.cos(fieldT));
               fieldT += dT;

               lastRY = inchRightY;
               lastRX = inchRightX;
               lastLX = inchLeftX;





               telemetry.addData("x coordinate: ", fieldX);
               telemetry.addData("y coordinate: ", fieldY);
               telemetry.addData("t coordinate: ", Math.toDegrees(fieldT)  );
               telemetry.addData("RY:",inchRightY);
               telemetry.addData("RX:",inchRightX);
               telemetry.addData("LX:",inchLeftX);
               telemetry.addData("t:",t);
               telemetry.update();
               telemetry.clear();

           }



        }

    }

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

    public void setTheta(double targetTheta, double power)     //degrees input
    {
        double num = 15;
        t = Math.toDegrees(t);

        while (Math.abs(t - targetTheta) > .02)
        {
            t = (inchLeftX - inchRightX)/14.5;
            t = Math.toDegrees(t);
            x = (inchLeftX + inchRightX)/2;
            y = inchRightY  - 16*t/(2*Math.PI);

            pulseLeftX = frontLeft.getCurrentPosition();
            pulseRightY = frontRight.getCurrentPosition();
            pulseRightX = backRight.getCurrentPosition();

            inchLeftX = pulseLeftX * pulseToInch * -1;
            inchRightX = pulseRightX * pulseToInch * -1;
            inchRightY = pulseRightY * pulseToInch;

            telemetry.addData("Left x: ",  inchLeftX);
            telemetry.addData("Right y: " , inchRightY);
            telemetry.addData("Right x: " , inchRightX);
            telemetry.addData("dTheta (t): ", t);
            telemetry.addData("dX: ", x);
            telemetry.addData("dY: ", y);
            telemetry.update();
            telemetry.clear();

            if (Math.abs(t - targetTheta) < num)
            {
                power = power / 1.2;
                num = num / 2;
            }

            if((targetTheta < t) && (targetTheta <= t - 180) )
            {
                frontLeft.setPower(power);
                frontRight.setPower(-power);
                backLeft.setPower(power);
                backRight.setPower(-power);
            }
            else if((targetTheta < t) && (targetTheta > t - 180) )
            {
                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(-power);
                backRight.setPower(power);
            }
            else if((targetTheta > t) && (targetTheta <= t + 180) )
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

}