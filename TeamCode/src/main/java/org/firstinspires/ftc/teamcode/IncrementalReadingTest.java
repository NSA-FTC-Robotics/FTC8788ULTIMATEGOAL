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