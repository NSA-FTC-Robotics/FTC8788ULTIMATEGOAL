package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorController;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@SuppressWarnings("FieldCanBeLocal")
//@Disabled
public abstract class OdometryAutonomous extends LinearOpMode
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
    private double lastRY = 0;
    private double lastRX = 0;
    private double lastLX = 0;
    private  double diffRY = 0;
    private double diffRX = 0;
    private double diffLX = 0;
    private double dX = 0;
    private double dY = 0;
    private double dT = 0;
    private double fieldX;
    private double fieldY;
    private double fieldT;
    private double flta = 0;
    private double frta = 0;
    private double brta = 0;
    private double blta = 0;
    private double flma = 0;
    private double frma = 0;
    private double brma = 0;
    private double blma = 0;
    private double lastTime = 0;
    private double diffTime = 0;
    private double vX;
    private double vY;
    private double speed;


    public void setConfig()
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
    public void updateposition()
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
       double  t = Math.toDegrees(fieldT);
        if(Math.abs(t-targetTheta)>1)
        {
            while (Math.abs(t - targetTheta) > .02) {
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
    public void driveTo (double targetX, double targetY, double power)
    {

        double distance =0;
        double da = 1;
        double sd = Math.hypot((targetX-fieldX),(targetY-fieldY));

            distance = Math.hypot((targetX-fieldX),(targetY-fieldY));
            while (distance>.5)
            {
                while (distance>.1) {
                    distance = Math.hypot((targetX - fieldX), (targetY - fieldY));
                    if (distance < 30) da = 0.4;
                    if (distance < 10)
                    {
                        da = 0.25;
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

           // halt();

            updateposition();
        }





}