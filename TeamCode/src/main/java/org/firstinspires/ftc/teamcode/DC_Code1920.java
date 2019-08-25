package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private double dampener = 1;
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
    private boolean mode = false;

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

        pulseRightX = backRight.getCurrentPosition();
        pulseLeftX = frontLeft.getCurrentPosition();

        inchRightX = pulseRightX * pulseToInch * -1;
        inchLeftX = pulseLeftX * pulseToInch * -1;

        diffLX = inchLeftX-lastLX;
        diffRX= inchRightX-lastRX;

        dT = (diffLX-diffRX)/14.5;

        fieldT += dT;

        lastRX = inchRightX;
        lastLX = inchLeftX;

        if (fieldT >= 2*Math.PI) fieldT -= 2*Math.PI;
        else if (fieldT<0) fieldT += 2*Math.PI;

        if(gamepad1.x) mode = true;
        if (gamepad1.y) mode = false;

        double commandTheta = Math.atan(-gamepad1.left_stick_y/gamepad1.left_stick_x);
        double driveTheta = commandTheta + fieldT;
        if(driveTheta<0) driveTheta += 2*Math.PI;
        else if (driveTheta>=2*Math.PI) driveTheta -= 2*Math.PI;
        driveTheta = 0.5*Math.PI - driveTheta;

        dampener = 1-(0.7*(gamepad1.left_trigger));
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
        } else
        {
           if (mode)
           {

               double x = Math.cos(driveTheta);
               double y =  Math.sin(driveTheta);

               frontLeft.setPower((y + x + (gamepad1.right_stick_x)));
               frontRight.setPower((y - x - (gamepad1.right_stick_x)));
               backLeft.setPower((y - x + (gamepad1.right_stick_x)));
               backRight.setPower((y + x - (gamepad1.right_stick_x)));

           }
           else
            {
                frontLeft.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_x))*dampener);
                frontRight.setPower(((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (gamepad1.right_stick_x))*dampener);
                backLeft.setPower(((-gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_x))*dampener);
                backRight.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x) - (gamepad1.right_stick_x))*dampener);
        }



        }
        pulseRightX = backRight.getCurrentPosition();
        pulseLeftX = frontLeft.getCurrentPosition();

        inchRightX = pulseRightX * pulseToInch * -1;
        inchLeftX = pulseLeftX * pulseToInch * -1;

        fieldT = (inchLeftX-inchRightX)/14.5;

        if (fieldT >= 2*Math.PI) fieldT -= 2*Math.PI;
        else if (fieldT<0) fieldT += 2*Math.PI;

        telemetry.addData("Field-Centric Mode", mode);
        telemetry.addData("commandTheta:",commandTheta);
        telemetry.addData("Theta",Math.toDegrees(fieldT));

        telemetry.update();
        telemetry.clear();


    }

    @Override
    public void stop() { }
}
