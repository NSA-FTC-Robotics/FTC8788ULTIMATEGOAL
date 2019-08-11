package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Main DC", group="Iterative Opmode")
//@Disabled
public class DC_Code1920 extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    double dampener = 1;

    private int pulseLeftX ;
    private int pulseRightX ;
    private int pulseRightY ;
    private double inchLeftX;
    private double inchRightX;
    private double inchRightY;
    private final double pulseToInch = .0032639031;
    double lastRY = 0;
    double lastRX = 0;
    double lastLX = 0;
    double diffRY = 0;
    double diffRX = 0;
    double diffLX = 0;
    double dX = 0;
    double dY = 0;
    double dT = 0;
    private double fieldX = 72;
    private double fieldY = 72;
    private double fieldT = 0;

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
        } else if (gamepad1.dpad_up) {
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

        if (fieldT >= 2*Math.PI) fieldT -= 2*Math.PI;
        else if (fieldT<0) fieldT += 2*Math.PI;





        telemetry.addData("x coordinate: ", fieldX);
        telemetry.addData("y coordinate: ", fieldY);
        telemetry.addData("t coordinate: ", Math.toDegrees(fieldT)  );
        telemetry.update();
        telemetry.clear();


    }

    @Override
    public void stop() { }
}
