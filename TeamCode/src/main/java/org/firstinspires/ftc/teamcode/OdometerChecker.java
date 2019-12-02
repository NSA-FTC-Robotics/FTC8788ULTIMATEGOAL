package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Odometer Checker", group="Iterative Opmode")
//@Disabled

public class OdometerChecker extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private DcMotor intake1; //port 0
    private DcMotor intake2; //port 1
    private DcMotor passiveWinch; //port 2

    private int pulseLeftX ;
    private int pulseRightX ;
    private int pulseRightY ;
    private double inchLeftX;
    private double inchRightX;
    private double inchRightY;
    private double pulseToInch = .0032639031;

    public void init()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backRight.setDirection(DcMotor.Direction.REVERSE);

        intake1 = hardwareMap.get(DcMotor.class, "Intake1");
        intake1.setDirection(DcMotor.Direction.FORWARD);

        intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        intake2.setDirection(DcMotor.Direction.FORWARD);

        passiveWinch = hardwareMap.get(DcMotor.class, "passiveWinch");
        passiveWinch.setDirection(DcMotor.Direction.FORWARD);

        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        passiveWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        passiveWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        pulseRightY = passiveWinch.getCurrentPosition(); //passiveWich
        pulseRightX = intake2.getCurrentPosition(); //intake 2
        pulseLeftX = intake1.getCurrentPosition(); //intake1

        inchRightY = pulseRightY * pulseToInch;
        inchRightX = pulseRightX * pulseToInch * -1;
        inchLeftX = pulseLeftX * pulseToInch * -1;

        telemetry.addData("Left X ", inchLeftX);
        telemetry.addData("Right X ", inchRightX);
        telemetry.addData("Right Y", inchRightY );
        telemetry.update();
        telemetry.clear();


    }

    @Override
    public void stop() { }
}
