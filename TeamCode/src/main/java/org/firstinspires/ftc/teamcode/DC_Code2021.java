package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;





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
    public class DC_Code2021 extends OpMode
    {
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor frontLeft;
        private DcMotor frontRight;
        private DcMotor backLeft;
        private DcMotor backRight;

        private double commandAngle; // The angle at which the controller is pressed
        private double driveangle;
        private double speed;

        public void init()
        {
            frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
            frontRight = hardwareMap.get(DcMotor.class, "frontright");
            backLeft = hardwareMap.get(DcMotor.class, "backleft");
            backRight = hardwareMap.get(DcMotor.class, "backright");
        }
        @Override
        public void start()
        {
            runtime.reset();
        }

        public void loop()
        {
            commandAngle = (Math.atan2(-gamepad1.left_stick_y,gamepad1.left_stick_x));

            driveangle = commandAngle - Math.PI/4;

            speed = Math.hypot(-gamepad1.left_stick_y,gamepad1.left_stick_x);

            frontLeft.setPower(Math.cos(driveangle)*speed);
            frontRight.setPower(Math.sin(driveangle)*speed);
            backLeft.setPower(Math.sin(driveangle)*speed);
            backRight.setPower(Math.cos(driveangle)*speed);
        }






        @Override
        public void stop() { }
    }
