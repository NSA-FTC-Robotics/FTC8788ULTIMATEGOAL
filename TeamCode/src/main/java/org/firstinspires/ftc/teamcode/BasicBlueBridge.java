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

import org.firstinspires.ftc.teamcode.OdometryAutonomous;

@Autonomous(name = "CBBB")
//@Disabled
public class BasicBlueBridge extends OdometryAutonomous
{
    @Override
    public void runOpMode()
    {
        setConfig();
        initCoords(12,60,90);
        waitForStart();
        if (opModeIsActive())
        {

            driveToVector(36,72,0.8,90);

        }
    }
}