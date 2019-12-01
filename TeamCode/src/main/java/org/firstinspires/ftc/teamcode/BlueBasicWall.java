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

@Autonomous(name = "CBBW")
//@Disabled
public class BlueBasicWall extends OdometryAutonomous
{
    @Override
    public void runOpMode()
    {
        setConfig();
        initCoords(12,60,90);
        waitForStart();
        if (opModeIsActive())
        {

            driveTo(12,72,0.8);

        }
    }
}