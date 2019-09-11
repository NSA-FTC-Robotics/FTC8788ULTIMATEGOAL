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

@Autonomous(name = "Test Odometry")
//@Disabled

public class BottomTierPsuedo extends OdometryAutonomous
{
    public void runOpMode()
    {
        setConfig();
       // initCoords(0,24,0);
        waitForStart();

        if (opModeIsActive())
        {
            /* Start of Psuedo

            1. Bottom left corner of robot at (0, 24)
            2. Drive forward 39.5 so center of robot is at (0, 72)
            3. Sleep for remainder

             */

        }


    }
}
