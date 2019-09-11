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

public class FirstTierPsuedo extends OdometryAutonomous
{
    public void runOpMode()
    {
        setConfig();
        // initCoords(0,120,270);
        waitForStart();

        if (opModeIsActive())
        {
            /* Start of Psuedo

            1. Top left corner of robot at (0, 120)
            2. Drive forward 31 in so robot is touching platform at top corner approx (48, 120)
            3. Trigger hook attachment
            4. Drive straight back to (0, 120) 31 inches
            5. Release hook attachment
            6. Strafe 180 degrees to (0, 80.75) top left corner
                of robot - approx 39.25 in
            7. Sleep

             */

        }


    }
}
