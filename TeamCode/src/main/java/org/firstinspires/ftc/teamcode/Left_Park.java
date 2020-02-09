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
@Autonomous(name = "Left Park")
//@Disabled
public class Left_Park extends OdometryAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        setConfig();
        initCoords(0,24,0);
        waitForStart();
        while (opModeIsActive()&& !isStopRequested())
        {
            driveToVector(24,0,0.5,0);
            openCollector();
            sleep(2000);
            stop();


        }

    }
}