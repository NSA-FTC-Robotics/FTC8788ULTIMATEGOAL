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
@Autonomous(name = "Blue Park")
//@Disabled
public class BluePark extends OdometryAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        setConfig();
        initCoords(135.25, 48, 270);
        waitForStart();
        while (opModeIsActive()&& !isStopRequested())
        {
            driveTo(135.25,72,270);
            sleep(1000);
            stop();


        }

    }
}