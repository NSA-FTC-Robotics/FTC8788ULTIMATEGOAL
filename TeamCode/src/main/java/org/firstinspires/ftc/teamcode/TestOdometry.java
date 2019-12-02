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
public class TestOdometry extends OdometryAutonomous
{
@Override
    public void runOpMode() throws InterruptedException
    {

        setConfig();
       initCoords(12,12,0);
        waitForStart();
        while (opModeIsActive()&& !isStopRequested())
        {

            driveTo(24,24,0.6);
            setTheta(0,0.4);
        }
        idle();
    }
}