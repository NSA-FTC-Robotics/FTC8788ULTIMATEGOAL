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
@Autonomous(name = "Red Foundation")
public class Red_Foundation extends OdometryAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        setConfig();
        initCoords(8.75, 105, 90);
        waitForStart();
        while (opModeIsActive()&& !isStopRequested())
        {
            lift();
            driveToVector(24,120,0.8,180);
            backwards(0.35,1500);
            lower();
            driveToVector(12,120,0.8,180);
            setTheta(270,0.5);
            lift();
            driveToVector(12,96,0.8,270);
            lower();
            driveToVector(14,72,0.8,270);
            openCollector();
            stop();
        }

    }
}
