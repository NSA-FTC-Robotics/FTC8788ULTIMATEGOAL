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
@Autonomous(name = "Blue Foundation")
//@Disabled
public class Blue_Foundation extends OdometryAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        setConfig();
        initCoords(8.75, 39, 270);
        waitForStart();
        while (opModeIsActive()&& !isStopRequested())
        {
            lift();
            driveToVector(24,24,0.8,180);
            backwards(0.35,1500);
            lower();
            driveToVector(12,24,0.8,180);
            setTheta(90,0.5);
            lift();
            driveToVector(12,48,0.8,90);
            lower();
            driveToVector(14,72,0.8,90);
            openCollector();
             stop();


        }

    }
}