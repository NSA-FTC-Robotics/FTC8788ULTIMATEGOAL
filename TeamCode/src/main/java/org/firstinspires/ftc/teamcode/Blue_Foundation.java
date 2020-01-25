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
@Autonomous(name = "Blue Foundation - Park")
public class Blue_Foundation extends OdometryAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        setConfig();
        initCoords(8.75, 38, 270);
        release();
        waitForStart();
        while (opModeIsActive()&& !isStopRequested())
        {
            driveToVector(31,20,0.8,180);
            backwards(0.4,800);
            grab();
            backwards(0.4,600);
            // driveToVector(12,124,0.8,180);
            driveToVector(24,36,1,90);
            release();
            backwards(0.5,800);
            driveToVector(12,48,0.8,90);
            grab();
            driveToVector(12,72,0.8,90);
            openCollector();
            stop();

        }

    }
}
