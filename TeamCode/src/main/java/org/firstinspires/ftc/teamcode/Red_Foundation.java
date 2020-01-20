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
@Autonomous(name = "Red Foundation - Park")
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
           driveToVector(31,124,0.8,180);
           backwards(0.5,500);
           grab();
           driveToVector(24,108,0.8,270);
           release();
           backwards(0.5,500);
           driveToVector(12,96,0.8,270);
           grab();
           driveToVector(12,72,0.8,270);
           stop();
        }

    }
}
