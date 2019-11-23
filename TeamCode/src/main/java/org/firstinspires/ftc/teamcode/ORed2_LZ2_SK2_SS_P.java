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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;



@Autonomous(name = "Red LZ2-SK1-P")
public class ORed2_LZ2_SK2_SS_P extends OdometryAutonomous
{
    private static final double ScreenSizeX = 1280;
    private static final double ScreenSizeY = 720;
    private static final double ScreenMiddleX = ScreenSizeX/2;
    private static final double ScreenMiddleY = ScreenSizeY/2;
    private int SkystonePosition = 0;
    private int FinalSystonePosition;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    double SkystoneLeft, SkystoneRight, SkystoneTop, SkystoneBottom;
    double SkystoneMiddleX, SkystoneMiddleY;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AfJzjJr/////AAABmemsjo2zJENej1y8r9qhkaEKCiKoOr5q7Czz5gge3XrThPKl0Pp2eFeQ7ZF96JItNtNrBy9wsk3MHJ1i+DM9TWPwvKNOazd3qZGHFIB4GQg/CN9nsklTwCo20PfgvRftG90CMk7rCBHkDn2qpO+mWz5imgF4G96IDZANpndyrl/zgMgCI+YlEoGMHe8tH8ZH/yWerD6WGcLsfDfGThG495J8qB0DXdp300peDPQCSipTTTsmvUWU/j1a4/JzN6rXW7AkvWpbw5WI+rgSof1VLUaMkjnHthlOILXF6oE1FQeG8qSkTEYbsD0VKmuMw3q6kyZMlF4NLDZHR1xgL4Ho+r/F0siUFi2XwuLlODCSP+q+";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public void runOpMode()
    {
        setConfig();
        initCoords(8.75,39 ,270);
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        while(!opModeIsActive())
        {
            getSkystoneVars();
            if(SkystoneMiddleX<400) SkystonePosition = 0;
            if(SkystoneMiddleX>=400&&SkystoneMiddleX<=700) SkystonePosition = 1;
            if(SkystoneMiddleX>700) SkystonePosition = 2;
            telemetry.addData("SkystoneX", SkystoneMiddleX);
            telemetry.addData("SkystoneY", SkystoneMiddleY);
            telemetry.addData("Skystone Position:",SkystonePosition);
            telemetry.update();
        }
        waitForStart();
        FinalSystonePosition = SkystonePosition;
        openCollector();
        suction();
        if(FinalSystonePosition==0)
        {
            driveToVector(36,28,0.8,0);
            driveToVector(42,28,0.8,0);
            //stopCollector();
            driveToVector(32,28,0.8,0);
        }
        else if(FinalSystonePosition==1)
        {
            driveToVector(36,36,0.8,0);
            driveToVector(42,36,0.8,0);
           // stopCollector();
            driveToVector(32,36,0.8,0);
        }
        else
        {
            driveToVector(36,44,0.8,0);
            driveToVector(42,44,0.8,0);
           // stopCollector();
            driveToVector(32,44,0.8,0);
        }
        driveToVector(30,76,1,90);
        spit();
        driveToVector(30,30,1,90);
        suction();
        if(FinalSystonePosition==0)
        {
            driveToVector(36,28,0.8,0);

        }
        else if(FinalSystonePosition==1)
        {
            driveToVector(36,12,0.8,0);


            driveToVector(42,12,0.8,0);
            stopCollector();
            driveToVector(32,12,0.8,0);
        }
        else
        {
            driveToVector(36,44,0.8,0);

        }
        driveToVector(30,76,1,90);
        spit();
        driveToVector(30,70,0.8,90);



    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.314159265354;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void getSkystoneVars()
    {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if(updatedRecognitions!=null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals("Skystone")) {
                        SkystoneLeft = recognition.getLeft();
                        SkystoneRight = recognition.getRight();
                        SkystoneTop = recognition.getTop();
                        SkystoneBottom = recognition.getBottom();
                    }
                }
            }
            SkystoneMiddleX = (SkystoneLeft + SkystoneRight) / 2;
            SkystoneMiddleY = (SkystoneBottom + SkystoneTop) / 2;
        }
    }
}



