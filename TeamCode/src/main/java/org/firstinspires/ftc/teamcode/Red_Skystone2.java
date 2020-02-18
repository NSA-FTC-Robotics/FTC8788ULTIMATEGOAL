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

@Autonomous(name = "Red 2 Skystones")
public class Red_Skystone2 extends OdometryAutonomous
{
    private static final double ScreenSizeX = 1280;
    private static final double ScreenSizeY = 720;
    private static final double ScreenMiddleX = ScreenSizeX/2;
    private static final double ScreenMiddleY = ScreenSizeY/2;
    private int SkystonePosition = 0;
    private int FinalSkystonePosition;
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

    public void runOpMode() {
        setConfig();
        initCoords(8.75, 39, 270);
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

        while (!opModeIsActive() && !isStopRequested())
        {
            getSkystoneVars();
            telemetry.addData("SkystonePosition", SkystonePosition);
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            FinalSkystonePosition = SkystonePosition;
            openCollector();


            // collects first stone
            if (FinalSkystonePosition == 0) {
                driveToVector(32, 28, .8, 0);
                driveToVector(54, 28, .8,0);

                driveToVector(32, 28, .8, 0);
            } else if (FinalSkystonePosition == 1) {
                driveToVector(32, 36, .8, 0);
                driveToVector(52, 36, .8,0);

                driveToVector(32, 36, .8, 0);
            } else if (FinalSkystonePosition == 2) {
                driveToVector(32, 44, .8, 0);
                driveToVector(52, 44, .8, 0); // needs fixing

                driveToVector(32, 44, .8, 0);
            }

            // drops off first stone
            driveToVector(33, 72, .8, 90);
            driveToVector(33, 80, .8, 90);




               // exact same loop, but all target coords are 24 less y

            // collects second stone
            if (SkystonePosition != 0)
            {
                driveToVector(32, 30, 1, 90);
                if (SkystonePosition == 1) {
                    driveToVector(32, 12, .8, 0);
                    driveToVector(52, 12, .8, 0);
                    driveToVector(32, 12, .8, 0);
                } else {
                    driveToVector(32, 20, .8, 0);
                    driveToVector(52, 20, .8, 0);
                    driveToVector(32, 20, .8, 0);
                }

                driveToVector(33, 72, .8, 90);
                driveToVector(33, 96, .8, 90);

            }
            driveToVector(28, 72, .8, 90);       // park
                stop();
            }

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
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void getSkystoneVars()
    {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if(updatedRecognitions!=null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals("Skystone"))
                    {
                        SkystoneLeft = recognition.getLeft();
                        SkystoneRight = recognition.getRight();
                        SkystoneTop = recognition.getTop();
                        SkystoneBottom = recognition.getBottom();
                    }
                }
            }
            SkystoneMiddleX = (SkystoneLeft + SkystoneRight) / 2;
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 1) // determines if the stone is in the center position as the phone would interpret all three stones as one
                    SkystonePosition = 1; // center stone
                else if (SkystoneMiddleX >= 650 && updatedRecognitions.size() != 1) // Checks to see if the skystone and stone interpetted as a single skystone object is on the right of the detection zone.
                    SkystonePosition = 2; //right stone
                else // if it sees two objects and the skystone is not on the right, then the skystone must be on the left
                    SkystonePosition = 0; //left stone
            }
            }


        }
    }





