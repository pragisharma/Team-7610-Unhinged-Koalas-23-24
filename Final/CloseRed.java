package org.firstinspires.ftc.teamcode.Auto.Final;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@Autonomous(name = "red close") // closer to the backdrop
public class CloseRed extends LinearOpMode {

    // motors
    private DcMotor tlm, trm, blm, brm;

    double power = 0.4;
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 0.855;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // gyro
    IMU imu;
    IMU.Parameters myIMUparameters;
    private static final int OFFSET = 10;

    // cv

    private static final boolean USE_WEBCAM = true;

    //CHANGE ALL THIS STUFF TO BLUE CONE WHEN WE TRAIN IT >>>>
    private static final String TFOD_MODEL_ASSET = "redsphere.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/redsphere.tflite";
    private static final String[] LABELS = {
            "red sphere",
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {


        tlm = hardwareMap.get(DcMotor.class, "topLeft");
        trm = hardwareMap.get(DcMotor.class, "topRight");
        blm = hardwareMap.get(DcMotor.class, "backLeft");
        brm = hardwareMap.get(DcMotor.class, "backRight");

        tlm.setDirection(DcMotorSimple.Direction.REVERSE);
        blm.setDirection(DcMotorSimple.Direction.REVERSE);

        tlm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        trm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tlm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        imu = hardwareMap.get(IMU.class, "imu");

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                90,
                                0,
                                -45,
                                0  // acquisitionTime, not used
                        )
                )
        );



        // Initialize IMU using Parameters
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            moveBackward(power, (int)(-(trm.getCurrentPosition() + COUNTS_PER_INCH * 7)));
            sleep(20);

            turn(power, -25);
            sleep(20);

            boolean detected = checkTfod();
            sleep(1000);
//
            if(detected){
                // right spike
                // PLACE PIXEL

                endOutput();
                break;
            }

            turn(power, OFFSET);
            sleep(20);

            detected = checkTfod();
            sleep(1000);

            if(detected){
                // case 2 (right spike - 1)
                // PLACE PIXEL
                sleep(20);
                endOutput();
                break;
            }

            turn(power, 25);
            sleep(20);

            detected = checkTfod();
            sleep(1000);

            // left spike
            // PLACE PIXEL
            endOutput();
            break;

        }
    }


    public void endOutput(){
        turn(power, -OFFSET);
        sleep(20);
        moveLeft(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 34));
        sleep(20);
        telemetry.addLine("OUPUT COMPLETED");
        telemetry.update();
        sleep(20);
    }
    public void resetMotors() {
        trm.setPower(0);
        tlm.setPower(0);
        blm.setPower(0);
        brm.setPower(0);

        imu.resetYaw();

        trm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        trm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tlm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveForward(double power, int dist) {
        telemetry.addData("TRM CURR POS (forward) ", trm.getCurrentPosition());
        telemetry.update();
        while (trm.getCurrentPosition() <= dist) {
            telemetry.addLine("INSIDE LOOP ");
            telemetry.addData("TRM CURR POS (forward) ", trm.getCurrentPosition());

            telemetry.update();

            // both right motors were given power * 0.8 and it rotated cw at about 6 in
            trm.setPower(power);    // abs -> (+) => + power to trm
            brm.setPower(power);     // abs -> (+) => + power to brm
            tlm.setPower(power);     // abs -> (+) => + power to tlm (keep in mind left is being fed reverse, so +reverse is -)
            blm.setPower(power);
            sleep(200);
        }

        telemetry.addLine("to be reset (forward)");
        telemetry.update();


        resetMotors();
        sleep(20);
    }


    public void turn(double power, double deg){
        // reset yaw each time?
        // imu.resetYaw();
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
//        telemetry.addData("CURRENT DEGREES: ", robotOrientation.getYaw(AngleUnit.DEGREES));
//        telemetry.update();

        if(deg >= 0){
            // while we want to rotate it cw (+ angle)
            telemetry.addLine("DEGREE IS POSITIVE");
            telemetry.update();
            sleep(20);

            while(deg >=  robotOrientation.getYaw(AngleUnit.DEGREES)) {
                robotOrientation = imu.getRobotYawPitchRollAngles();
                telemetry.addData("CURRENT DEGREES: ", robotOrientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();
                //sleep(20);

                trm.setPower(power);
                brm.setPower(power);
                tlm.setPower(-power);
                blm.setPower(-power);
                sleep(20);
            }
        }else if(deg < 0){
            // while we want to rotate it ccw (- angle)
            telemetry.addLine("DEGREE IS NEGATIVE");
            telemetry.update();
            sleep(20);

            while(deg <= robotOrientation.getYaw(AngleUnit.DEGREES)) {

                robotOrientation = imu.getRobotYawPitchRollAngles();
                telemetry.addData("CURRENT DEGREES: ", robotOrientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();
                trm.setPower(-power);
                brm.setPower(-power);
                tlm.setPower(power);
                blm.setPower(power);
                sleep(20);
            }
        }


        resetMotors();
        sleep(20);

    }
    // u hafta upload the code again b/c its a static variable issue for backwards and right
    public void moveBackward(double power, int dist) {
        telemetry.addData("TRM CURR POS (backward) ", trm.getCurrentPosition());
        telemetry.update();

        while (trm.getCurrentPosition() >= dist) {
            telemetry.addLine("INSIDE LOOP ");
            telemetry.addData("TRM CURR POS (backward) ", trm.getCurrentPosition());
            telemetry.update();
            trm.setPower(-power);    // abs -> (+) => + power to trm
            brm.setPower(-power);     // abs -> (+) => + power to brm
            tlm.setPower(-power);     // abs -> (+) => + power to tlm (keep in mind left is being fed reverse, so +reverse is -)
            blm.setPower(-power);
            sleep(200);
        }

        telemetry.addLine("to be reset. (backward)");
        telemetry.update();

        resetMotors();
        sleep(20);
    }

    public void moveLeft(double power, int dist) {
        telemetry.addData("TRM CURR POS (left) ", trm.getCurrentPosition());
        telemetry.update();

        while (trm.getCurrentPosition() <= dist) { //ref the - motor
            telemetry.addLine("INSIDE LOOP ");
            telemetry.addData("TRM CURR POS (left) ", brm.getCurrentPosition());
            telemetry.update();
            trm.setPower(power);
            brm.setPower(-power);
            blm.setPower(power);
            tlm.setPower(-power);
            sleep(200);

            telemetry.addLine();
            telemetry.update();
        }

        telemetry.addLine("to be reset. (left)");
        telemetry.update();

        resetMotors();
        sleep(20);
    }

    // problem child
    public void moveRight(double power, int dist) {
        // moves 28 inches when its supposed to move 12
        telemetry.addData("TRM CURR POS (right) ", trm.getCurrentPosition());
        telemetry.update();
        while (trm.getCurrentPosition() >= dist) { //ref the + motor
            telemetry.addLine("INSIDE THE LOOP!");
            telemetry.addData("TRM CURR POS (right) ", trm.getCurrentPosition());
            telemetry.update();
            trm.setPower(-power);
            brm.setPower(power);
            blm.setPower(-power);
            tlm.setPower(power);
            sleep(200);
        }

        telemetry.addLine("to be reset. (right)");
        telemetry.update();

        resetMotors();
        sleep(20);
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.

        // check
//        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    private boolean checkTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop


        telemetry.addData("detected: ", currentRecognitions.size() > 0);
        telemetry.update();
        sleep(20);

        return currentRecognitions.size() > 0;

    }   // end method checkTfod()


}