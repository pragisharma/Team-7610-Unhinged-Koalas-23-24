package org.firstinspires.ftc.teamcode.Auto.Final;

import static org.firstinspires.ftc.teamcode.Auto.Final.CloseBlue.ArmStates.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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


@Autonomous(name = "blue far") // closer to the backdrop
public class FarBlue extends LinearOpMode {

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
    private static final String TFOD_MODEL_ASSET = "bluesphere.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/bluesphere.tflite";
    private static final String[] LABELS = {
            "blue sphere",
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    //arm stuff
    DcMotor joint1, joint2;
    AnalogInput pot;
    Servo claw;

    enum ArmStates {STORAGE, L1_DEPLOY_HIGH, L2_PICKUP, PICKUP, L1_RETRACT_HIGH, L2_STORAGE}
    CloseBlue.ArmStates armState = STORAGE;

    double STORAGE_ALPHA_ANGLE; // Make sure to init this when you init the opmode
    //STORAGE_ALPHA_ANGLE = (pot.getVoltage() * 270 / 3.3);
    //^^init code
    int STORAGE_BETA_TICKS = 0;

    double PICKUP_ALPHA_ANGLE; //to be init later
    int PICKUP_BETA_TICKS = 303;

    final double bP = 0.005;
    double maxArmPower = 1.0;

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

        resetMotors(); //no robot restart needed then

        joint1 = hardwareMap.get(DcMotor.class, "joint 1");
        joint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pot = hardwareMap.get(AnalogInput.class, "joint 1 pot");

        joint2 = hardwareMap.get(DcMotor.class, "joint 2");
        joint2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.get(Servo.class, "claw");

        //more arm stuff
        STORAGE_ALPHA_ANGLE = (pot.getVoltage() * 270 / 3.3);
        PICKUP_ALPHA_ANGLE = STORAGE_ALPHA_ANGLE + 115;
        // int PICKUP_BETA_TICKS = 303;



        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        claw.setPosition(0);
        waitForStart();

        while (opModeIsActive()) {
            moveBackward(power, (int)(-(trm.getCurrentPosition() + COUNTS_PER_INCH * 7)));
            sleep(20);

            turn(power, -25);
            sleep(20);

            boolean detected = false;

            for(int i = 0; i < 2; i++){
                detected = checkTfod();
                sleep(1000);
            }

            if(detected){
                bonkProp(20, 9);
                storageToPickup();
                sleep(20);
                // open claw
                claw.setPosition(1);
                sleep(500);
                pickupToStorage();
                sleep(20);
                endOutput(28);
                break;
            }

            turn(power, OFFSET);
            sleep(20);

            for(int i = 0; i < 2; i++){
                detected = checkTfod();
                sleep(1000);
            }

            if(detected){
                // case 2 (right spike - 1)
                bonkProp(18, 6);
                moveRight(power, (int)(-(trm.getCurrentPosition() + COUNTS_PER_INCH * 2)));
                sleep(20);
                storageToPickup();
                sleep(20);
                // open claw
                claw.setPosition(1);
                sleep(500);
                pickupToStorage();
                sleep(20);
                endOutput(0);
                break;
            }

            turn(power, 25 + OFFSET);
            sleep(20);

            for(int i = 0; i < 2; i++){
                detected = checkTfod();
                sleep(1000);
            }
            // left spike
            bonkProp(15, 7);
            storageToPickup();
            sleep(20);
            // open claw
            claw.setPosition(1);
            sleep(500);
            pickupToStorage();
            sleep(20);
            endOutput(-OFFSET);
            break;

        }
    }

    public void bonkProp(int back, int forw){
        moveBackward(power, (int)(-(trm.getCurrentPosition() + COUNTS_PER_INCH * back))); // 15
        sleep(20);
        moveForward(power, (int)((trm.getCurrentPosition() + COUNTS_PER_INCH * forw))); // 7
        sleep(20);
    }

    public void endOutput(int deg){
        turn(power, deg);
        sleep(20);
        moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 6));
        sleep(20);
        moveRight(power, (int)(-(trm.getCurrentPosition() + COUNTS_PER_INCH * 102)));
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
        //telemetry.addData("TRM CURR POS (backward) ", trm.getCurrentPosition());
        // orginially greater than or equal to
        // dist = 637.2203205144
        // initial trm position is 180

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

    public void storageToPickup() {
        double alpha = (pot.getVoltage() * 270 / 3.3);
        double beta = joint2.getCurrentPosition();
        while (!aClose(alpha, PICKUP_ALPHA_ANGLE) || !bClose(beta, PICKUP_BETA_TICKS)) {
            alpha = (pot.getVoltage() * 270 / 3.3);
            beta = joint2.getCurrentPosition();

            joint1.setPower(setJoint1Power(alpha, PICKUP_ALPHA_ANGLE));
            joint2.setPower(Range.clip(bP * (PICKUP_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            sleep(20);
        }
    }

    public void pickupToStorage() {
        double alpha = (pot.getVoltage() * 270 / 3.3);
        double beta = joint2.getCurrentPosition();
        while (!aClose(alpha, STORAGE_ALPHA_ANGLE) || !bClose(beta, STORAGE_BETA_TICKS)) {
            alpha = (pot.getVoltage() * 270 / 3.3);
            beta = joint2.getCurrentPosition();

            joint1.setPower(setJoint1Power(alpha, STORAGE_ALPHA_ANGLE));
            joint2.setPower(Range.clip(bP * (STORAGE_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            sleep(20);
        }
    }

    public double setJoint1Power(double alpha, double target) {
        if (aClose(alpha, target)) {
            return 0;
            //a little fudge since it would do the flop
        } else if (target - alpha > 0) {
            return -1 * Math.abs(target-alpha)/alpha;
        } else {
            return 1 * Math.abs(target-alpha)/alpha;
        }
    }

    public boolean aClose(double actual, double target) {
        return Math.abs(target - actual) <= 2;
    }

    public boolean bClose(double actual, double target) {
        return Math.abs(target - actual) <= 10;
    }
}