package org.firstinspires.ftc.teamcode.Auto.Movement;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Auto.Vision.CustomObjectScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "NEW auto red close") // closer to the backdrop
public class Auto_Red_Close_ROI_CV extends LinearOpMode {

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
    OpenCvCamera cam;



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

        // initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        int cameraMonitorViewId;
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        CustomObjectScanner scanner = new CustomObjectScanner();
        cam.setPipeline(scanner);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        while (opModeIsActive()) {

            moveBackward(power, (int)(-(trm.getCurrentPosition() + COUNTS_PER_INCH * 16)));
            sleep(20);

            telemetry.addData("Object color: ", scanner.color());

            if (scanner.color() == 1) {
                telemetry.addLine("red");
            } else {
                telemetry.addLine("error");
            }

            telemetry.update();
            sleep(500);

            if(scanner.color() == 1){
                //case 1 (middle spike - 2)
                moveBackward(power, (int)(-(trm.getCurrentPosition() + COUNTS_PER_INCH * 3)));
                moveLeft(power, (int)(-(trm.getCurrentPosition() + COUNTS_PER_INCH * 48)));
                sleep(20);
                break;
            }

            turn(power, 90 - OFFSET); // 80 degrees cuz gyro is off
            sleep(20);

            telemetry.addData("Object color: ", scanner.color());

            if (scanner.color() == 1) {
                telemetry.addLine("red");
            } else {
                telemetry.addLine("error");
            }

            telemetry.update();
            sleep(500);

            if(scanner.color() == 2){
                // case 2 (right spike - 1)
                moveBackward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 30));
                sleep(20);
                break;
            }

            turn(power, -180 + OFFSET); //  170 degrees cuz gyro is off
            sleep(20);

            telemetry.addData("Object color: ", scanner.color());

            if (scanner.color() == 1) {
                telemetry.addLine("red");
            } else {
                telemetry.addLine("error");
            }

            telemetry.update();
            sleep(500);

            if(scanner.color() == 1) {
                // case 3 (left spike - 3)
                moveRight(power, (int) -(trm.getCurrentPosition() + COUNTS_PER_INCH * 24));
                sleep(20);
                moveForward(power, (int) (trm.getCurrentPosition() + COUNTS_PER_INCH * 40));
                sleep(20);
                break;
            }

            moveBackward(power, (int)-(trm.getCurrentPosition() + COUNTS_PER_INCH * 30));
            telemetry.addLine("OUPUT COMPLETED");
            sleep(20);
            break;
        }

        cam.stopStreaming();

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

}