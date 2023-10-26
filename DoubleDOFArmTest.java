package org.firstinspires.ftc.teamcode;

import android.util.Pair;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Two DoF Arm Test")
public class DoubleDOFArmTest extends LinearOpMode {
    // -- code helpers --
    //just so the arm doesn't mistake 0 for an actual value
    final double INFINITY = 9999999999.0;
    //since I don't trust the computer on several trig calculations
    final double TOLERANCE = Math.pow(10,-4);

    //TODO 2.0: add potentiometers?

    // -- arm specs --
    //the unit doesn't matter as long as you are consistent with them.
    //inches here.
    final double R1 = 18;
    final double R2 = 18;
    final double ARM_HEIGHT = 12;

    //remember to account for sprockets, pulleys, and gears!
    final double TICKS_PER_MOTOR_ROTATION_1 = 1120;
    final double TICKS_PER_MOTOR_ROTATION_2 = 1120;

    //when you're not doing anything, please don't break the arm
    final double STORAGE_ALPHA = -Math.acos(12/ R1);
    final double STORAGE_BETA = Math.PI/2 - Math.asin(12/ R1);

    // -- field specs --
    //backdrop (and field) heights relative to tiles
    final double FIELD_FLOOR = 0.0;
    final double FIRST_LINE = 12.375;
    final double SECOND_LINE = 19.0;
    final double THIRD_LINE = 25.75;

    //same as above, relative to first pivot point
    final double GROUND_POSITION = FIELD_FLOOR - ARM_HEIGHT;
    final double FIRST_POSITION = FIRST_LINE - ARM_HEIGHT;
    final double SECOND_POSITION = SECOND_LINE - ARM_HEIGHT;
    final double THIRD_POSITION = THIRD_LINE - ARM_HEIGHT;

    // -- electronics setup: motors and webcams --
    //motors
    DcMotor topLeft, topRight, bottomLeft, bottomRight;
    DcMotor joint1, joint2;

    //set up vision for x
    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .build();

    VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
            .setCameraResolution(new Size(640, 480)) // works well w/o impacting performance
            .build();

    // -- not exactly code stuff --
    //variables to compute
    double x,h;
    double alpha,beta, alphaTicks, betaTicks;

    public void runOpMode() {
        //drive motor setup
        topLeft = hardwareMap.get(DcMotor.class, "top left");
        topRight = hardwareMap.get(DcMotor.class, "top right");
        bottomLeft = hardwareMap.get(DcMotor.class, "bottom left");
        bottomRight = hardwareMap.get(DcMotor.class, "bottom right");

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //arm motor setup
        joint1 = hardwareMap.get(DcMotor.class, "joint 1");
        joint2 = hardwareMap.get(DcMotor.class, "joint 2");

        waitForStart();
        while (opModeIsActive()) {
            /*
            if (gamepad2.left_bumper) {
                alignChassis();
            }
            */
            moveArm();

            //telemetry
            telemetry.addData("x: ", x);
            telemetry.addData("h: ", h);
            telemetry.addData("alpha: ", alpha);
            telemetry.addData("beta: ", beta);
            telemetry.update();

            sleep(20);
        }
    }

    //call moveArm() and alignChassis() in main loop
    public void moveArm() {
        if (!gamepad2.a && !gamepad2.y && !gamepad2.x && !gamepad2.y) {
            alpha = STORAGE_ALPHA;
            beta = STORAGE_BETA;
        } else {
            h = setH();
            x = setX();

            alpha = calcAB(R1, R2, x, h).first;
            beta = calcAB(R1, R2, x, h).second;
        }

        //modify x to accout for the 60 degree slant on the backdrop
        x += h / (Math.tan(Math.toRadians(60)));

        //specific to 7610: due to joint1 being on the first level, adjust beta since it's also affected by alpha
        beta -= alpha;

        //modify alpha and beta so storage = 0
        alpha -= STORAGE_ALPHA;
        beta -= STORAGE_BETA;

        //convert alpha and beta to ticks
        alphaTicks = TICKS_PER_MOTOR_ROTATION_1 * alpha / (2 * Math.PI);
        betaTicks = TICKS_PER_MOTOR_ROTATION_2 * beta / (2 * Math.PI);

        //move the motors with p control
        joint1.setPower(0.01 * (alphaTicks - joint1.getCurrentPosition())); //TODO tune
        joint2.setPower(0.01 * (betaTicks - joint2.getCurrentPosition())); //TODO tune
    }

    /*
    //SCUFFED VERSION USE GYRO NEXT TIME
    public void alignChassis() {
        double toTurn = 0;
        if(tagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            toTurn = tag.ftcPose.bearing;
            // side notes: bearing: left/right rotation to center
        }
        double power = toTurn * 0.01;
        // CW if power +, check
        topLeft.setPower(power);
        topRight.setPower(-power);
        bottomLeft.setPower(power);
        bottomRight.setPower(-power);
    }
    */

    //a ton of "helper" methods that do the actual heavy lifting
    public double setH() {
        if (gamepad2.a) {
            return GROUND_POSITION;
        } else if (gamepad2.b) {
            return FIRST_POSITION;
        } else if (gamepad2.x) {
            return SECOND_POSITION;
        } else if (gamepad2.y) {
            return THIRD_POSITION;
        } else {
            return INFINITY;
        }
    }

    public double setX() {
        if(tagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            return tag.ftcPose.x;
            // side notes: bearing: left/right rotation to center, elevation: up/down rotation to center
        }
        return INFINITY;
    }

    public boolean checkAB(double alpha, double beta) {
        return Math.abs(Math.cos(alpha-beta) - (h*h + x*x - R1 * R1 - R2 * R2)/(2* R1 * R2)) <= TOLERANCE;
    }

    public Pair<Double, Double> calcAB(double r1, double r2, double x, double h) {
        //will automatically use higher alpha value
        double a = 2*x;
        double b = 2*h;
        double cx = (h*h + x*x + r1*r1 - r2*r2)/(r2);
        double cy = (h*h + x*x - r1*r1 + r2*r2)/(r2);

        double alphaPos = Math.acos(cx/(Math.sqrt(a*a + b*b))) + Math.atan(b/a);
        double alphaNeg = -Math.acos(cx/(Math.sqrt(a*a + b*b))) + Math.atan(b/a);

        double betaPos = Math.acos(cy/(Math.sqrt(a*a + b*b))) + Math.atan(b/a);
        double betaNeg = -Math.acos(cy/(Math.sqrt(a*a + b*b))) + Math.atan(b/a);

        //check each alpha, beta pair
        if (checkAB(alphaPos, betaPos)) {
            return new Pair<>(alphaPos, betaPos);
        }
        else if (checkAB(alphaPos, betaNeg)) {
            return new Pair<>(alphaPos, betaNeg);
        }
        else if (checkAB(alphaNeg, betaPos)) {
            return new Pair<>(alphaNeg, betaPos);
        }
        else if (checkAB(alphaNeg, betaNeg)) {
            return new Pair<>(alphaNeg, betaNeg);
        }
        else {
            //no solutions --> keep in storage position
            return new Pair<>(STORAGE_ALPHA,STORAGE_BETA);
        }
    }
}
