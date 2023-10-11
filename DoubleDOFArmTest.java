package org.firstinspires.ftc.teamcode;

import android.util.Pair;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class DoubleDOFArmTest extends LinearOpMode {
    DcMotor joint1, joint2;

    final double r1 = 18;
    final double r2 = 18;
    double x,h;
    double alpha,beta, alphaTicks, betaTicks;

    final double STORAGE_ALPHA = -Math.acos(12/r1);
    final double STORAGE_BETA = Math.PI/2 - Math.asin(12/r1);

    final double GROUND_POSITION = -12;
    final double FIRST_POSITION = 0;
    final double SECOND_POSITION = 12;
    final double THIRD_POSITION = 24;

    final double TICKS_PER_ROTATION = 1120;

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

    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.atRest()) {
                alpha = STORAGE_ALPHA;
                beta = STORAGE_BETA;
            } else {
                h = setH();
                x = setX();

                alpha = calcAB(r1, r2, x, h).first;
                beta = calcAB(r1, r2, x, h).second;
            }
            //convert alpha and beta to ticks
            alphaTicks = TICKS_PER_ROTATION * alpha / (2 * Math.PI);
            betaTicks = TICKS_PER_ROTATION * beta / (2 * Math.PI);

            //move the motors with p control
            joint1.setPower(0.01 * (alphaTicks - joint1.getCurrentPosition()));
            joint2.setPower(0.01 * (betaTicks - joint2.getCurrentPosition()));

            //telemetry
            telemetry.addData("x: ", x);
            telemetry.addData("h: ", h);
            telemetry.addData("alpha: ", alpha);
            telemetry.addData("beta: ", beta);
            telemetry.update();

            sleep(20);
        }
    }

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
            return 0;
        }
    }

    public double setX() {
        if(tagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            return tag.ftcPose.x;
            // side notes: bearing: left/right rotation to center, elevation: up/down rotation to center
        }
        return 0;
    }

    public boolean checkAB(double alpha, double beta) {
        //might need to add tolerance to this
        return Math.cos(alpha-beta) == (h*h + x*x - r1*r1 - r2*r2)/(2*r1*r2);
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
            //no solutions
            return new Pair<>(0.0,0.0);
        }
    }
}
