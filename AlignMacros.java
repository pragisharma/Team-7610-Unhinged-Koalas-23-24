package org.firstinspires.ftc.teamcode.arm;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp(name = "Arm Auto Align")
public class AlignMacros extends LinearOpMode {
    DcMotor fl, fr, bl, br;

    AprilTagProcessor tagProcessor;

    VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        fr = hardwareMap.get(DcMotor.class, "topRight");
        fl = hardwareMap.get(DcMotor.class, "topLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480)) // works well w/o impacting performance
                .build();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.dpad_left) {
                alignHeading();
            } else if (gamepad2.dpad_up) {
                alignDistance();
            } else {
                stopMotors();
            }

            telemetry.addData("Front Left ticks", fl.getCurrentPosition());
            telemetry.addData("Front Right ticks", fr.getCurrentPosition());
            telemetry.addData("Back Left ticks", bl.getCurrentPosition());
            telemetry.addData("Back Right ticks", br.getCurrentPosition());

            telemetry.addLine();

            telemetry.addData("Front Left power", fl.getPower());
            telemetry.addData("Front Right power", fr.getPower());
            telemetry.addData("Back Left power", bl.getPower());
            telemetry.addData("Back Right power", br.getPower());

            telemetry.update();

            sleep(20);
        }
    }

    public void stopMotors() {
        telemetry.addLine("MOTORS STOPPED");
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void alignHeading() {
        telemetry.addLine("ROTATING");

        double heading = 0;
        ArrayList<AprilTagDetection> detections =  tagProcessor.getDetections();
        if(detections.size() >= 2){
            //drive bearings to be equal
            AprilTagDetection tag1 = detections.get(0);
            AprilTagDetection tag2 = detections.get(1);

            double d = Math.abs(tag1.id - tag2.id) * 6 * 2.54; //in cm
            double da = tag1.ftcPose.range;
            double db = tag2.ftcPose.range; //alternate

            double theta = tag1.ftcPose.yaw - tag2.ftcPose.yaw;
            theta = theta / 360 * 2 * Math.PI; //convert to radians???

            telemetry.addData("Distance between tags", d);
            telemetry.addData("Distance to tag 1", da);
            telemetry.addData("Distance to tag 2", db);
            telemetry.addData("Theta", theta);
            telemetry.addData("Tag 1 id", tag1.id);
            telemetry.addData("Tag 2 id", tag2.id);

            heading = tag2.ftcPose.yaw - Math.acos(da * Math.sin(theta) / d); // TODO turn into taylor
        } else if (detections.size() == 1) {
            heading = detections.get(0).ftcPose.bearing; //point towards the tag and hope another one gets in the way

            telemetry.addLine("can only see one tag");
        } else {
            telemetry.addLine("can't see apriltag");
        }
        telemetry.addData("Heading difference", heading);

        double distance = 0;
        if(tagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            distance = tag.ftcPose.y;
            telemetry.addData("Distance", distance);
        } else {
            telemetry.addLine("can't see apriltag");
        }

        double turnPower;
        //turn left if positive
        if (Math.abs(heading) < 0.5) {
            turnPower = 0;
        } else if (Math.abs(heading) < 10) {
            turnPower = Range.clip(0.2 * heading * (Math.abs(distance - 15)/15), -0.2, 0.2);
        } else {
            turnPower = Range.clip(0.2 * heading, -0.2, 0.2);
        }

        if (distance > 15) {
            if (turnPower > 0) {
                fl.setPower(turnPower);
                fr.setPower(0);
                bl.setPower(turnPower);
                br.setPower(0);
            } else {
                fl.setPower(0);
                fr.setPower(-turnPower);
                bl.setPower(0);
                br.setPower(-turnPower);
            }
        } else {
            if (turnPower > 0) {
                fl.setPower(-turnPower);
                fr.setPower(0);
                bl.setPower(-turnPower);
                br.setPower(0);
            } else {
                fl.setPower(0);
                fr.setPower(turnPower);
                bl.setPower(0);
                br.setPower(turnPower);
            }
        }
    }

    public void alignDistance() {
        telemetry.addLine("MOVING FRONT/BACK");

        double distance = 0;
        if(tagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            distance = tag.ftcPose.y;
            telemetry.addData("Distance", distance);
        } else {
            telemetry.addLine("can't see apriltag");
        }

        double drivePower = Range.clip(0.05 * (distance - 15), -0.3, 0.3);

        fl.setPower(drivePower);
        fr.setPower(drivePower);
        bl.setPower(drivePower);
        br.setPower(drivePower);
    }
}
