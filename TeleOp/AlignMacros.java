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

@TeleOp(name = "Arm Auto Align")
public class AlignMacros extends LinearOpMode {
    DcMotor fl, fr, bl, br;

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

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                alignHeading();
            } else if (gamepad1.dpad_up) {
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
        if(tagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            heading = tag.ftcPose.yaw;
        }

        //turn left if positive
        double power = Range.clip(0.1 * heading, -0.5, 0.5);

        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(-power);
        br.setPower(power);
    }

    public void alignDistance() {
        telemetry.addLine("MOVING FRONT/BACK");

        double distance = 0;
        if(tagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            distance = tag.ftcPose.y;
        }

        double power = Range.clip(0.1 * (distance - 6), -0.5, 0.5);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
}
