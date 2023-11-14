package org.firstinspires.ftc.teamcode;

import android.util.Size;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp
public class visionv2 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        // have a custom game element be yellow cone

        OpenCvCamera cam;
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        AprilTagProcessor aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera, aprilTagProcessor);


        waitForStart();
        while(opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            telemetry.addData("num detections", detections.size());

                for (AprilTagDetection detection : detections) {
                    int id = detection.id;
                    AprilTagPoseFtc tagPose = detection.ftcPose;
                    telemetry.addLine("Detection tag ID: " + id);
                    telemetry.addLine("Distance to tag: " + tagPose.range);
                    telemetry.addLine("Bearing to tag: " + tagPose.bearing);
                    telemetry.addLine("Angle to tag " + tagPose.yaw);
                }
            }
            telemetry.update();
            sleep(50);


    }

}