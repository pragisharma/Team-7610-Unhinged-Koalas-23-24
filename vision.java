package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class Vision extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{

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

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            if(tagProcessor.getDetections().size() > 0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("x: ", tag.ftcPose.x);// x: left and right
                telemetry.addData("y: ", tag.ftcPose.y); // y: straight out
                telemetry.addData("z: ", tag.ftcPose.z); // z: up and down
                telemetry.addData("x: ", tag.ftcPose.pitch); // pitch: x
                telemetry.addData("x: ", tag.ftcPose.roll); // roll: y
                telemetry.addData("x: ", tag.ftcPose.yaw); // yaw: z
                // side notes: bearing: left/right rotation to center, elevation: up/down rotation to center
            }
            telemetry.update();
        }

    }

}