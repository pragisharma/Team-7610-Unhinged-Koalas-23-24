package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Pixel Intake Auto")
public class Webcam extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");


        waitForStart();

        while(opModeIsActive()){

        }
    }




}
