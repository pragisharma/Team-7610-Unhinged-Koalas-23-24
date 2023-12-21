package org.firstinspires.ftc.teamcode.Auto.Movement;

//import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.IMU;
        import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

        import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Gyro Test")
//Gyro is under the "PROUD" on the Control Hub
//Counter Clockwise => Positive Degrees
//Clockwise => Negative Degrees
//Future: Make program that does not make anything past
//180 degrees turn into -180 degrees and vice versa.
//AKA make it so that it continuously adds degrees past
//180 degrees, and make it continuously decrease past -180 degrees.
public class GyroTest extends LinearOpMode {
    IMU imu;
    IMU.Parameters myIMUparameters;



    Orientation angles;


    @Override
    public void runOpMode() throws InterruptedException {
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

        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation;



        waitForStart();

        while(opModeIsActive()){
            robotOrientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("degrees", robotOrientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();

            sleep(50);
        }

    }
}


