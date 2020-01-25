package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Rover Auto")

public class RoverAuto extends LinearOpMode{

    Robot2019 rover;

    public void runOpMode() {

        rover = new Robot2019(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            //String Log;

            //rover.SetDriveDistance(1000, 1000, 1000,1000,1,1,1,1);

            //telemetry.addData("position", Log);
            rover.DriveUntilColor(telemetry);
            telemetry.update();
            break;



        }

    }

}

