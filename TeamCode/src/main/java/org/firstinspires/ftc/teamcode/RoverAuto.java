package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RuckusAutoDepotNoCraterHanging")

public class RoverAuto extends LinearOpMode{

    Robot rover;

    public void runOpMode() {

        rover = new Robot2019(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            //String Log;

            rover.SetDriveDistance(1000, 1000, 1000,1000,1,1,1,1);

            //telemetry.addData("position", Log);
            telemetry.update();

            break;

        }

    }

}

