package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Thread.sleep;

@Autonomous(name="Rover Auto Tape Wall")

public class RoverAutoTapeWall extends LinearOpMode{

    Robot2019 rover;

    public void runOpMode() {

        rover = new Robot2019(hardwareMap);
        rover.PrepRobotAuto();
        int color_val = 9;

        telemetry.addData("Blue",rover.color_sensor.blue());
        telemetry.addData("Red",rover.color_sensor.red());
        telemetry.addData("Green",rover.color_sensor.green());
        telemetry.addData("status", "searching...");
        telemetry.update();

        waitForStart();

        //while (opModeIsActive()) {

            rover.DriveUntilColor(.5);
            //String Log;
            while (rover.color_sensor.blue() < color_val && rover.color_sensor.red() < color_val){
                try {
                    sleep(100);
                } catch (Exception e){
                    //do nothing
                }
                telemetry.addData("Blue",rover.color_sensor.blue());
                telemetry.addData("Red",rover.color_sensor.red());
                telemetry.addData("Green",rover.color_sensor.green());
                telemetry.addData("status", "searching...");
                telemetry.update();

                if(isStopRequested()){
                    break;
                }
            }
            telemetry.addData("status","found!");
            telemetry.update();
            rover.DriveUntilColor(0);
            //rover.SetDriveDistance(1000, 1000, 1000,1000,1,1,1,1);

            //telemetry.addData("position", Log);
            //rover.DriveUntilColor(telemetry);
            //telemetry.update();
           // break;



        //}

    }

}

