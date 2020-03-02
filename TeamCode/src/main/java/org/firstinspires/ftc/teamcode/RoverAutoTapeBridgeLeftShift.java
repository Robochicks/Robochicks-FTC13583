package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Rover Auto Tape Bridge Left Shift")

public class RoverAutoTapeBridgeLeftShift extends LinearOpMode{

    Robot2019 rover;
    int color_val = 9;

    public void runOpMode() {

        rover = new Robot2019(hardwareMap);
        rover.PrepRobotAuto();

        telemetry.addData("Blue",rover.color_sensor.blue());
        telemetry.addData("Red",rover.color_sensor.red());
        telemetry.addData("Green",rover.color_sensor.green());
        telemetry.addData("status", "searching...");
        telemetry.update();

        waitForStart();

        //while (opModeIsActive()) {

            rover.DriveUntilColor(.5);
            //String Log;

            rover.ClearEncoders();
            rover.SetEncoderDistance(direction.SHIFT_L, 4000);
            rover.SetDrivePower(1,1,1,1,0,0);
            while(rover.isBusy()) {
                rover.logMotors(telemetry);
                telemetry.update();
            }
            rover.SetDrivePower(0,0,0,0,0,0);
            rover.ClearEncoders();


            rover.DriveUntilColor(.5);
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
            //break;



       // }

    }

}

