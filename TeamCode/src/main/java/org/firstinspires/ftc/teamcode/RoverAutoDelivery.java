package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Thread.sleep;

@Autonomous(name="Rover Auto Delivery")

public class RoverAutoDelivery extends LinearOpMode{

    Robot2019 rover;

    public void runOpMode() {

        rover = new Robot2019(hardwareMap);
        rover.PrepRobotAuto();

        waitForStart();

        //reset encoders and drive FORWARD a distance
        rover.ClearEncoders();
        rover.SetEncoderDistance(4628, 4628, 4628, 4628);
        rover.SetDrivePower(1,1,1,1,0,0);


        //this will log the motor outputs while we wait for the position to be found.
        while(rover.isBusy()) {
            rover.logMotors(telemetry);
            telemetry.update();
        }

        //this just waits for 3 seconds
        try {
            telemetry.addData("Mode", "Waiting");
            telemetry.update();
            sleep(3000);
        } catch (Exception e){
            //do nothing
        }

        //stop and clear encoders
        rover.SetDrivePower(0,0,0,0,0,0);
        rover.ClearEncoders();

        //drive BACKWARDS that distance
        rover.SetEncoderDistance(-4628, -4628, -4628, -4628);
        rover.SetDrivePower(1,1,1,1,0,0);

        //this will log the motor outputs while we wait for the position to be found.
        while(rover.isBusy()) {
            rover.logMotors(telemetry);
            telemetry.update();
        }

        //stop and clear encoders
        rover.SetDrivePower(0,0,0,0,0,0);
        rover.ClearEncoders();


        telemetry.addData("Mode" , "Done!");
        telemetry.update();

        /**
         * MIA: go ahead and finish the program to drive to the tape and drop off the box.
         * Perfect the turns and distances first, the color finding should be easy once that's done
         */
        /*rover.SetDriveTime(telemetry,1,0,0,0,0,1.0);

            rover.SetDriveDistance(telemetry,4628,4628,4628,4628,1.0,1.0,1.0,1.0);

            rover.SetDriveTime(telemetry,1,0,0,0,0,-1.0);

            rover.SetDriveDistance(telemetry,-560,560,-560,560,1.0,1.0,1.0,1.0);

            rover.SetDriveDistance(telemetry,1070,1070,1070,1070, 1.0,1.0,1.0,1.0);

            rover.SetDriveDistance(telemetry,-280,280,-280,280,1.0,1.0,1.0,1.0);
*/
            //rover.DriveUntilColor(.5);

            //String Log;
           /* while (rover.color_sensor.blue() < 7 && rover.color_sensor.red() < 7){
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
            //telemetry.update();*/
            //break;

    }

}

