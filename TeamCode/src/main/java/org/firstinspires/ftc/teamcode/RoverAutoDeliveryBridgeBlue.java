package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Rover Auto Delivery Bridge Blue")

//TODO: cmake two versions, one "close" one "far" lane
public class RoverAutoDeliveryBridgeBlue extends LinearOpMode{

    Robot2019 rover;

    public void runOpMode() {

        rover = new Robot2019(hardwareMap);
        rover.PrepRobotAuto();

        waitForStart();

        //reset encoders and drive FORWARD a distance

        rover.SetDriveTime(telemetry,.75,0,0,0,0,-1);

        rover.ClearEncoders();
        rover.SetEncoderDistance(direction.FORWARD, 6500);
        rover.SetDrivePower(1,1,1,1,0,0);
        //this will log the motor outputs while we wait for the position to be found.
        while(rover.isBusy()) {
            rover.logMotors(telemetry);
            telemetry.update();
        }
        //stop and clear encoders
        rover.SetDrivePower(0,0,0,0,0,0);
        rover.ClearEncoders();


        rover.SetEncoderDistance(direction.TURN_L,3200);
        rover.SetDrivePower(1,1,1,1,0,0);
        while(rover.isBusy()) {
            rover.logMotors(telemetry);
            telemetry.update();
        }
        rover.SetDrivePower(0,0,0,0,0,0);
        rover.ClearEncoders();


        //drive BACKWARDS that distance
        rover.SetEncoderDistance(direction.SHIFT_L, 2600);
        rover.SetDrivePower(1,1,1,1,0,0);
        //this will log the motor outputs while we wait for the position to be found.
        while(rover.isBusy()) {
            rover.logMotors(telemetry);
            telemetry.update();
        }
        //stop and clear encoders
        rover.SetDrivePower(0,0,0,0,0,0);
        rover.ClearEncoders();


        rover.SetEncoderDistance(direction.FORWARD, 7700);
        rover.SetDrivePower(1,1,1,1,0,0);
        while(rover.isBusy()) {
            rover.logMotors(telemetry);
            telemetry.update();
        }
        rover.SetDrivePower(0,0,0,0,0,0);
        rover.ClearEncoders();

        rover.SetEncoderDistance(direction.BACKWARD, 3858);
        rover.SetDrivePower(1,1,1,1,0,0);
        while(rover.isBusy()) {
            rover.logMotors(telemetry);
            telemetry.update();
        }
        rover.SetDrivePower(0,0,0,0,0,0);
        rover.ClearEncoders();

        telemetry.addData("Mode" , "Done!");
        telemetry.update();

    }

}

