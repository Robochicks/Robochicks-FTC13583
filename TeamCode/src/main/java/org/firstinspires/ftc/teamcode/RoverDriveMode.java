package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="RoverDriveMode")
public class RoverDriveMode extends LinearOpMode {

    Robot rover;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rover = new Robot2019(hardwareMap);


        waitForStart();


        while (opModeIsActive()) {

            String Log;

            Log = rover.DriveFunction(gamepad1, gamepad2);

            telemetry.addData("position", Log);
            telemetry.update();

        }
    }
}
