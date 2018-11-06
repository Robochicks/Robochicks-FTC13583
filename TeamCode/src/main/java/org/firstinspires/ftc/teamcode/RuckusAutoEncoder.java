package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.external.Telemetry;

@Autonomous(name="RuckusAutoEncoder")

public class RuckusAutoEncoder extends LinearOpMode {
    private Gyroscope imu;
    private Gyroscope imu_1;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor arm;
    private Blinker expansion_Hub_2;
    private Blinker expansion_Hub_3;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fl  = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        arm = hardwareMap.get(DcMotor.class, "Arm");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        //Reset all encoders to prevent inaccurate movements; Wait for the game to start (driver presses PLAY)
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("EncoderMovement", "Waiting");
        telemetry.update();

        waitForStart();


        telemetry.addData("EncoderMovement", "Driving Forward");
        telemetry.update();


        // Giving all the motors a specific movement/rotation and speed with the encoder
        // Mia 10/29 making the change of a longer path so we can make it to the depot
        bl.setTargetPosition(4020);
        br.setTargetPosition(4020);
        fl.setTargetPosition(4020);
        fr.setTargetPosition(4020);

        // Preparing all the encoders on all of the motors to initiate movement
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bl.setPower(0.2);
        br.setPower(0.2);
        fl.setPower(0.2);
        fr.setPower(0.2);

        while(fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) {
            telemetry.addData("Mode" ,"Moving to position");
            telemetry.addData("Distance", bl.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("EncoderMovement", "Stop");
        telemetry.update();

        //Stopping the encoders to ensure accurate measurements
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("EncoderMovement", "Start Turn");
        telemetry.update();


        //Here I am trying to turn the robot
        bl.setTargetPosition(700);
        br.setTargetPosition(-700);// GET RID OF THE NEGATIVE!
        fl.setTargetPosition(700);
        fr.setTargetPosition(-700);

        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bl.setPower(0.2);
        br.setPower(0.2);
        fl.setPower(0.2);
        fr.setPower(0.2);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy()) {
            telemetry.addData("Mode" , "Turning");
            telemetry.addData( "Distance", bl.getCurrentPosition());
            telemetry.addData( "Distance", br.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("EncoderMovement", "Complete");
        telemetry.update();
    }

}