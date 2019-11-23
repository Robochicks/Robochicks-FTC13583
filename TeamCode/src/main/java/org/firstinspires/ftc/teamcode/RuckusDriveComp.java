package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;



@TeleOp(name="RuckusDriveComp")
public class RuckusDriveComp extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor lift;
    private DcMotor slide;
    private DcMotor spin;
    private DcMotor tilt;
    private Blinker expansion_Hub_2;
    private DigitalChannel limit;
    private Blinker expansion_Hub_3;
    private ElapsedTime runtime = new ElapsedTime();
    boolean ArmIsUp = true;
   // private double hookmax = 1;
    //private double hookmin = 0;
    private boolean Moving = false;
   // private DcMotor kickstand;
   // private double kickstandmax = 450;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fl = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        lift = hardwareMap.get(DcMotor.class, "Lift");
        slide = hardwareMap.get(DcMotor.class,"Slide");
        spin = hardwareMap.get (DcMotor.class, "Spin");
        tilt = hardwareMap.get (DcMotor.class, "Tilt");
        limit = hardwareMap.get(DigitalChannel.class, "Limit");


        // hook = hardwareMap. get(Servo.class, "hook");
        // kickstand = hardwareMap.get(DcMotor.class, "Kickstand");

        // kickstand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        spin.setDirection(DcMotor.Direction.FORWARD);
        tilt.setDirection (DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double flPower;
            double frPower;
            double blPower;
            double brPower;
            double liftPower;
            double slidePower = 0;
            double tiltPower = 0;
            double spinPower = 0;

            double y1 = -gamepad1.left_stick_y;
            //double x1 = gamepad1.left_stick_x;
            double x2 = gamepad1.right_stick_x;
            double Opy1 = gamepad2.left_stick_y;
            double Opy2 = gamepad2.right_stick_y;
            int bumper = Boolean.compare(gamepad2.right_bumper, false) - Boolean.compare(gamepad2.left_bumper, false);


            //double y2Operator = gamepad2.right_stick_y;
            flPower = Range.clip(y1 + x2, -1.0, 1.0);
            frPower = Range.clip(y1 - x2, -1.0, 1.0);
            blPower = Range.clip(y1 + x2, -1.0, 1.0);
            brPower = Range.clip(y1 - x2, -1.0, 1.0);
            tiltPower = Range.clip(Opy2, -0.2, 0.2);
            liftPower = 0;

            if (!limit.getState() == false || Opy1 <= 0 ) {
                liftPower = Range.clip(Opy1, -1.0, 1.0);
            }

            //double y2Operator = gamepad2.right_stick_y;
           // flPower = Range.clip(y1 + x1 + x2, -1.0, 1.0);
            //frPower = Range.clip(y1 - x1 - x2, -1.0, 1.0);
            //blPower = Range.clip(y1 - x1 + x2, -1.0, 1.0);
            //brPower = Range.clip(y1 + x1 - x2, -1.0, 1.0);
            //kickPower = Range.clip(y2Operator,-.1,1.0);

            // Left and right bumpers control shifting movements
            if (gamepad1.left_bumper == true) {
                telemetry.addData("Left Bumper", "Pressed");
                frPower =(0.7);
                blPower =(0.7);
                flPower =(-0.7);
                brPower =(-0.7);
            }

            if (gamepad1.right_bumper ==true) {
                telemetry.addData("Right Bumper", "Pressed");
                brPower =(0.7);
                flPower =(0.7);
                blPower =(-0.7);
                frPower =(-0.7);
            }
            //Bumpers control extrusion movement in and out
            if (gamepad2.right_bumper == true){
                telemetry.addData("slide", "Extending");
                slidePower = (0.3);
            }
            if (gamepad2.left_bumper == true){
                telemetry.addData("slide", "Retracting");
                slidePower = (-0.3);
            }
            // The "X" and "A" buttons control the spin intake
            if (gamepad2.x || gamepad2.a == true) {
                telemetry.addData ("x button","Pressed");
                telemetry.addData ("a button","Pressed");
                spinPower = (0.8);
            }
            // The "Y" and "B" buttons control the spin output
            if (gamepad2.y || gamepad2.b == true) {
                telemetry.addData ("y button","Pressed");
                telemetry.addData ("b button", "Pressed");
                spinPower = (-0.8);
            }
            // The right thumb sticks control the tilt of the collector motor


            SetDrivePower(flPower, frPower, blPower, brPower, liftPower,slidePower, spinPower, tiltPower);
        }
    }

    private void SetDrivePower(double FrontLeftPower, double FrontRightPower, double BackLeftPower, double BackRightPower, double LiftPower, double SlidePower, double SpinPower, double TiltPower){
        fl.setPower(FrontLeftPower);
        fr.setPower(FrontRightPower);
        bl.setPower(BackLeftPower);
        br.setPower(BackRightPower);
        lift.setPower(LiftPower);
        slide.setPower(SlidePower);
        spin.setPower(SpinPower);
        tilt.setPower (TiltPower);
       // kickstand.setPower(KickPower);
    }

}