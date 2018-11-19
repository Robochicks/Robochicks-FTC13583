package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@TeleOp(name="Ruckus_Drive_Competition")

public class RuckusDriveComp extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor arm;
    private Blinker expansion_Hub_2;
    private Blinker expansion_Hub_3;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor spin;
    private Servo hook;
    boolean ArmIsUp = true;
    private double hookmax = 1;
    private double hookmin = 0;
    private boolean Moving = false;
    private DcMotor kickstand;
    private double kickstandmax = 450;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fl  = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class,  "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        spin = hardwareMap.get(DcMotor.class, "Spin");
        hook = hardwareMap. get(Servo.class, "hook");
        kickstand = hardwareMap.get(DcMotor.class, "Kickstand");

        kickstand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        spin.setDirection(DcMotor.Direction.FORWARD);


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
            double armPower;
            double spinPower;
            double kickstandPower;
            //int kickpos;
            double kickPower;

            if (hook.getPosition()== hookmax || hook.getPosition()== hookmin) {
                Moving = false;
            }
            else { Moving = true;
            }

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            // Gamepad1 is the driver's controller
            double y1 = -gamepad1.left_stick_y;
            double x1 = gamepad1.left_stick_x;
            double x2 = gamepad1.right_stick_x;
            double y1Operator = gamepad2.left_stick_y;
            double y2Operator = gamepad2.right_stick_y;
            flPower = Range.clip(y1 + x1 - x2, -1.0, 1.0);
            frPower = Range.clip(y1 - x1 + x2, -1.0, 1.0);
            blPower = Range.clip(y1 - x1 - x2, -1.0, 1.0);
            brPower = Range.clip(y1 + x1 + x2, -1.0, 1.0);
            armPower = Range.clip(y1Operator, -1.0, 1.0);
            //spinPower = Range.clip(y2Operator, -1.0, 1.0);
            kickPower = Range.clip(y2Operator,-.1,1.0);

            //This controls the hook. Pressing the bumper once will either open or close it.
            if (gamepad2.right_bumper == true && Moving == false) {
                telemetry.addData("Right Bumper", "Pressed");
                if (hook.getPosition() == hookmax) {
                    hook.setPosition(hookmin);
                } else {
                    hook.setPosition(hookmax);
                }
            } else {
                telemetry.addData("Right Bumper", "Unpressed");
            }
            telemetry.update();

            /*if (gamepad2.left_bumper == true) {
                telemetry.addData("Left Bumper", "Pressed");
                if (kickstand.getCurrentPosition()>0) {
                    kickpos = -440;
                } else {
                    kickpos = 440;
                }
                kickstand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                kickstand.setTargetPosition(kickpos);
                kickstand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                kickstand.setPower(0.5);
                while (kickstand.isBusy()) {
                    telemetry.addData("Kickstand", "Moving");
                    telemetry.update();
                }
            }*/
            /*if (gamepad2.left_bumper == true){
                if ( ArmIsUp == true) {
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setTargetPosition(160);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.1);
                    while (arm.isBusy()) {
                        telemetry.addData("Status", "Lowering Arm");
                        telemetry.update();

                    }
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ArmIsUp = false;
                }
                else if ( ArmIsUp == false) {
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setTargetPosition(-160);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.1);
                    while(arm.isBusy()){
                        telemetry.addData("Status", "Raising Arm");
                        telemetry.update();

                    }
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ArmIsUp = true;
                }
            }*/



            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            SetDrivePower(flPower, frPower, blPower, brPower, armPower, 0,kickPower);
            // Send calculated power to wheels


            // Show the elapsed game time and wheel power
            // telemetry.addData("Status", "Run Time: " + runtime.toString());
           // telemetry.addData("Motors", "Front Left (%d), Front Right (%d), " +
           //                 "Back Left (%d), Back Right (%d)",
           //         flPower, frPower, blPower, brPower);
           // telemetry.addData("Mechanisms", "Arm (%d), Spin (%d), " +
           //     armPower, spinPower);
           // telemetry.addData("Raw Inputs (Gamepad 1)","X1 (%.2f), Y1 (.2f), X2 (.2f)",x1,y1,x2);
            //telemetry.addData("Raw Inputs (Gamepag 2)","Y1 (%.2f), Y2 (%.2f");
           // telemetry.update();
        }
    }

    private void SetDrivePower(double FrontLeftPower, double FrontRightPower, double BackLeftPower, double BackRightPower, double ArmPower, double SpinPower, double KickPower){
        fl.setPower(FrontLeftPower);
        fr.setPower(FrontRightPower);
        bl.setPower(BackLeftPower);
        br.setPower(BackRightPower);
        arm.setPower(ArmPower);
        spin.setPower(SpinPower);
        kickstand.setPower(KickPower);
    }

}