package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.external.Telemetry;

@Autonomous(name="Ruckus Auto Encoder Hanging")

public class RuckusAutoEncoderHanging extends LinearOpMode {
    private Gyroscope imu;
    private Gyroscope imu_1;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor arm;
    private DcMotor spin;
    private Blinker expansion_Hub_2;
    private Blinker expansion_Hub_3;
    private ElapsedTime runtime = new ElapsedTime();
    private double hookmax = 1;
    private double hookmin = 0;
    private boolean moving = false;
    private Servo hook;

    private DigitalChannel touch;
    private boolean limitswitch = false;
    @Override
    public void runOpMode() {


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fl = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        spin = hardwareMap.get(DcMotor.class, "Spin");
        hook = hardwareMap.get(Servo.class, "hook");
        touch = hardwareMap.get(DigitalChannel.class,"Touch");

        waitForStart();
        arm.setPower(0.3);
        while (limitswitch == false) {
            telemetry.addData("arm", "Moving Down");
            telemetry.update();
            touch.getState();
            if (touch.getState() == false) {
               limitswitch = true;
               telemetry.addData("arm", "Stopped");
               telemetry.update();
               arm.setPower(0.0);
            }
        }



        //telemetry.addData("Movement", "Lowering");

       /* arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(1000);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.2);

        while (arm.isBusy()) {
            telemetry.addData("Mode", "Moving");
            telemetry.addData("Distance ARM", arm.getCurrentPosition());
            telemetry.addData("Busy ARM", arm.isBusy());
            telemetry.update();
        }

        hook.setPosition(0);
        sleep(1500);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(-1000);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.2);

        while (arm.isBusy()) {
            telemetry.addData("Mode", "Moving");
            telemetry.addData("Distance ARM", arm.getCurrentPosition());
            telemetry.addData("Busy ARM", arm.isBusy());
            telemetry.update();
        }

        telemetry.addData("EncoderMovement", "Complete");
        telemetry.update();*/
    }

    private void SetDriveDistance(int FrontLeftDistance, int FrontRightDistance, int BackLeftDistance, int BackRightDistance, double FrontLeftPower, double FrontRightPower, double BackLeftPower, double BackRightPower) {
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bl.setTargetPosition(BackLeftDistance);
        br.setTargetPosition(BackRightDistance);
        fl.setTargetPosition(FrontLeftDistance);
        fr.setTargetPosition(FrontRightDistance);

        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bl.setPower(BackLeftPower);
        br.setPower(BackRightPower);
        fl.setPower(FrontLeftPower);
        fr.setPower(FrontRightPower);

        while (fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy()) {
            telemetry.addData("Mode", "Moving");
            telemetry.addData("Distance BL", bl.getCurrentPosition());
            telemetry.addData("Distance BR", br.getCurrentPosition());
            telemetry.addData("Distance FL", fl.getCurrentPosition());
            telemetry.addData("Distance FR", fr.getCurrentPosition());

            telemetry.addData("Busy BL", bl.isBusy());
            telemetry.addData("Busy BR", br.isBusy());
            telemetry.addData("Busy FL", fl.isBusy());
            telemetry.addData("Busy FR", fr.isBusy());

           /* telemetry.addData( "Distance",  "Front Left (%.2f), Front Right (%.2f), " +
                    "Back Left (%.2f), Back Right (%.2f)",
            fr.getCurrentPosition(),fl.getCurrentPosition(),bl.getCurrentPosition(),br.getCurrentPosition());
            telemetry.addData("Is It Busy?", "Front Left (%.2f), Front Right (%.2f), " +
                            "Back Left (%.2f), Back Right (%.2f)",
                    String.valueOf(fl.isBusy()),  String.valueOf(fr.isBusy()),  String.valueOf(bl.isBusy()), String.valueOf(br.isBusy()));*/
            telemetry.update();

            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            telemetry.addData("EncoderMovement", "Complete");
            telemetry.update();


        }
    }
}