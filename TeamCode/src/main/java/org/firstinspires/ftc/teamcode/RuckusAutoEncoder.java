package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private DcMotor spin;
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
        spin = hardwareMap.get(DcMotor.class, "Spin");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        spin.setDirection(DcMotor.Direction.FORWARD);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("EncoderMovement", "Waiting");
        telemetry.update();

        waitForStart();

        telemetry.addData("EncoderMovement", "Driving Forward");
        telemetry.update();
        sleep(1000);

        SetDriveDistance(4673, 4673, 4673, 4673, 0.6, 0.6, 0.6, 0.6);


        telemetry.addData("EncoderMovement", "Turning");
        telemetry.update();

        sleep(1000);

        spin.setPower(-0.2);
        sleep(2000);
        spin.setPower(0);

        SetDriveDistance(2150, -2150, 2150, -2150, 0.3, 0.3, 0.3, 0.3);//Turn less
        sleep(1000);

        SetDriveDistance(4500, 4500, 4500, 4500, 0.1, 0.1, 0.1, 0.1);//Measure new distance
        //sleep(5000);

        telemetry.addData("EncoderMovement", "Complete");
        telemetry.update();
    }
    private void SetDriveDistance(int FrontLeftDistance, int FrontRightDistance, int BackLeftDistance, int BackRightDistance, double FrontLeftPower, double FrontRightPower, double BackLeftPower, double BackRightPower){
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

        while(fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy()) {
            telemetry.addData("Mode" , "Moving");
            telemetry.addData( "Distance BL", bl.getCurrentPosition());
            telemetry.addData( "Distance BR", br.getCurrentPosition());
            telemetry.addData( "Distance FL", fl.getCurrentPosition());
            telemetry.addData( "Distance FR", fr.getCurrentPosition());

            telemetry.addData( "Busy BL", bl.isBusy());
            telemetry.addData( "Busy BR", br.isBusy());
            telemetry.addData( "Busy FL", fl.isBusy());
            telemetry.addData( "Busy FR", fr.isBusy());

           /* telemetry.addData( "Distance",  "Front Left (%.2f), Front Right (%.2f), " +
                    "Back Left (%.2f), Back Right (%.2f)",
            fr.getCurrentPosition(),fl.getCurrentPosition(),bl.getCurrentPosition(),br.getCurrentPosition());
            telemetry.addData("Is It Busy?", "Front Left (%.2f), Front Right (%.2f), " +
                            "Back Left (%.2f), Back Right (%.2f)",
                    String.valueOf(fl.isBusy()),  String.valueOf(fr.isBusy()),  String.valueOf(bl.isBusy()), String.valueOf(br.isBusy()));*/
            telemetry.update();
        }

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("EncoderMovement", "Complete");
        telemetry.update();


    }
}