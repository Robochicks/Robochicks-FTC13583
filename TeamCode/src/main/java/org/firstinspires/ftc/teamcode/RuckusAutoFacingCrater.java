package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Blinker;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



import java.util.Set;

//import com.qualcomm.robotcore.external.Telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.*;

@Autonomous(name="RuckusAutoFacingCrater")



public class RuckusAutoFacingCrater extends LinearOpMode {

    //private Gyroscope imu;

    //private Gyroscope imu_1;

    private BNO055IMU imu;

    private DcMotor fl;

    private DcMotor fr;

    private DcMotor bl;

    private DcMotor br;

    private DcMotor arm;

    private DcMotor Spin;

    private Orientation lastAndle = new Orientation();

    private double globalAngle, power = .30, correction;

    //  private Blinker expansion_Hub_2;

    // private Blinker expansion_Hub_3;

    //private ElapsedTime runtime = new ElapsedTime();



    @Override

    public void runOpMode() {





        // Send telemetry message to signify robot waiting;

        telemetry.addData("Status", "Ready to run");    //

        telemetry.update();



        // Initialize the hardware variables. Note that the strings used here as parameters

        // to 'get' must correspond to the names assigned during the robot configuration

        // step (using the FTC Robot Controller app on the phone).

        fl = hardwareMap.get(DcMotor.class, "FL");

        fr = hardwareMap.get(DcMotor.class, "FR");

        bl = hardwareMap.get(DcMotor.class, "BL");

        br = hardwareMap.get(DcMotor.class, "BR");

        arm = hardwareMap.get(DcMotor.class, "Arm");

        Spin = hardwareMap.get(DcMotor.class, "Spin");



        imu = hardwareMap.get(BNO055IMU.class,"imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();

        params.mode                = BNO055IMU.SensorMode.IMU;

        params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        params.loggingEnabled      = false;

        imu.initialize(params);

        telemetry.addData("Mode", "calibrating...");

        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())

        {

            sleep(50);

            idle();

        }

        telemetry.addData("Mode", "calibrated, waiting...");

        telemetry.update();



        // Most robots need the motor on one side to be reversed to drive forward

        // Reverse the motor that runs backwards when connected directly to the battery

        fl.setDirection(DcMotor.Direction.REVERSE);

        fr.setDirection(DcMotor.Direction.FORWARD);

        bl.setDirection(DcMotor.Direction.REVERSE);

        br.setDirection(DcMotor.Direction.FORWARD);

        arm.setDirection(DcMotor.Direction.FORWARD);

        Spin.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        telemetry.addData("EncoderMovement", "Waiting");

        telemetry.update();



        waitForStart();



        telemetry.addData("EncoderMovement", "Driving Forward");

        telemetry.update();

        //sleep(1000);



        // Movement A (drive forward from shuttle)

        SetDriveDistance(1460, 1460, 1460, 1460, 0.8, 0.8, 0.8, 0.8);

        //sleep(1000);



        /*Spit out marker

        spin.setPower(-0.2);

        sleep(2000);

        spin.setPower(0);**/



        // Movement A ~> B (turn)

        telemetry.addData("EncoderMovement", "Turning");

        telemetry.update();



        //SetDriveDistance(-1469, 1469, -1469, 1469, 0.4, 0.4, 0.4, 0.4);

        //sleep(1000



        SetTurn(0.4, 75);



        SetDriveDistance(3384, 3384, 3384, 3384, 0.8, 0.8, 0.8, 0.8);



        SetTurn(0.4, 70);



        SetDriveDistance(3200, 3200, 3200, 3200, 0.8,0.8,0.8,0.8);



        //Ejecting the marker

        Spin.setPower(-1);

        sleep(1000);

        Spin.setPower(0);



        SetTurn(0.4, 180);



        SetDriveDistance(7091, 7091,7091,7091,0.8, 0.8, 0.8, 0.8);

    }



    private void SetTurn (double Power, double Angle){

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double LeftPower, RightPower;

        if (Angle > 0){

            LeftPower = -Power;

            RightPower = Power;

        }

        else if(Angle < 0){

            LeftPower = Power;

            RightPower = -Power;

        }

        else{

            LeftPower = Power;

            RightPower = Power;

        }



        fr.setPower(RightPower);

        fl.setPower(LeftPower);

        br.setPower(RightPower);

        bl.setPower(LeftPower);

        Orientation Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double Heading = Angles.firstAngle;

        if(Angle < 0){

            while(Heading > Angle){

                Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                Heading = Angles.firstAngle;

                telemetry.addData("EncoderMovement", "Turning Right");

                telemetry.addData("Turn Info", LeftPower + "," + RightPower);

                telemetry.addData("Turn Info2", Angle + "," + Heading);

                telemetry.update();

            }

        }

        else if(Angle > 0){

            while(Heading < Angle){

                Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                Heading = Angles.firstAngle;

                telemetry.addData("EncoderMovement", "Turning Left");

                telemetry.addData("Turn Info", LeftPower + "," + RightPower);

                telemetry.addData("Turn Info2", Angle + "," + Heading);

                telemetry.update();

            }

        }

        fr.setPower(0);

        fl.setPower(0);

        br.setPower(0);

        bl.setPower(0);



        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);



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