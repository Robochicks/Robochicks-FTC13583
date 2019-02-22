package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="RuckusAutoDepotNoCraterHanging")
@Disabled

public class RuckusAutoDepotNoCraterHanging extends LinearOpMode {

//import com.qualcomm.robotcore.external.Telemetry;

    // private Gyroscope imu;

    //private Gyroscope imu_1;

    private DcMotor fl;

    private DcMotor fr;

    private DcMotor bl;

    private DcMotor br;

    private DcMotor lift;

    private DcMotor Spin;

    boolean IsMagnetSensed;

    private DigitalChannel limit;

    // private Blinker expansion_Hub_2;

    // private Blinker expansion_Hub_3;

    //private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;



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

        lift = hardwareMap.get(DcMotor.class, "Lift");

        Spin = hardwareMap.get(DcMotor.class, "Spin");

        limit = hardwareMap.get(DigitalChannel.class, "Limit");


        // Most robots need the motor on one side to be reversed to drive forward

        // Reverse the motor that runs backwards when connected directly to the battery

        fl.setDirection(DcMotor.Direction.REVERSE);

        fr.setDirection(DcMotor.Direction.FORWARD);

        bl.setDirection(DcMotor.Direction.REVERSE);

        br.setDirection(DcMotor.Direction.FORWARD);

        lift.setDirection(DcMotor.Direction.FORWARD);

        Spin.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limit.setMode(DigitalChannel.Mode.INPUT);


        telemetry.addData("EncoderMovement", "Waiting");

        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            telemetry.update();

            // lower robot
            DetachFromLander();

            telemetry.addData("EncoderMovement", "Driving Forward");

            telemetry.update();
            telemetry.addData("EncoderMovement", "Driving Forward");
            telemetry.update();

            // driving forward to the depot
            SetDriveDistance(4200, 4200, 4200, 4200, 0.8, 0.8, 0.8, 0.8);


            telemetry.addData("EncoderMovement", "Turning");
            telemetry.update();

            //Ejecting the marker
            Spin.setPower(-1);
            sleep(1000);
            Spin.setPower(0);

            //Drive Backwards
            SetDriveDistance(-2100, -2100, -2100, -2100, 0.8, 0.8, 0.8, 0.8);
        }
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
    private void DetachFromLander () {
        lift.setPower(0.1);
        while (IsMagnetSensed == false) {

            telemetry.addData("Mode", "Lowering robot");
            telemetry.update();

            IsMagnetSensed = limit.getState();
        }

        lift.setPower(0);
        //strafe off the hook then turn around
        SetDriveDistance(-300, 300, 300, -300, 0.1, 0.1, 0.1, 0.1);

        SetDriveDistance(-2869, 2869, -2869, 2869, 0.8, 0.8, 0.8, 0.8);

    }
}
