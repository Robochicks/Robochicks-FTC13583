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


//import com.qualcomm.robotcore.external.Telemetry;



    @Autonomous(name="RuckusFacingCraterHanging")
    @Disabled


    public class RuckusAutoFacingCraterHanging extends LinearOpMode {

        // private Gyroscope imu;

        //private Gyroscope imu_1;

        private DcMotor fl;

        private DcMotor fr;

        private DcMotor bl;

        private DcMotor br;

        private DcMotor lift;

        private DcMotor Spin;

        boolean IsMagnetSensed = false;

        private DigitalChannel limit;

        // private Blinker expansion_Hub_2;

        // private Blinker expansion_Hub_3;

        //private ElapsedTime runtime = new ElapsedTime();
        //BNO055IMU imu;
        Orientation             lastAngles = new Orientation();
        double globalAngle, power = .30, correction;



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

            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            limit.setMode(DigitalChannel.Mode.INPUT);



            telemetry.addData("EncoderMovement", "Waiting");

            telemetry.update();

        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        */

            waitForStart();

            while (opModeIsActive()) {
                //telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
                //telemetry.update();

                // lower robot
                DetachFromLander();

                telemetry.addData("EncoderMovement", "Driving Forward");

                telemetry.update();

                // driving forward to the depot

                // Movement A (drive forward from shuttle)
                //sleep(1000);

        /*Spit out marker
        spin.setPower(-0.2);
        sleep(2000);
        spin.setPower(0);**/

                // Movement A ~> B (turn)
                telemetry.addData("EncoderMovement", "Turning");
                telemetry.update();

                telemetry.addData("EncoderMovement", "Heading to depot");
                telemetry.update();

                // Movement B (move forward)
                SetDriveDistance(3984, 3984, 3984, 3984, 0.8, 0.8, 0.8, 0.8);//Measure new distance
                //sleep(5000);

                //telemetry.addData("EncoderMovement, ")
                // Movement B ~> C (turn)
                SetDriveDistance(-750, 750, -750, 750, 0.8,0.8, 0.8, 0.8);

                //Movement C (move forward towards depot)
                SetDriveDistance(3200, 3200, 3200, 3200, 0.4,0.4,0.4,0.4);

                //Ejecting the marker
                Spin.setPower(1);
                sleep(1500);
                Spin.setPower(0);

                telemetry.addData("EncoderMovement", "Heading to crater");
                telemetry.update();


                //Movement D (180 turn)
                SetDriveDistance(-2869, 2869, -2869,2869,0.8,0.8,0.8,0.8);

                //Movement E (move forward into crater)
                SetDriveDistance(6091, 6091,6091,6091,0.8, 0.8, 0.8, 0.8);



                telemetry.addData("EncoderMovement", "Complete");

                telemetry.update();
                break;

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
    /*private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
*/
        /**
         * Get current cumulative angle rotation from last reset.
         * @return Angle in degrees. + = left, - = right.
         */
    /*private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    */

        /**
         * See if we are moving in a straight line and if not return a power correction value.
         * @return Power adjustment, + is adjust left - is adjust right.
         */
    /*private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
*/

    /*private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        bl.setPower(leftPower);

        br.setPower(rightPower);

        fl.setPower(leftPower);

        fr.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        bl.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        fr.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    /*

     */
        private void DetachFromLander () {
            lift.setPower(-0.7);
            while (IsMagnetSensed == false) {
                IsMagnetSensed = !limit.getState();

                telemetry.addData("Mode", "Lowering robot");
                telemetry.addData("Sensed:",!limit.getState());
                telemetry.update();


            }

            lift.setPower(0);
            //strafe off the hook then turn around
            SetDriveDistance(-1200, 1200, 1200, -1200, 0.4, 0.4, 0.4, 0.4);

            SetDriveDistance(-1460, -1460, -1460, -1460, 0.8, 0.8, 0.8, 0.8);

            SetDriveDistance(1400, -1400, 1400, -1400, 0.8, 0.8, 0.8, 0.8);

        }
    }