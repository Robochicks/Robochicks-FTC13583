package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

public class Robot2019 implements Robot {
    //add variables

    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;
    //private CRServoImplEx gs;
    private Servo gs;
    private DcMotor lm;
    String grip_status = "closed";

    private boolean gs_open;
    private boolean gs_moving = false;
    private double gsTarget = 0;
    public ColorSensor color_sensor;

    //JA changes start
    private CRServoImpl exServo;
    //private DcMotor rm;
   // private DcMotor em;
    //JA changes end


    public Robot2019(HardwareMap hardwareMap) {
        //assign variables

        //Getting the extended DCMotor class.
        //This is needed to correct the PID of the motors for using the encoder.
        fl = (DcMotorEx) hardwareMap.get(DcMotor.class, "FL");
        fr = (DcMotorEx) hardwareMap.get(DcMotor.class, "FR");
        bl = (DcMotorEx) hardwareMap.get(DcMotor.class, "BL");
        br = (DcMotorEx) hardwareMap.get(DcMotor.class, "BR");
        gs = hardwareMap.get(Servo.class, "GS");
        lm = hardwareMap.get(DcMotor.class, "LM");

        color_sensor = hardwareMap.colorSensor.get("color");

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        //JA changes start
        //rm = hardwareMap.get(DcMotor.class, "RM");
        lm.setDirection(DcMotor.Direction.REVERSE);
        //rm.setDirection(DcMotor.Direction.FORWARD);
        //JA changes end

        exServo = hardwareMap.get(CRServoImpl.class, "exServo");

        /*em = hardwareMap.get (DcMotor.class, "EM");
        em.setDirection(DcMotorSimple.Direction.FORWARD);*/
        //sr.setDirection(CRServoImplEx.);


        lm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * Drive function. Use with gamepads for teleop.
     * Goal will be to interpret the inputs, and operate the robot.
     * @opmode
     * @param gamepad1
     * @param gamepad2
     * @param telemetry
     */
    public void DriveFunction(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {

        double flPower;
        double frPower;
        double blPower;
        double brPower;
        double lmPower;

        //double gsPower;
        double gsPosition;

        //JA changes start
        //double rmPower;
        int lmPosition;
        //JA changes end

        double emPower;

        double drive = -gamepad1.left_stick_y;
        double shift = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        //double OPx2 = gamepad2.right_stick_x;
        boolean OperatorBumper = gamepad2.right_bumper;
        boolean OperatorBumperLeft = gamepad2.left_bumper;
        double lift = gamepad2.left_stick_y;
        double extend = gamepad2.right_stick_x;

        color_sensor.red();
        color_sensor.green();
        color_sensor.blue();

        color_sensor.alpha();
        color_sensor.argb();

        //TODO: add gamepad1.left_stick_x to powers
        flPower = Range.clip(drive + rotate + shift, -1.0, 1.0);
        frPower = Range.clip(drive - rotate - shift, -1.0, 1.0);
        blPower = Range.clip(drive + rotate - shift, -1.0, 1.0);
        brPower = Range.clip(drive - rotate + shift, -1.0, 1.0);

        lmPower = lift * .75;

        emPower = extend;

        if (OperatorBumper == true ){
            gsTarget = 0.0;
            gs.setPosition(gsTarget);
            grip_status = "closed";
        }
        else if (OperatorBumperLeft == true) {
            gsTarget = 1.0;
            gs.setPosition(gsTarget);
            grip_status = "open";
        }

        if (gamepad1.left_bumper == true) {
            // telemetry.addData("Left Bumper", "Pressed");
            frPower = (0.7);
            blPower = (0.7);
            flPower = (-0.7);
            brPower = (-0.7);
        }

        if (gamepad1.right_bumper == true) {
            //  telemetry.addData("Right Bumper", "Pressed");
            brPower = (0.7);
            flPower = (0.7);
            blPower = (-0.7);
            frPower = (-0.7);
        }

        SetDrivePower(flPower,frPower,blPower,brPower, emPower, lmPower);

        telemetry.addData("Grip",grip_status);
        telemetry.addData("FrontLeftMotor",flPower);
        telemetry.addData("FrontRightMotor",frPower);
        telemetry.addData("BackLeftMotor",blPower);
        telemetry.addData("BackRightMotor",brPower);
        telemetry.addData("ExtensionMotor",emPower);
        telemetry.addData("LiftMotor",lmPower);
    }

    public void DriveUntilColor (double power){

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    /**
     * SetDrivePower
     * Associates powers with corresponding motors
     * To be used with encoder
     * @opmode
     * @param FrontLeftPower Sets front left power
     * @param FrontRightPower Sets front right power
     * @param BackLeftPower Sets back left power
     * @param BackRightPower Sets back right power
     * @param LiftMotorPower Sets lift motor power
     */
    public void SetDrivePower(double FrontLeftPower, double FrontRightPower, double BackLeftPower, double BackRightPower, double ExtendMotorPower, double LiftMotorPower) {
        fl.setPower(FrontLeftPower);
        fr.setPower(FrontRightPower);
        bl.setPower(BackLeftPower);
        br.setPower(BackRightPower);


        exServo.setPower(ExtendMotorPower);
        lm.setPower(LiftMotorPower);
    }

    /**
     * Function to clear the motor encoders.
     * Do nothing if not using encoders.
     * @autonomous
     */
    public void ClearEncoders(){
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Sets the robot up for autonomous mode.
     * this sets the
     * also sets the PID of motors
     * IMPORTANT: this should NOT be touched, it currently works and stops the
     *              robot from jittering when driving with encoders.
     * @autonomous
     */
    public void PrepRobotAuto(){
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double P = 10, I = 3, D = 0.00, F = 1.17;
        fl.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(new PIDCoefficients(P,I,D)));
        bl.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(new PIDCoefficients(P,I,D)));
        br.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(new PIDCoefficients(P,I,D)));
        fr.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(new PIDCoefficients(P,I,D)));
    }

    /**
     * This handles the motor logging.
     * @param telemetry
     */
    public void logMotors(Telemetry telemetry){
        telemetry.addData("Mode" , "Moving");
        telemetry.addData( "Distance BL", bl.getCurrentPosition() + "/" + bl.getTargetPosition());
        telemetry.addData( "Distance BR", br.getCurrentPosition() + "/" + br.getTargetPosition());
        telemetry.addData( "Distance FL", fl.getCurrentPosition() + "/" + fl.getTargetPosition());
        telemetry.addData( "Distance FR", fr.getCurrentPosition() + "/" + fr.getTargetPosition());

        telemetry.addData( "Busy BL", bl.isBusy());
        telemetry.addData( "Busy BR", br.isBusy());
        telemetry.addData( "Busy FL", fl.isBusy());
        telemetry.addData( "Busy FR", fr.isBusy());
    }

    /**
     * Function to drive the robot a set amount of distance.
     * Uses encoder (likely) to determine movement.
     * Use each motor equipped with an encoder.
     * @autonomous
     * @param direction
     * @param distance
     */
    public void SetEncoderDistance(direction direction, int distance){
        bl.setTargetPosition(direction.bl_dir * distance);
        br.setTargetPosition(direction.br_dir * distance);
        fl.setTargetPosition(direction.fl_dir * distance);
        fr.setTargetPosition(direction.fr_dir * distance);

        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void SetEncoderDistance(){

    }

    /**
     * This tells the user if the drive motors are still busy.
     * Expand to manipulators if needed.
     * @autonomous
     * @return true if the motors are busy.
     */
    public boolean isBusy(){
        return fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy();
    }


    /**
     * Function to drive the robot a set amount of time.
     * @autonomous
     * @param telemetry
     * @param time
     * @param FrontLeftPower
     * @param FrontRightPower
     * @param BackLeftPower
     * @param BackRightPower
     * @param LiftPower
     */
    public void SetDriveTime (Telemetry telemetry, double time, double FrontLeftPower, double FrontRightPower, double BackLeftPower, double BackRightPower, double LiftPower){

        bl.setPower(BackLeftPower);
        br.setPower(BackRightPower);
        fl.setPower(FrontLeftPower);
        fr.setPower(FrontRightPower);
        lm.setPower(LiftPower);
        telemetry.addData("Time movement", "Starting");
        telemetry.update();

        try {
            telemetry.addData("Time movement", "Waiting");
            telemetry.update();
            sleep(Math.round(time*1000));
        } catch (Exception e){
            //do nothing
        }
        telemetry.addData("Time movement", "Ending");
        telemetry.update();

        bl.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        lm.setPower(0);
    }

    public void SetDriveTime(Telemetry telemetry, direction direction, double power, int time){
        bl.setPower(power * direction.bl_dir);
        br.setPower(power * direction.br_dir);
        fl.setPower(power * direction.fl_dir);
        fr.setPower(power * direction.fr_dir);
        telemetry.addData("Time movement", "Starting");
        telemetry.update();

        try {
            telemetry.addData("Time movement", "Waiting");
            telemetry.update();
            sleep(time*1000);
        } catch (Exception e){
            //do nothing
        }
        telemetry.addData("Time movement", "Ending");
        telemetry.update();

        bl.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        lm.setPower(0);
    }

    /**
     * @deprecated
     * drive with a set position.
     * No longer needed. Abandoned to allow autonomous to stop robot.
     * @param telemetry
     * @double power,
     * @param FrontLeftDistance
     * @param FrontRightDistance
     * @param BackLeftDistance
     * @param BackRightDistance
     */
    public void SetDriveDistance(Telemetry telemetry, double drive_power, int FrontLeftDistance, int FrontRightDistance, int BackLeftDistance, int BackRightDistance){
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

        bl.setPower(drive_power);
        br.setPower(drive_power);
        fl.setPower(drive_power);
        fr.setPower(drive_power);

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

