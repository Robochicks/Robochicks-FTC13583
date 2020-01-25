package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

public class Robot2019 implements Robot {
    //add variables

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
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

        fl = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        gs = hardwareMap.get(Servo.class, "GS");
        lm = hardwareMap.get(DcMotor.class,"LM");

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

    //drive function

    /**
     * DriveFunction
     * Driving during the teleop
     * @param gamepad1 To control the robot in drive mode (Driver)
     * @param gamepad2 To control the robot in drive mode (Operator)
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
        double lift = -gamepad2.left_stick_y;
        double extend = gamepad2.right_stick_x;

        color_sensor.red();
        color_sensor.green();
        color_sensor.blue();

        color_sensor.alpha();
        color_sensor.argb();

        //TODO: add gamepad1.left_stick_x to powers
        flPower = Range.clip(drive + rotate - shift, -1.0, 1.0);
        frPower = Range.clip(drive - rotate + shift, -1.0, 1.0);
        blPower = Range.clip(drive + rotate + shift, -1.0, 1.0);
        brPower = Range.clip(drive - rotate - shift, -1.0, 1.0);

        //JA changes start
        lmPower = lift / 3;
        //lmPower = extend;
        ////rmPower = lift;
        //JA changes end

        //288


        //rmPower = lift;
        emPower = extend;

        //JA changes start
        /*if ((gs.getPosition() == 1 || gs.getPosition() == 0) && gs_moving == true){

            gs_moving = false;

        }*/

        //MEASURE OPEN AND CLOSE STATE AND MAKE PRESETS FOR THE ARM.
        //&& (gsPosition != 1.0 && gsPosition != 0.0)
        /*if (OperatorBumper == true && gs_moving == false){
            if (gs_open == true){
                    gsPosition = 0.0;
                    gs_open = false;
                    gs_moving = true;
                }else{
                gsPosition = 1.0;
                gs_open = true;
                gs_moving =true;
            }
            gs.setPosition(gsPosition);
            telemetry += "New target: " + gsTarget;

        }*/
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
        //JA changes end




        /*if (OPx2 > 0){
            lmPosition = lm.getCurrentPosition() + 2;
            lm.setTargetPosition(lmPosition);
            lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (OPx2 < 0) {
            lmPosition = lm.getCurrentPosition() - 2;
            lm.setTargetPosition(lmPosition);
            lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{
            lmPosition = lm.getCurrentPosition();
        }


        //lm.setPower(1);

         */


        //TODO: address powers, potentially remove shift function from bumpers or readdress priority for values
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

       /* if (gamepad2.a == true){

        }

        // gs.setPosition(gsPosition);
        //telemetry = telemetry + "sv:" + gsPosition
        //            + " m: " + lmPosition;
        */

       //JA change start
       // SetDrivePower(flPower,frPower,blPower,brPower,rmPower, emPower);
        SetDrivePower(flPower,frPower,blPower,brPower, emPower, lmPower);
        //JA change end

        /*if (gamepad1.a == true) {
            gs.setPower(1.0);
        }
        else{
            gs.setPower(0.0);
        }*/

        //telemetry = "";
        telemetry.addData("Grip",grip_status);
        telemetry.addData("FrontLeftMotor",flPower);
        telemetry.addData("FrontRightMotor",frPower);
        telemetry.addData("BackLeftMotor",blPower);
        telemetry.addData("BackRightMotor",brPower);
        telemetry.addData("ExtensionMotor",emPower);
        telemetry.addData("LiftMotor",lmPower);

        //return "";
    }

    public void DriveUntilColor (Telemetry telemetry){

        double power = 1;

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        while (color_sensor.blue() < 10 && color_sensor.red() < 10){
            try {
                sleep(100);
            } catch (Exception e){
                //do nothing
            }
            telemetry.addData("Blue",color_sensor.blue());
            telemetry.addData("Red",color_sensor.red());
            telemetry.addData("Green",color_sensor.green());
            telemetry.addData("status", "searching...");
            telemetry.update();
        }
        telemetry.addData("status","found!");
        telemetry.update();


        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    /**
     * SetDrivePower
     * Associates powers with corresponding motors
     * @param FrontLeftPower Sets front left power
     * @param FrontRightPower Sets front right power
     * @param BackLeftPower Sets back left power
     * @param BackRightPower Sets back right power
     */
    //JA changes start
    private void SetDrivePower(double FrontLeftPower, double FrontRightPower, double BackLeftPower, double BackRightPower, double ExtendMotorPower, double LiftMotorPower) {
      //  private void SetDrivePower(double FrontLeftPower, double FrontRightPower, double BackLeftPower, double BackRightPower, Double RaiseMotorPower, double ExtendMotorPower) {
        //JA chagnes end
        fl.setPower(FrontLeftPower);
        fr.setPower(FrontRightPower);
        bl.setPower(BackLeftPower);
        br.setPower(BackRightPower);
        //lm.setPower(LiftMotorPower);

        //JA changes start
        //em.setPower(ExtendMotorPower);
        //rm.setPower(RaiseMotorPower);

        exServo.setPower(ExtendMotorPower);
        lm.setPower(LiftMotorPower);
        //JA changes end
    }

    public void SetDriveDistance(int FrontLeftDistance, int FrontRightDistance, int BackLeftDistance, int BackRightDistance, double FrontLeftPower, double FrontRightPower, double BackLeftPower, double BackRightPower){
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
           /* telemetry.addData("Mode" , "Moving");
            telemetry.addData( "Distance BL", bl.getCurrentPosition());
            telemetry.addData( "Distance BR", br.getCurrentPosition());
            telemetry.addData( "Distance FL", fl.getCurrentPosition());
            telemetry.addData( "Distance FR", fr.getCurrentPosition());

            telemetry.addData( "Busy BL", bl.isBusy());
            telemetry.addData( "Busy BR", br.isBusy());
            telemetry.addData( "Busy FL", fl.isBusy());
            telemetry.addData( "Busy FR", fr.isBusy());

           telemetry.update();*/
        }

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       // telemetry.addData("EncoderMovement", "Complete");
        //telemetry.update();

        //return"";


    }
}

