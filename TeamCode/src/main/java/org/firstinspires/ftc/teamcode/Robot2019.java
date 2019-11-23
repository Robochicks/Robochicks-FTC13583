package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

public class Robot2019 implements Robot {
    //add variables

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    //private CRServoImplEx gs;
    private ServoImplEx gs;
   // private DcMotor lm;
    private DcMotor rm;
    private DcMotor em;


    public Robot2019(HardwareMap hardwareMap) {
        //assign variables

        fl = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        gs = hardwareMap.get(ServoImplEx.class, "GS");
        //lm = hardwareMap.get(DcMotor.class,"LM");
        rm = hardwareMap.get(DcMotor.class, "RM");
        em = hardwareMap.get (DcMotor.class, "EM");

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        //lm.setDirection(DcMotor.Direction.FORWARD);
        rm.setDirection(DcMotor.Direction.FORWARD);
        em.setDirection(DcMotorSimple.Direction.FORWARD);
        //sr.setDirection(CRServoImplEx.);



        //lm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    //drive function

    /**
     * DriveFunction
     * Driving during the teleop
     * @param gamepad1 To control the robot in drive mode (Driver)
     * @param gamepad2 To control the robot in drive mode (Operator)
     */
    public String DriveFunction(Gamepad gamepad1, Gamepad gamepad2) {

        String telemetry = "";
        double flPower;
        double frPower;
        double blPower;
        double brPower;
        //double lmPower;
        double rmPower;
        double emPower;
        //double gsPower;
        double gsPosition;
        int lmPosition;

        double y1 = -gamepad1.left_stick_y;
        double x2 = gamepad1.right_stick_x;
        double OPx2 = gamepad2.right_stick_x;
        boolean OperatorBumper = gamepad2.right_bumper;
        boolean OperatorBumperLeft = gamepad2.left_bumper;
        double Operator1y = gamepad2.left_stick_y;
        double Opertor2x = gamepad2.right_stick_x;


        flPower = Range.clip(y1 + x2, -1.0, 1.0);
        frPower = Range.clip(y1 - x2, -1.0, 1.0);
        blPower = Range.clip(y1 + x2, -1.0, 1.0);
        brPower = Range.clip(y1 - x2, -1.0, 1.0);
        //lmPower = OPx2;

        //288


        rmPower = Operator1y;
        emPower = Opertor2x;






        //MEASURE OPEN AND CLOSE STATE AND MAKE PRESETS FOR THE ARM.
        if (OperatorBumper == true){
            gsPosition = gs.getPosition() + .02;

        }
        else if (OperatorBumperLeft == true) {
            gsPosition = gs.getPosition() - .02;
        }
        else{
            gsPosition = gs.getPosition();
        }

        gs.setPosition(gsPosition);
        telemetry += gsPosition;

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

        SetDrivePower(flPower,frPower,blPower,brPower,rmPower, emPower);
        /*if (gamepad1.a == true) {
            gs.setPower(1.0);
        }
        else{
            gs.setPower(0.0);
        }*/


        return telemetry;
    }

    /**
     * SetDrivePower
     * Associates powers with corresponding motors
     * @param FrontLeftPower Sets front left power
     * @param FrontRightPower Sets front right power
     * @param BackLeftPower Sets back left power
     * @param BackRightPower Sets back right power
     */
    private void SetDrivePower(double FrontLeftPower, double FrontRightPower, double BackLeftPower, double BackRightPower, Double RaiseMotorPower, double ExtendMotorPower) {
        fl.setPower(FrontLeftPower);
        fr.setPower(FrontRightPower);
        bl.setPower(BackLeftPower);
        br.setPower(BackRightPower);
        //lm.setPower(LiftMotorPower);
        rm.setPower(RaiseMotorPower);
        em.setPower(ExtendMotorPower);
    }
}

