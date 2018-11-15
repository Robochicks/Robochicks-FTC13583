package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="RuckusMeasure")
public class RuckusMeasure extends LinearOpMode{

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor arm;
    private Servo hook;
    private ServoController piratehook;

    public void runOpMode(){
        fl  = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        hook = hardwareMap.get(Servo.class, "hook");
        piratehook = hook.getController();

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData( "Distance", "BL (%d), BR (%d), FL (%d), FR (%d)", bl.getCurrentPosition(), br.getCurrentPosition(), fl.getCurrentPosition(), fr.getCurrentPosition());
            telemetry.addData( "Arm",arm.getCurrentPosition());
            telemetry.addData( "hook",piratehook.getServoPosition(0));
            telemetry.update();

        }


    }
}
