package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name="RuckusMeasure")
@Disabled

public class RuckusMeasure extends LinearOpMode{

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor arm;
    private Servo hook;
    private ServoController piratehook;
   // private BNO055IMU imu;
    private Orientation lastAndle = new Orientation();
    private double globalAngle, power = .30, correction;
    private DigitalChannel limit;


    public void runOpMode(){
        fl  = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        limit = hardwareMap.get(DigitalChannel.class, "Limit");
        //arm = hardwareMap.get(DcMotor.class, "Arm");
        //hook = hardwareMap.get(Servo.class, "hook");

       /*imu = hardwareMap.get(BNO055IMU.class,"imu");
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
        telemetry.update();*/

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
       // arm.setDirection(DcMotor.Direction.FORWARD);
        limit.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData( "Distance", "BL (%d), BR (%d), FL (%d), FR (%d)", bl.getCurrentPosition(), br.getCurrentPosition(), fl.getCurrentPosition(), fr.getCurrentPosition());
            //telemetry.addData( "Arm",arm.getCurrentPosition());
            //telemetry.addData( "hook",piratehook.getServoPosition(0));
            telemetry.addData("Sensed", !limit.getState());
            //telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            telemetry.update();



        }


    }
}
