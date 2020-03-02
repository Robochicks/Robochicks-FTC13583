package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Robot {
    /**
     * Drive function. Use with gamepads for teleop.
     * Goal will be to interpret the inputs, and operate the robot.
     * @opmode
     * @param gamepad1
     * @param gamepad2
     * @param telemetry
     */
    public void DriveFunction(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry);

    /**
     * Function to drive the robot a set amount of distance.
     * Uses encoder (likely) to determine movement.
     * Use each motor equipped with an encoder.
     * @autonomous
     * @param direction
     * @param distance
     */
    public void SetEncoderDistance(direction direction, int distance);

    /**
     * Function to drive the robot a set amount of time.
     * @autonomous
     * @param telemetry
     * @param time
     * @param direction
     * @param power
     * TODO: Fix declaration
     */
   // public void SetDriveTime(Telemetry telemetry, int time, direction direction, double...power);

    /**
     * SetDrivePower
     * Associates powers with corresponding motors
     * @autonomous
     * @param direction
     * @param power
     * TODO: Fix declaration
     */
    //public void SetDrivePower(direction direction, double power);

    /**
     * Function to clear the motor encoders.
     * Do nothing if not using encoders.
     * @autonomous
     */
    public void ClearEncoders();

    /**
     * Sets the robot up for autonomous mode.
     * @autonomous
     */
    public void PrepRobotAuto();

    /**
     * This tells the user if the drive motors are still busy.
     * Expand to manipulators if needed.
     * @autonomous
     * @return true if the motors are busy.
     */
    public boolean isBusy();

    /**
     * This handles the motor logging.
     * @param telemetry
     */
    public void logMotors(Telemetry telemetry);

}
