package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Robot {
    public void DriveFunction(Gamepad gamepad1, Gamepad gamepad2, Telemetry telementry);
    public void SetDriveDistance(int FrontLeftDistance, int FrontRightDistance, int BackLeftDistance, int BackRightDistance, double FrontLeftPower, double FrontRightPower, double BackLeftPower, double BackRightPower);
}
