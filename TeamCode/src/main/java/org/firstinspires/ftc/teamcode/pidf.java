package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Config
@TeleOp(name="teleop_robot")
public class pidf extends OpMode {
    private PIDController controller;

    public static double kp = 0, ki = 0, kd = 0;
    public static double kf = 0;
    public static int target = 0;
    private final double encoderTicksInDegrees = 365.5;
    private DcMotor arm = null;

    @Override
    public void init() {
        controller = new PIDController(kp, ki, kd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.dcMotor.get("arm");
    }

    @Override
    public void loop() {
        if (gamepad2.x) {
            pidf();
        }
    }
    private void pidf(){
        controller.setPID(kp, ki, kd);
        // gets the current position of the arm
        int currentPosition = arm.getCurrentPosition();
        // calculates the power needed to move the arm with the feedforward constant
        double pid = controller.calculate(currentPosition, target);
        double ff = Math.cos(Math.toRadians(target / encoderTicksInDegrees)) * kf;
        double power = pid + ff;

        arm.setPower(power);
        telemetry.addData("CurrentPosition", currentPosition);
        telemetry.addData("target", target);
        telemetry.addData("PID",pid);
        telemetry.addData("FeedForward",ff);
        telemetry.update();

    }
}
