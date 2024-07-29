package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PID", group = "Robot")
public class PID extends LinearOpMode {
    // initializing all components
    private DcMotor arm = null;
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    CRServo bucketServo;
    double integralSum = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        bucketServo = hardwareMap.crservo.get("BucketServo");
        arm = hardwareMap.dcMotor.get("ArmMotor");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        double power = pidControl(728 / 4, arm.getCurrentPosition());
        arm.setPower(power);

    }

    public double pidControl(double targetPosistion, double currentPosistion) {
        double error = targetPosistion - currentPosistion;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        if (timer.seconds() <= 1) {
            timer.reset();
        }

        double output = (error * kp) + (derivative * kd) + (integralSum * ki);
        return output;
    }
}