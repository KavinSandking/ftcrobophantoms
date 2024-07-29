package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

@Autonomous(name = "Gyro", group = "Robot")
public class Gyro extends LinearOpMode {
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    private DcMotor arm;
    private CRServo bucketServo;


    private GyroSensor gyro;

    private int lastRawGyroHeading = 0;
    private int GyroHeading = 0;

    private int getGyroHeading() {
        int rawGyroHeading = gyro.getHeading();

        if (rawGyroHeading - 180 > lastRawGyroHeading) {
            GyroHeading = GyroHeading + rawGyroHeading - 360 - lastRawGyroHeading;
        } else if (rawGyroHeading + 180 < lastRawGyroHeading) {
            GyroHeading = GyroHeading + rawGyroHeading + 360 - lastRawGyroHeading;
        } else {
            GyroHeading = GyroHeading + rawGyroHeading - lastRawGyroHeading;
        }

        lastRawGyroHeading = rawGyroHeading;
        return GyroHeading;
    }

    private void resetGyroHeading() {
        lastRawGyroHeading = 0;
        GyroHeading = 0;
        gyro.resetZAxisIntegrator();
    }

    private void turn(double power, long durationMillis) {
        resetGyroHeading();
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < durationMillis && opModeIsActive()) {
            if (getGyroHeading() < 0) {
                FrontLeft.setPower(power);
                FrontRight.setPower(-power);
                BackLeft.setPower(power);
                BackRight.setPower(-power);
            } else {
                FrontLeft.setPower(-power);
                FrontRight.setPower(power);
                BackLeft.setPower(-power);
                BackRight.setPower(power);
            }
            idle();
        }
        stopMotors();
    }

    private void drive(double power, long durationMillis) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < durationMillis && opModeIsActive()) {
            FrontLeft.setPower(power);
            FrontRight.setPower(power);
            BackLeft.setPower(power);
            BackRight.setPower(power);
        if (durationMillis > 0){
            stopMotors();


        }

            }
            idle();
        stopMotors();
        }


    private void arm(double armPower, long durationMillis) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < durationMillis && opModeIsActive()) {
            arm.setPower(armPower);
            idle();
        }
        arm.setPower(0.0);
    }
    private void bucket(double bucketPower, long durationMillis) {
        long startTime = System.currentTimeMillis();
        long currentTime;
        while (opModeIsActive()) {
            currentTime = System.currentTimeMillis();
            if (currentTime - startTime <= durationMillis) {
                bucketServo.setPower(bucketPower);
            } else {
                break;
            }
            idle();
        }
        bucketServo.setPower(0.0);
    }

    private void stopMotors() {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        FrontLeft = hardwareMap.dcMotor.get("right_motor");
        FrontRight = hardwareMap.dcMotor.get("left_motor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        bucketServo = hardwareMap.crservo.get("racpServo");

        gyro.calibrate();

        sleep(1000);

        while (gyro.isCalibrating()) {
            idle();
            sleep(50);
        }

        waitForStart();

        resetGyroHeading();

        turn(0.5, 1000);
        sleep(500);
        drive(0.5, 1500);
        sleep(500);
        turn(0.5, 1000);
        sleep(500);
        drive(0.5, 2000);
        sleep(500);
        arm(0.5, 1500);
        bucket(0.5,1500);

        stopMotors();
    }
}
