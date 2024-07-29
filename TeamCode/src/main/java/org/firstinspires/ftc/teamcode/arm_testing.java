package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "arm_testing", group = "Robot")
public class arm_testing extends LinearOpMode {

    // Declare OpMode members
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor arm;
    private CRServo bucketServo;
    private ElapsedTime runtime = new ElapsedTime();

    // Named constants
    private static final double MOVE_FORWARD_DURATION = 2.0;
    private static final double RAISE_ARM_DURATION = 0.5;
    private static final double RAISE_BUCKET_DURATION = 0.5;
    private static final double MOTOR_POWER = 0.5;
    private static final double BUCKET_SERVO_POWER = -0.5;

    @Override
    public void runOpMode() {
        // Initialize the drive system variables
        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        backLeft = hardwareMap.dcMotor.get("BL");
        backRight = hardwareMap.dcMotor.get("BR");
        bucketServo = hardwareMap.crservo.get("BucketServo");
        arm = hardwareMap.dcMotor.get("ArmMotor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // Raise the bucket for a duration
        bucketServo.setPower(BUCKET_SERVO_POWER);
        while (opModeIsActive() && (runtime.seconds() < RAISE_BUCKET_DURATION)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Move forward for a duration
        runtime.reset();
        frontLeft.setPower(MOTOR_POWER);
        backLeft.setPower(-MOTOR_POWER);
        frontRight.setPower(MOTOR_POWER);
        backRight.setPower(-MOTOR_POWER);
        while (opModeIsActive() && (runtime.seconds() < MOVE_FORWARD_DURATION)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Raise the arm
        runtime.reset();
        arm.setPower(MOTOR_POWER);
        while (opModeIsActive() && (runtime.seconds() < RAISE_ARM_DURATION)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Lower the bucket
        runtime.reset();
        bucketServo.setPower(-BUCKET_SERVO_POWER);
        while (opModeIsActive() && (runtime.seconds() < RAISE_BUCKET_DURATION)) {
            telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop all motors and complete
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
