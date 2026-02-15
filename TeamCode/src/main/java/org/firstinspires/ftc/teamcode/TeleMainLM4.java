package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "TeleMainLM4", group = "StarterBot")
public class TeleMainLM4 extends OpMode {
    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */

    /* =======================================================================================
     * Constants
     * ======================================================================================= */
    private static final double SHOOTER_VELOCITY_PRESET = 1200;
    private static final double LEFT_PUSHER_VELOCITY = -2400;
    private static final double RIGHT_PUSHER_VELOCITY = 2400;
    private static final double INTAKE_POWER = -1.0;

    /* =======================================================================================
     * Hardware
     * ======================================================================================= */
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor intake;

    private DcMotorEx shooter;
    private DcMotorEx leftPusher;
    private DcMotorEx rightPusher;

    /* =======================================================================================
     * Drive telemetry
     * ======================================================================================= */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        mapHardware();
        configureMotors();

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        handleShooter();
        handleIntake();
        handlePushers();
        handleDrive();
        updateTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /* =======================================================================================
     * Hardware setup
     * ======================================================================================= */

    private void mapHardware() {
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftPusher = hardwareMap.get(DcMotorEx.class, "leftPusher");
        rightPusher = hardwareMap.get(DcMotorEx.class, "rightPusher");
    }

    private void configureMotors() {
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(280, 0, 0, 14.8177));
        shooter.setDirection(REVERSE);

        leftPusher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(22, 0, 0, 15.049));
        leftPusher.setDirection(FORWARD);

        rightPusher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(14, 0, 0, 11.6928));
        rightPusher.setDirection(FORWARD);

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(REVERSE);
        leftBackDrive.setDirection(REVERSE);
        rightFrontDrive.setDirection(FORWARD);
        rightBackDrive.setDirection(FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
    }

    /* =======================================================================================
     * Subsystem handlers
     * ======================================================================================= */

    private void handleShooter() {
        if (gamepad2.a) {
            shooter.setVelocity(SHOOTER_VELOCITY_PRESET);
        } else if (gamepad2.b) {
            shooter.setPower(0);
        }
    }

    private void handleIntake() {
        if (gamepad1.right_bumper) {
            intake.setPower(INTAKE_POWER);
        }
        else {
            intake.setPower(0);
        }
    }

    private void handlePushers() {
        if (gamepad2.left_bumper) {
            leftPusher.setVelocity(LEFT_PUSHER_VELOCITY);
        } else {
            leftPusher.setVelocity(0);
        }

        if (gamepad2.right_bumper) {
            rightPusher.setVelocity(RIGHT_PUSHER_VELOCITY);
        } else {
            rightPusher.setVelocity(0);
        }
    }

    private void handleDrive() {
        mecanumDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
    }

    /* =======================================================================================
     * Drive math
     * ======================================================================================= */

    private void mecanumDrive(double forward, double strafe, double rotate) {

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /* =======================================================================================
     * Telemetry
     * ======================================================================================= */

    private void updateTelemetry() {
        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.addData("Left Pusher Velocity", leftPusher.getVelocity());
        telemetry.addData("Right Pusher Velocity", rightPusher.getVelocity());
        telemetry.update();
    }
}