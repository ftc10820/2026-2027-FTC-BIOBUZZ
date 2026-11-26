package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
// import org.firstinspires.ftc.teamcode.PIDConstants;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Shared hardware initialization for the robot OpModes.
 */
public abstract class InitOpMode extends LinearOpMode {

    protected static final int PIPE_ANY_HIVE = 0;
    protected static final int PIPE_RED_HIVE = 1;
    protected static final int PIPE_BLUE_HIVE = 2;
    private static final String LIMELIGHT_NAME = "limelight";

    protected static final double STOPPER_CLOSED_POSITION = 0.0;
    protected static final double STOPPER_OPEN_POSITION = 0.2;
    // Shooter shaft RPM; the 2:1 reduction means 2400 here commands 4800 motor RPM.
    protected static final double SHOOTER_START_RPM = 2400.0;
    protected static final double SHOOTER_RPM_STEP = 50.0;
    protected static final double SHOOTER_MIN_RPM = 250.0;
    protected static final long SHOOTER_RPM_SAMPLE_INTERVAL_NS = 1_000_000_000L;

    protected DcMotorEx intakeMotor;
    protected DcMotorEx transferMotor;
    protected DcMotorEx turretMotor;
    protected Servo flowerRampServo;
    protected Servo stopperServo;
    public MecanumDrive drive;

    // Optional until installed and configured. Check for null before using it.
    protected DcMotorEx shooterMotor;
    protected LimelightVision vision;
    private TurretTracking turretTracking;

    public void handleExpection(Exception e){
        StringWriter SW = new StringWriter();
        PrintWriter PW = new PrintWriter(SW);
        e.printStackTrace(PW);
        telemetry.addLine(PW.toString());
        telemetry.update();

    }
    public void initialize() {
        initialize(PIPE_ANY_HIVE);
    }

    protected void initialize(int pipeline) {
        stopperServo = hardwareMap.get(Servo.class, "stopperServo");
        stopperServo.setPosition(STOPPER_CLOSED_POSITION);

        // Match the drive motor names used by Road Runner MecanumDrive.
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intakeMotor = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "intakeMotor");

        transferMotor = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setPower(0.0);
        transferMotor.setDirection(DcMotorEx.Direction.FORWARD);
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setPower(0.0);

        // Configure Control Hub servo port 5 as "flowerRampServo".
        flowerRampServo = hardwareMap.get(Servo.class, "flowerRampServo");
        flowerRampServo.setPosition(0.5);

        shooterMotor = (DcMotorEx) hardwareMap.tryGet(DcMotorEx.class, "shooterMotor");
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
            // Preserve encoder velocity control for PID tuning and TeleOp speed adjustment.
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            // Incoming PIDF override disabled to preserve the working shooter settings.
            // shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(PIDConstants.KP,PIDConstants.KI,PIDConstants.KD,PIDConstants.KF));
            shooterMotor.setVelocity(0.0);
        }

        // Based on the forward test; verify each name maps to the correct physical wheel.

        System.out.println("Drive motors initialized");
        telemetry.addLine("Drive motors initialized");
        vision = new LimelightVision(hardwareMap.tryGet(Limelight3A.class, LIMELIGHT_NAME), pipeline);
        vision.addTelemetry(telemetry);
        telemetry.update();

    }

    protected void initializeTurretTeleOp(int pipeline) {
        initialize(pipeline);
        turretTracking = new TurretTracking(turretMotor);
        vision.startStatusPolling();
    }

    protected void updateTurretTracking() {
        vision.update();
        turretTracking.update(vision.getTargetTx(), vision.getFrameTimestamp());
    }

    protected void addTurretTelemetry() {
        vision.addTelemetry(telemetry);
        turretTracking.addTelemetry(telemetry);
    }

    protected void stopTurretTeleOp() {
        try {
            turretTracking.stop();
        } finally {
            vision.stop();
        }
    }
}
