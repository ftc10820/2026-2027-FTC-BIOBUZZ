package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Collections;
import java.util.List;

/**
 * Shared hardware initialization and Limelight vision helpers.
 * Call initialize() before using the helpers, then poll the desired helper in the OpMode loop.
 * Only one pipeline runs at a time; switching tasks requires a new camera frame.
 */
public abstract class InitOpMode extends LinearOpMode {

    protected static final double STOPPER_CLOSED_POSITION = 0.0;
    protected static final double STOPPER_OPEN_POSITION = 0.2;
    // Shooter shaft RPM; the 2:1 reduction means 2250 here commands 4500 motor RPM.
    protected static final double SHOOTER_START_RPM = 2250.0;
    protected static final double SHOOTER_RPM_STEP = 250.0;
    protected static final double SHOOTER_MIN_RPM = 250.0;

    // Stock goBILDA 3208-0004-0001 turret: 2.75 servo turns per turret turn.
    // https://www.gobilda.com/gear-driven-turret-2-75-1-ratio-105mm-id/
    protected static final double TURRET_STEP_DEGREES = 10.0;
    // Eighteen 10-degree steps from center in either direction.
    protected static final double TURRET_MAX_ANGLE_DEGREES = 180.0;
    protected static final long TURRET_REPEAT_INTERVAL_NS = 100_000_000L;
    private static final double TURRET_GEAR_RATIO = 2.75;
    // 2000-0025-0502 in its default positional mode: 1800 degrees at 500–2500 us.
    // https://www.gobilda.com/2000-series-5-turn-dual-mode-servo-25-2-torque/
    private static final double TURRET_SERVO_TRAVEL_DEGREES = 1800.0;
    private static final double TURRET_CENTER_POSITION = 0.5;
    protected double turretTargetDegrees = 0.0;
    private ServoImplEx turretServo;

    protected DcMotor intakeMotor;
    protected DcMotor transferMotor;
    protected DcMotor slideMotor;
    protected Servo grabberServo;
    protected Servo flowerRampServo;
    protected Servo stopperServo;
    public MecanumDrive drive;

    // Optional until installed and configured. Check for null before using it.
    protected DcMotorEx shooterMotor;
    public void handleExpection(Exception e){
        StringWriter SW = new StringWriter();
        PrintWriter PW = new PrintWriter(SW);
        e.printStackTrace(PW);
        telemetry.addLine(PW.toString());
        telemetry.update();

    }
    public void initialize() {

        stopperServo = hardwareMap.get(Servo.class, "stopperServo");
        stopperServo.setPosition(STOPPER_CLOSED_POSITION);

        // Match the drive motor names used by Road Runner MecanumDrive.
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        transferMotor.setPower(0.0);
        transferMotor.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Configure Control Hub servo port 5 as "flowerRampServo".
        flowerRampServo = hardwareMap.get(Servo.class, "flowerRampServo");
        flowerRampServo.setPosition(0.5);

        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        grabberServo.setPosition(0.1);

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setPower(0.0);
        // Positive power raises the slide; reverse this direction if needed for the mechanism.
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor = hardwareMap.tryGet(DcMotorEx.class, "shooterMotor");
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
            // Preserve encoder velocity control for PID tuning and TeleOp speed adjustment.
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setVelocity(0.0);
        }

        // Based on the forward test; verify each name maps to the correct physical wheel.

        System.out.println("Drive motors initialized");
        telemetry.addLine("Drive motors initialized");
        // Temporarily disabled until the Limelight is configured; uncomment to enable vision.
        // initializeLimelight();
        telemetry.update();

    }

    /**
     * Called by TeleOp before waitForStart(). Configure the connected Hub servo port
     * as "turretServo". Mechanically align the turret forward at position 0.5.
     */
    protected void initializeTurret() {
        turretServo = hardwareMap.get(ServoImplEx.class, "turretServo");
        turretServo.setDirection(Servo.Direction.FORWARD);
        turretServo.setPwmRange(new PwmControl.PwmRange(500.0, 2500.0));
        turretTargetDegrees = 0.0;
        turretServo.setPosition(TURRET_CENTER_POSITION);
        telemetry.addData("Turret target (deg, +left)", turretTargetDegrees);
        telemetry.update();
    }

    /**
     * Positive angles turn the turret left (counterclockwise viewed from above).
     * Increasing PWM turns the upright servo clockwise; the external gears reverse
     * that into counterclockwise turret motion. Never wrap around at a travel limit.
     */
    protected void moveTurretByDegrees(double deltaDegrees) {
        if (deltaDegrees == 0.0) {
            return;
        }
        double nextTargetDegrees = turretTargetDegrees + deltaDegrees;
        if (Math.abs(nextTargetDegrees) > TURRET_MAX_ANGLE_DEGREES) {
            telemetry.log().add("Turret limit: 180 degrees from center");
            return;
        }
        double nextPosition = TURRET_CENTER_POSITION
                + nextTargetDegrees * TURRET_GEAR_RATIO / TURRET_SERVO_TRAVEL_DEGREES;
        if (nextPosition < 0.0 || nextPosition > 1.0) {
            telemetry.log().add("Turret travel limit: cannot take another 10-degree step");
            return;
        }
        turretServo.setPosition(nextPosition);
        turretTargetDegrees = nextTargetDegrees;
    }

    //region Limelight 3A

    /*
     * Based on the official Limelight 3A quick-start and FTC Java examples:
     * https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a
     * https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
     *
     * The SDK supplies the detections and robot pose. These helpers select the requested
     * pipeline, validate its latest frame, and pass the pose to our Road Runner localizer.
     */

    // Pipeline indices configured on the Limelight 3A.
    protected static final int LIMELIGHT_PIPELINE_APRILTAGS = 0;
    protected static final int LIMELIGHT_PIPELINE_BLUE_NECTAR = 1;
    protected static final int LIMELIGHT_PIPELINE_RED_NECTAR = 2;
    // TODO: Configure pipeline 3 as a yellow-pollen color pipeline on the Limelight.
    protected static final int LIMELIGHT_PIPELINE_YELLOW_POLLEN = 3;

    protected Limelight3A limelight;
    private static final long LIMELIGHT_MAX_RESULT_AGE_MS = 100;
    private int requestedLimelightPipeline = -1;
    private double lastAppliedPoseTimestamp = Double.NaN;

    /**
     * Uses the documented Limelight 3A hardware-map, polling, pipeline, and start calls.
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a#7-ftc-programming">Limelight 3A FTC example</a>
     */
    private void initializeLimelight() {
        // The Limelight 3A must be named "limelight" in the robot configuration.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        requestedLimelightPipeline = limelight.pipelineSwitch(LIMELIGHT_PIPELINE_APRILTAGS)
                ? LIMELIGHT_PIPELINE_APRILTAGS : -1;
        lastAppliedPoseTimestamp = Double.NaN;
        // Start polling so getLatestResult() can provide vision data.
        limelight.start();

        telemetry.addLine(requestedLimelightPipeline == LIMELIGHT_PIPELINE_APRILTAGS
                ? "Limelight initialized: AprilTags (pipeline 0)"
                : "Limelight started; AprilTag pipeline switch failed");
    }

    /**
     * Selects pipeline 0 and returns detected AprilTags, including IDs and target angles.
     * Returns an empty list if no fresh, valid AprilTag frame is available yet.
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming#72-fiducialapriltag-results">SDK AprilTag result example</a>
     */
    public List<FiducialResult> findAprilTags() {
        LLResult result = getLatestResultForPipeline(LIMELIGHT_PIPELINE_APRILTAGS);
        return result == null ? Collections.emptyList() : result.getFiducialResults();
    }

    /**
     * Selects pipeline 1 and returns blue-nectar color targets with angles and target area.
     * Returns an empty list if no fresh, valid blue-nectar frame is available yet.
     */
    public List<ColorResult> findBlueNectar() {
        return findColorTargets(LIMELIGHT_PIPELINE_BLUE_NECTAR);
    }

    /**
     * Selects pipeline 2 and returns red-nectar color targets with angles and target area.
     * Returns an empty list if no fresh, valid red-nectar frame is available yet.
     */
    public List<ColorResult> findRedNectar() {
        return findColorTargets(LIMELIGHT_PIPELINE_RED_NECTAR);
    }

    /**
     * Selects pipeline 3 and returns yellow-pollen color targets with angles and target area.
     * Configure this color pipeline on the Limelight before using it.
     * Returns an empty list if no fresh, valid yellow-pollen frame is available yet.
     */
    public List<ColorResult> findYellowPollen() {
        return findColorTargets(LIMELIGHT_PIPELINE_YELLOW_POLLEN);
    }

    /**
     * Selects the AprilTag pipeline and applies a new MegaTag 1 pose to the drive localizer.
     * Requires the correct field tag map, Full 3D enabled, and the camera's mounting pose
     * configured on the Limelight. Road Runner paths must use the same FTC field-centered
     * coordinate frame as Limelight's botpose (origin at field center).
     *
     * @return true if a new vision pose was applied; false leaves the pose unchanged
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming#5-wheres-my-robot-megatag-1">SDK MegaTag 1 pose example</a>
     */
    public boolean updateRobotPose() {
        if (drive == null) {
            return false;
        }

        LLResult result = getLatestResultForPipeline(LIMELIGHT_PIPELINE_APRILTAGS);
        if (result == null || result.getBotposeTagCount() < 1
                || result.getTimestamp() == lastAppliedPoseTimestamp) {
            return false;
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            return false;
        }

        // Limelight reports meters; Road Runner uses inches and radians.
        Position position = botpose.getPosition().toUnit(DistanceUnit.INCH);
        double heading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        if (!Double.isFinite(position.x) || !Double.isFinite(position.y)
                || !Double.isFinite(heading)) {
            return false;
        }

        // Consume pending odometry before correcting its field pose.
        drive.updatePoseEstimate();
        drive.localizer.setPose(new Pose2d(position.x, position.y, heading));
        lastAppliedPoseTimestamp = result.getTimestamp();
        return true;
    }

    /**
     * All three color finders read the SDK's color-results list, as in the FTC guide.
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming#71-color-results">SDK color result example</a>
     */
    private List<ColorResult> findColorTargets(int pipelineIndex) {
        LLResult result = getLatestResultForPipeline(pipelineIndex);
        return result == null ? Collections.emptyList() : result.getColorResults();
    }

    /**
     * Requests a pipeline only when it changes, without waiting for the next camera frame.
     * Rejects missing, invalid, stale, or wrong-pipeline results during a switch.
     * Uses the FTC guide's pipeline-index, result-validity, and staleness checks.
     */
    private LLResult getLatestResultForPipeline(int pipelineIndex) {
        if (limelight == null) {
            return null;
        }

        if (requestedLimelightPipeline != pipelineIndex) {
            requestedLimelightPipeline = -1;
            if (limelight.pipelineSwitch(pipelineIndex)) {
                requestedLimelightPipeline = pipelineIndex;
            }
            return null;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getPipelineIndex() != pipelineIndex
                || result.getStaleness() > LIMELIGHT_MAX_RESULT_AGE_MS) {
            return null;
        }
        return result;
    }

    //endregion
}
