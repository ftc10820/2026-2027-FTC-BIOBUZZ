package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

final class LimelightVision {
    private static final long MAX_FRAME_AGE_MS = 100;

    private final Limelight3A camera;
    private final int selectedPipeline;
    private final boolean switchAccepted;
    private ScheduledExecutorService statusPoller;
    private volatile int runningPipeline = -1;
    private double frameTimestamp = Double.NaN;
    private long frameFirstSeenNanos;
    private String tagIds = "";
    private FiducialResult target;

    LimelightVision(Limelight3A camera, int selectedPipeline) {
        this.camera = camera;
        this.selectedPipeline = selectedPipeline;
        if (camera == null) {
            switchAccepted = false;
        } else {
            camera.setPollRateHz(100);
            switchAccepted = camera.pipelineSwitch(selectedPipeline);
            camera.start();
        }
    }

    void startStatusPolling() {
        if (camera == null) {
            return;
        }
        // getStatus() performs HTTP I/O. Keep it off the driver control loop.
        statusPoller = Executors.newSingleThreadScheduledExecutor();
        statusPoller.scheduleWithFixedDelay(new Runnable() {
            @Override
            public void run() {
                LLStatus status = camera.getStatus();
                runningPipeline = status == null
                        || (status.getName().isEmpty() && status.getPipelineType().isEmpty())
                        ? -1 : status.getPipelineIndex();
            }
        }, 0, 250, TimeUnit.MILLISECONDS);
    }

    void update() {
        LLResult result = camera == null ? null : camera.getLatestResult();
        if (result == null || !result.isValid() || result.getPipelineIndex() != selectedPipeline
                || result.getStaleness() > MAX_FRAME_AGE_MS
                || !Double.isFinite(result.getTimestamp())) {
            clearTarget();
            return;
        }

        // A successful HTTP poll can return a repeat of the same camera frame.
        long now = System.nanoTime();
        double timestamp = result.getTimestamp();
        if (timestamp == frameTimestamp) {
            if (now - frameFirstSeenNanos > TimeUnit.MILLISECONDS.toNanos(MAX_FRAME_AGE_MS)) {
                clearTarget();
            }
            return;
        }
        frameTimestamp = timestamp;
        frameFirstSeenNanos = now;
        clearTarget();

        List<FiducialResult> tags = result.getFiducialResults();
        if (tags == null) {
            return;
        }
        StringBuilder ids = new StringBuilder();
        for (FiducialResult tag : tags) {
            if (tag == null || !acceptsTag(tag.getFiducialId())) {
                continue;
            }
            if (ids.length() > 0) {
                ids.append(", ");
            }
            ids.append(tag.getFiducialId());
            double tx = tag.getTargetXDegrees();
            if (Double.isFinite(tx) && Math.abs(tx) <= 90.0
                    && Double.isFinite(tag.getTargetYDegrees())
                    && (target == null || tag.getTargetArea() > target.getTargetArea())) {
                target = tag;
            }
        }
        tagIds = ids.toString();
    }

    private void clearTarget() {
        target = null;
        tagIds = "";
    }

    private boolean acceptsTag(int id) {
        switch (selectedPipeline) {
            case InitOpMode.PIPE_RED_HIVE:
                return id >= 30 && id <= 37;
            case InitOpMode.PIPE_BLUE_HIVE:
                return id >= 38 && id <= 45;
            default:
                return true;
        }
    }

    double getTargetTx() {
        return target == null ? Double.NaN : target.getTargetXDegrees();
    }

    double getFrameTimestamp() {
        return frameTimestamp;
    }

    void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Limelight selected", pipelineLabel(selectedPipeline));
        int active = runningPipeline;
        telemetry.addData("Limelight running", active < 0 ? "Unavailable" : pipelineLabel(active));
        if (camera == null) {
            telemetry.addLine("Warning: Limelight missing from robot configuration");
        } else if (active >= 0 && active != selectedPipeline) {
            telemetry.addLine("Warning: running Limelight pipeline differs from selection");
        } else if (active < 0 && !switchAccepted) {
            telemetry.addLine("Warning: Limelight pipeline switch failed");
        }
        telemetry.addData("Tag seen", tagIds.isEmpty() ? "No" : "Yes");
        telemetry.addData("Tag IDs", tagIds.isEmpty() ? "None" : tagIds);
        if (target != null) {
            telemetry.addData("Aim tag", target.getFiducialId());
            telemetry.addData("tx", "%.1f deg", target.getTargetXDegrees());
            telemetry.addData("ty", "%.1f deg", target.getTargetYDegrees());
            double meters = distanceMeters(target);
            if (Double.isFinite(meters) && meters > 0.0) {
                telemetry.addData("Distance", "%.2f m", meters);
            } else {
                telemetry.addData("Distance", "Unavailable");
            }
        } else {
            telemetry.addData("tx", "Unavailable");
            telemetry.addData("ty", "Unavailable");
            telemetry.addData("Distance", "Unavailable");
        }
    }

    private static double distanceMeters(FiducialResult tag) {
        Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null || pose.getPosition() == null) {
            return 0.0;
        }
        Position position = pose.getPosition().toUnit(DistanceUnit.METER);
        return Math.hypot(Math.hypot(position.x, position.y), position.z);
    }

    private static String pipelineLabel(int pipeline) {
        switch (pipeline) {
            case InitOpMode.PIPE_ANY_HIVE:
                return "Pipeline 0 AnyHive";
            case InitOpMode.PIPE_RED_HIVE:
                return "Pipeline 1 RedHive";
            case InitOpMode.PIPE_BLUE_HIVE:
                return "Pipeline 2 BlueHive";
            default:
                return "Pipeline " + pipeline + " (unknown)";
        }
    }

    void stop() {
        if (statusPoller != null) {
            statusPoller.shutdownNow();
        }
        if (camera != null) {
            camera.stop();
        }
    }
}
