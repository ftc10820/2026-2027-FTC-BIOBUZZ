package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDConstants {
    public static double KP = 135.0;
    public static double KI = 0.0;
    public static double KD = 30.0;
    public static double KF = 25.6;
    public static double TARGET_VELOCITY = 0.0;
    public static double SHOOTER_IDLE_VELOCITY = PIDConstants.rpmToTicks(900.0d);

    public static double TICKS_PER_REVOLUTION = 28 * 2;

    public static double rpmToTicks(double rpm) {
        return rpm * PIDConstants.TICKS_PER_REVOLUTION / 60;
    }

    public static double rpsToTicks(double rps) {
        return rps * PIDConstants.TICKS_PER_REVOLUTION;
    }

    public static double ticksToRpm(double ticks) {
        return ticks / PIDConstants.TICKS_PER_REVOLUTION;
    }

    public static double ticksToRps(double ticks) {
        return ticks * 60 / PIDConstants.TICKS_PER_REVOLUTION;
    }
}
