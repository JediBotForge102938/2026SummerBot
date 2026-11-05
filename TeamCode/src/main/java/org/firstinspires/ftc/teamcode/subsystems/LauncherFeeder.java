package org.firstinspires.ftc.teamcode.subsystems;

/** Feeder output gated by combined launcher readiness. */
public interface LauncherFeeder {
    /** Deliberately leaves feeder hardware untouched until its travel is configured. */
    LauncherFeeder DISABLED = feed -> { };

    void update(boolean feed);
}
