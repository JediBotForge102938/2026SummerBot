package org.firstinspires.ftc.teamcode.pedro.procedures;

import static org.mockito.Mockito.*;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;
import org.mockito.MockedStatic;

public class BrakingDiagnosticsTest {
    @Test
    public void reportsMeasuredMotionAndActualCommandWithoutReadingAtLoopRate() {
        Telemetry telemetry = mock(Telemetry.class);
        Localizer localizer = mock(Localizer.class);
        when(localizer.pose()).thenReturn(new Pose(12, 3, Math.PI / 2));
        when(localizer.velocity()).thenReturn(new Velocity(5, 1, Math.PI));
        DrivePowers command = new DrivePowers(0.3, 0, -0.15);
        BrakingDiagnostics diagnostics = new BrakingDiagnostics(telemetry, "forward");
        try (MockedStatic<RobotLog> logs = mockStatic(RobotLog.class)) {
            diagnostics.record(0, "DRIVE", 1, localizer, command);
            diagnostics.record(0.05, "DRIVE", 1, localizer, command);
            diagnostics.record(0.11, "BRAKE", 1, localizer, command);
            verify(telemetry, times(2)).update();
            verify(localizer, times(2)).pose();
            verify(telemetry, times(2)).addData("Heading (deg)", "%.3f", 90.0);
            verify(telemetry, times(2)).addData("Angular velocity (deg/s)", "%.3f", 180.0);
            verify(telemetry, times(2)).addData("Commanded power",
                    "Forward %.4f Strafe %.4f Turn %.4f", 0.3, 0.0, -0.15);
            logs.verify(() -> RobotLog.ii(eq("PedroBraking"), anyString(),
                    eq("forward"), eq(0.11), eq(2), eq("BRAKE"), eq(12.0), eq(3.0),
                    eq(5.0), eq(1.0), eq(Math.PI / 2), eq(Math.PI), eq(0.3), eq(0.0), eq(-0.15)));
        }
    }
}
