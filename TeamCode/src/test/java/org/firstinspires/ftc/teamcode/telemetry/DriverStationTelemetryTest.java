package org.firstinspires.ftc.teamcode.telemetry;

import static org.mockito.Mockito.*;

import android.content.Context;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;
import org.mockito.InOrder;

import java.util.Locale;

public class DriverStationTelemetryTest {
    @Test
    public void coloredValuesStyleBothLabelAndValueAndEscapeText() {
        Telemetry sdk = mock(Telemetry.class);
        RobotTelemetry output = new DriverStationTelemetry(sdk);
        output.coloredValue("Tracking alliance", "RED", RobotTelemetry.Color.RED);
        output.coloredValue("Alliance <selected>", "BLUE & ready", RobotTelemetry.Color.BLUE);
        verify(sdk).addLine("<b><font color='#FF6666'>Tracking alliance: RED</font></b>");
        verify(sdk).addLine("<b><font color='#66AAFF'>Alliance &lt;selected&gt;: BLUE &amp; ready</font></b>");
    }

    @Test
    public void coloredValuesRemainReadableInSimulator() {
        Telemetry sdk = mock(Telemetry.class);
        RobotTelemetry output = new DriverStationTelemetry(sdk, false);
        output.coloredValue("Tracking alliance", "BLUE", RobotTelemetry.Color.BLUE);
        verify(sdk).addLine("Tracking alliance: BLUE");
    }

    @Test
    public void frameOwnsFormattingClearAndSingleExplicitUpdate() {
        Telemetry sdk = mock(Telemetry.class);
        RobotTelemetry output = new DriverStationTelemetry(sdk);

        output.beginFrame();
        output.value("Mode", "FIELD-CENTRIC");
        output.endFrame();

        InOrder order = inOrder(sdk);
        order.verify(sdk).setDisplayFormat(Telemetry.DisplayFormat.HTML);
        order.verify(sdk).setAutoClear(false);
        order.verify(sdk).clearAll();
        order.verify(sdk).addLine("<b>Mode:</b> FIELD-CENTRIC");
        order.verify(sdk).update();
        verifyNoMoreInteractions(sdk);
    }

    @Test
    public void escapesAllExternalTextRatherThanInterpretingMarkup() {
        Telemetry sdk = mock(Telemetry.class);
        RobotTelemetry output = new DriverStationTelemetry(sdk);

        output.value("<label>", "<b>A&B</b> \"quoted\" 'value'");
        output.status("Status & state", "<READY>", RobotTelemetry.Level.SUCCESS);
        output.line("<font color='red'>Not markup</font>");
        output.number("Speed", 25, "<ticks/s>");

        verify(sdk).addLine("<b>&lt;label&gt;:</b> &lt;b&gt;A&amp;B&lt;/b&gt;"
                + " &quot;quoted&quot; &#39;value&#39;");
        verify(sdk).addLine("<b>Status &amp; state: <font color='#66FF99'>&lt;READY&gt;</font></b>");
        verify(sdk).addLine("&lt;font color=&#39;red&#39;&gt;Not markup&lt;/font&gt;");
        verify(sdk).addLine("<b>Speed:</b> 25.00 &lt;ticks/s&gt;");
    }

    @Test
    public void statusLevelsKeepReadableTextAlongsideColor() {
        Telemetry sdk = mock(Telemetry.class);
        RobotTelemetry output = new DriverStationTelemetry(sdk);

        output.status("Launcher", "OFF", RobotTelemetry.Level.INFO);
        output.status("Launcher", "READY", RobotTelemetry.Level.SUCCESS);
        output.status("Launcher", "BELOW TARGET", RobotTelemetry.Level.WARNING);
        output.status("Fault", "IMU unavailable", RobotTelemetry.Level.ERROR);

        verify(sdk).addLine("<b>Launcher: <font color='#66CCFF'>OFF</font></b>");
        verify(sdk).addLine("<b>Launcher: <font color='#66FF99'>READY</font></b>");
        verify(sdk).addLine("<b>Launcher: <font color='#FFCC66'>BELOW TARGET</font></b>");
        verify(sdk).addLine("<b>Fault: <font color='#FF6666'>IMU unavailable</font></b>");
    }

    @Test
    public void numberFormattingDoesNotDependOnDeviceLocale() {
        Locale previous = Locale.getDefault();
        try {
            Locale.setDefault(Locale.GERMANY);
            Telemetry sdk = mock(Telemetry.class);
            RobotTelemetry output = new DriverStationTelemetry(sdk);
            output.number("Velocity", 1250.25, "ticks/s");
            output.number("Trigger", 0.5, "");

            verify(sdk).addLine("<b>Velocity:</b> 1250.25 ticks/s");
            verify(sdk).addLine("<b>Trigger:</b> 0.50");
        } finally {
            Locale.setDefault(previous);
        }
    }

    @Test
    public void everyFrameReplacesPreviousContent() {
        Telemetry sdk = mock(Telemetry.class);
        RobotTelemetry output = new DriverStationTelemetry(sdk);

        output.beginFrame();
        output.value("Old", 1);
        output.endFrame();
        output.beginFrame();
        output.value("New", 2);
        output.endFrame();

        verify(sdk, times(2)).clearAll();
        verify(sdk, times(2)).update();
    }

    @Test
    public void simulatorContextUsesReadablePlainTextWithoutHtmlEntities() {
        Telemetry sdk = mock(Telemetry.class);
        Context context = mock(Context.class);
        when(context.getPackageName()).thenReturn("virtual_robot");
        RobotTelemetry output = new DriverStationTelemetry(sdk, context);

        output.value("Mode", "A & B");
        output.status("Launcher", "READY", RobotTelemetry.Level.SUCCESS);
        output.line("Target < limit");
        output.number("Velocity", 1250, "ticks/s");

        verify(sdk).setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
        verify(sdk).addLine("Mode: A & B");
        verify(sdk).addLine("Launcher: [SUCCESS] READY");
        verify(sdk).addLine("Target < limit");
        verify(sdk).addLine("Velocity: 1250.00 ticks/s");
    }

    @Test
    public void robotControllerContextUsesStyledTelemetry() {
        Telemetry sdk = mock(Telemetry.class);
        Context context = mock(Context.class);
        when(context.getPackageName()).thenReturn("com.qualcomm.ftcrobotcontroller");
        RobotTelemetry output = new DriverStationTelemetry(sdk, context);

        output.status("Launcher", "READY", RobotTelemetry.Level.SUCCESS);

        verify(sdk).setDisplayFormat(Telemetry.DisplayFormat.HTML);
        verify(sdk).addLine("<b>Launcher: <font color='#66FF99'>READY</font></b>");
    }
}
