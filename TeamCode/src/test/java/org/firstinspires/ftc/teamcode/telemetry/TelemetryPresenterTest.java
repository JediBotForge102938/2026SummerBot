package org.firstinspires.ftc.teamcode.telemetry;

import static org.junit.Assert.assertThrows;
import static org.mockito.Mockito.*;

import org.junit.Test;
import org.mockito.InOrder;

public class TelemetryPresenterTest {
    @Test
    public void customOutputReceivesOneCompleteFrameInSourceOrder() {
        RobotTelemetry output = mock(RobotTelemetry.class);
        TelemetrySource launcher = sink -> sink.status("Launcher", "READY", RobotTelemetry.Level.SUCCESS);
        TelemetrySource drive = sink -> sink.number("Heading", 90, "degrees");
        TelemetryPresenter presenter = new TelemetryPresenter(output, launcher, drive);

        presenter.publish();

        InOrder order = inOrder(output);
        order.verify(output).beginFrame();
        order.verify(output).status("Launcher", "READY", RobotTelemetry.Level.SUCCESS);
        order.verify(output).number("Heading", 90, "degrees");
        order.verify(output).endFrame();
        verifyNoMoreInteractions(output);
    }

    @Test
    public void sourceListCannotBeChangedAfterConstruction() {
        RobotTelemetry output = mock(RobotTelemetry.class);
        TelemetrySource original = mock(TelemetrySource.class);
        TelemetrySource replacement = mock(TelemetrySource.class);
        TelemetrySource[] sources = {original};
        TelemetryPresenter presenter = new TelemetryPresenter(output, sources);
        sources[0] = replacement;

        presenter.publish();
        presenter.publish();

        verify(original, times(2)).publishTelemetry(output);
        verifyNoInteractions(replacement);
        verify(output, times(2)).beginFrame();
        verify(output, times(2)).endFrame();
    }

    @Test
    public void sourceFailurePropagatesWithoutSendingAnIncompleteFrame() {
        RobotTelemetry output = mock(RobotTelemetry.class);
        TelemetrySource broken = sink -> {
            throw new IllegalStateException("Cannot read sensor");
        };
        TelemetryPresenter presenter = new TelemetryPresenter(output, broken);

        assertThrows(IllegalStateException.class, () -> presenter.publish());

        verify(output, never()).endFrame();
    }
}
