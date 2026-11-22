package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.pedropathing.follower.Follower;
import java.io.File;
import java.io.StringReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import org.firstinspires.ftc.teamcode.control.ProfiledPedroFollower;
import org.junit.Test;

public class ProfiledPedroFollowerTest {
    private String json() throws Exception {
        return new String(Files.readAllBytes(new File(
                "../robot-configs/biobuzz-competitionbot.profile.json").toPath()), StandardCharsets.UTF_8);
    }

    private RobotProfile read(String json) {
        return RobotProfile.read("test", new StringReader(json), "competitionbot");
    }

    @Test public void sharedFollowerUsesProfileCoefficientsAndMeasuredConstants() throws Exception {
        RobotProfile profile = read(json().replace("\"headingF\": 0.0", "\"headingF\": 0.08")
                .replace("\"translationalF\": 0.0", "\"translationalF\": 0.06")
                .replace("\"driveF\": 0.0", "\"driveF\": 0.04")
                .replace("\"forwardVelocity\": 25.0", "\"massKg\": 12.0,"
                        + "\"forwardZeroPowerAcceleration\": -30.0,"
                        + "\"lateralZeroPowerAcceleration\": -40.0,\"forwardVelocity\": 35.0"));
        Follower follower = ProfiledPedroFollower.create(mock(DriveSubsystem.class),
                mock(TrackingSubsystem.class), profile, 0.5);
        assertEquals(0.5, follower.getConstants().getCoefficientsHeadingPIDF().P, 0);
        assertEquals(0.08, follower.getConstants().getCoefficientsHeadingPIDF().F, 0);
        assertEquals(0.06, follower.getConstants().getCoefficientsTranslationalPIDF().F, 0);
        assertEquals(0.04, follower.getConstants().getCoefficientsDrivePIDF().F, 0);
        assertEquals(12, follower.getConstants().getMass(), 0);
        assertEquals(-30, follower.getConstants().getForwardZeroPowerAcceleration(), 0);
        assertEquals(-40, follower.getConstants().getLateralZeroPowerAcceleration(), 0);
        assertEquals(35, follower.getDrivetrain().xVelocity(), 0);
        assertEquals(profile.navigation.transitPower, follower.getMaxPowerScaling(), 0);
    }

    @Test public void optionalConstantsRetainOldDefaultsAndRejectInvalidInputs() throws Exception {
        RobotProfile old = read(json().replace("    \"headingF\": 0.0,\n", "")
                .replace("    \"driveF\": 0.0,\n", "").replace("    \"translationalF\": 0.0,\n", ""));
        assertEquals(0, old.navigation.headingF, 0);
        assertEquals(10.65, old.navigation.massKg, 0);
        assertEquals(-41.278, old.navigation.forwardZeroPowerAcceleration, 0);
        assertEquals(-59.7819, old.navigation.lateralZeroPowerAcceleration, 0);
        for (String replacement : new String[] {"-0.1", "1.1"}) {
            assertThrows(IllegalArgumentException.class,
                    () -> read(json().replace("\"headingF\": 0.0", "\"headingF\": " + replacement)));
        }
        for (String addition : new String[] {"\"massKg\": 0,", "\"forwardZeroPowerAcceleration\": 0,",
                "\"lateralZeroPowerAcceleration\": 10,"}) {
            assertThrows(IllegalArgumentException.class,
                    () -> read(json().replace("\"headingP\":", addition + "\"headingP\":")));
        }
    }
}
