package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class RobotProfileTuningStoreTest {
    @Rule public TemporaryFolder folder = new TemporaryFolder();

    private File copyProfile() throws Exception {
        File path = new File(folder.getRoot(), "biobuzz-competitionbot.profile.json");
        Files.copy(new File("../robot-configs/biobuzz-competitionbot.profile.json").toPath(), path.toPath());
        return path;
    }

    @Test public void persistsBothKindsOfSettingsAndReloadsWithoutChangingVerification() throws Exception {
        File path = copyProfile();
        RobotProfile original = RobotProfile.load(folder.getRoot(), "biobuzz-competitionbot", "competitionbot");
        RobotProfileTuningStore store = new RobotProfileTuningStore(folder.getRoot(), original);
        store.save("headingP", 1.25, false);
        store.save("testPower", 0.3, true);
        store.save("testDistanceIn", 15, true);
        RobotProfile saved = RobotProfile.load(folder.getRoot(), original.name, original.robot);
        assertEquals(1.25, saved.navigation.headingP, 0);
        assertEquals(0.3, saved.tuning.get("testPower").getAsDouble(), 0);
        assertEquals(15, saved.tuning.get("testDistanceIn").getAsDouble(), 0);
        assertEquals(original.navigation.pedroTuningVerified, saved.navigation.pedroTuningVerified);
        assertEquals(original.otosBackCm, saved.otosBackCm, 0);
        assertEquals(original.launcherFeedVelocityMargin, saved.launcherFeedVelocityMargin);
        assertEquals(1, folder.getRoot().listFiles().length);
        assertTrue(path.exists());
    }

    @Test public void invalidEditsLeaveExistingFileUnchanged() throws Exception {
        File path = copyProfile();
        RobotProfile profile = RobotProfile.load(folder.getRoot(), "biobuzz-competitionbot", "competitionbot");
        RobotProfileTuningStore store = new RobotProfileTuningStore(folder.getRoot(), profile);
        String before = new String(Files.readAllBytes(path.toPath()), StandardCharsets.UTF_8);
        assertThrows(IllegalArgumentException.class, () -> store.save("headingP", 0, false));
        assertThrows(IllegalArgumentException.class, () -> store.save("testPower", 2, true));
        assertThrows(IllegalArgumentException.class, () -> store.save("unknown", 1, true));
        assertEquals(before, new String(Files.readAllBytes(path.toPath()), StandardCharsets.UTF_8));
    }

    @Test public void patchesFreshDiskContentRatherThanStaleInitProfile() throws Exception {
        File path = copyProfile();
        RobotProfile profile = RobotProfile.load(folder.getRoot(), "biobuzz-competitionbot", "competitionbot");
        RobotProfileTuningStore store = new RobotProfileTuningStore(folder.getRoot(), profile);
        String updated = new String(Files.readAllBytes(path.toPath()), StandardCharsets.UTF_8)
                .replace("\"headingD\": 0.0", "\"headingD\": 0.02");
        Files.write(path.toPath(), updated.getBytes(StandardCharsets.UTF_8));
        store.save("headingP", 1.5, false);
        assertEquals(0.02, RobotProfile.load(folder.getRoot(), profile.name, profile.robot).navigation.headingD, 0);
    }
}
