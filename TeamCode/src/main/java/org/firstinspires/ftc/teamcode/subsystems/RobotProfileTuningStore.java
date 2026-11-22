package org.firstinspires.ftc.teamcode.subsystems;

import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.io.StringReader;
import java.nio.charset.StandardCharsets;

/** Patches one tuning value without replacing unrelated on-controller profile settings. */
public class RobotProfileTuningStore {
    private final File directory;
    private final RobotProfile profile;

    public RobotProfileTuningStore(RobotProfile profile) {
        this(new File("/sdcard/FIRST/robot-profiles"), profile);
    }

    public RobotProfileTuningStore(File directory, RobotProfile profile) {
        this.directory = directory;
        this.profile = profile;
    }

    public void save(String key, double value, boolean trialSetting) throws IOException {
        if (profile.isSimulatorRuntime()) {
            throw new IOException("Simulator profiles are bundled resources; saving is unavailable");
        }
        File path = new File(directory, profile.name + ".profile.json");
        JsonObject root;
        try (Reader reader = new FileReader(path)) {
            root = new JsonParser().parse(reader).getAsJsonObject();
        }
        String section = trialSetting ? "tuning" : "navigation";
        if (!root.has(section)) root.add(section, new JsonObject());
        root.getAsJsonObject(section).addProperty(key, value);
        String content = new GsonBuilder().setPrettyPrinting().create().toJson(root) + "\n";
        RobotProfile.read(profile.name, new StringReader(content), profile.robot);
        File temporary = File.createTempFile(profile.name + "-", ".tmp", directory);
        try {
            try (FileOutputStream output = new FileOutputStream(temporary)) {
                output.write(content.getBytes(StandardCharsets.UTF_8));
                output.getFD().sync();
            }
            if (!temporary.renameTo(path)) {
                throw new IOException("Unable to atomically replace controller profile " + path);
            }
        } finally {
            if (temporary.exists() && !temporary.delete()) {
                throw new IOException("Unable to remove temporary profile " + temporary);
            }
        }
    }
}
