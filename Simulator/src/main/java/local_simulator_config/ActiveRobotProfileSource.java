package org.firstinspires.ftc.teamcode.subsystems;

import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

public final class ActiveRobotProfileSource {
    private static volatile String selectedName;
    private ActiveRobotProfileSource() { }

    public static void select(String name) {
        selectedName = name;
    }

    static RobotProfile load(String requiredRobot) {
        String name = selectedName;
        if (name == null) {
            throw new IllegalStateException("Selected simulator configuration has no robot profile");
        }
        String path = "/robot-configs/" + name + ".profile.json";
        InputStream input = ActiveRobotProfileSource.class.getResourceAsStream(path);
        if (input == null) throw new IllegalStateException("Missing simulator robot profile: " + path);
        try (InputStreamReader reader = new InputStreamReader(input, StandardCharsets.UTF_8)) {
            return RobotProfile.read(name, reader, requiredRobot);
        } catch (IOException | RuntimeException exception) {
            throw new IllegalStateException("Invalid simulator robot profile " + path, exception);
        }
    }
}
