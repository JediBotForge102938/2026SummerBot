package org.firstinspires.ftc.teamcode.subsystems;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/** Validated sensor configuration paired with an FTC hardware configuration. */
public final class RobotProfile {
    public final String name;
    public final String robot;
    public final double chassisLengthCm;
    public final double chassisWidthCm;
    public final String imuName;
    public final RevHubOrientationOnRobot.LogoFacingDirection imuLogo;
    public final RevHubOrientationOnRobot.UsbFacingDirection imuUsb;
    public final String limelightName;
    public final int limelightPipeline;
    public final double limelightFromBackCm;
    public final double limelightFromLeftCm;
    public final double limelightHeightCm;
    public final double limelightYawDegrees;
    public final double limelightPitchUpDegrees;
    public final double limelightRollDegrees;
    public final String otosName;
    public final double otosBackCm;
    public final double otosLeftCm;
    public final double otosLinearScalar;
    public final double otosAngularScalar;

    private RobotProfile(String name, JsonObject root) {
        this.name = name;
        requireKnownKeys(root, "schemaVersion", "robot", "chassis", "imu", "limelight", "otos");
        if (integer(root, "schemaVersion") != 1) {
            throw new IllegalArgumentException("Unsupported profile schema version");
        }
        robot = text(root, "robot");
        JsonObject chassis = object(root, "chassis");
        requireKnownKeys(chassis, "lengthCm", "widthCm");
        chassisLengthCm = number(chassis, "lengthCm");
        chassisWidthCm = number(chassis, "widthCm");
        if (chassisLengthCm <= 0 || chassisWidthCm <= 0) {
            throw new IllegalArgumentException("Chassis dimensions must be positive");
        }
        JsonObject imu = object(root, "imu");
        requireKnownKeys(imu, "name", "logoFacing", "usbFacing");
        imuName = text(imu, "name");
        imuLogo = RevHubOrientationOnRobot.LogoFacingDirection.valueOf(text(imu, "logoFacing"));
        imuUsb = RevHubOrientationOnRobot.UsbFacingDirection.valueOf(text(imu, "usbFacing"));
        if (imuLogo.name().equals(imuUsb.name())
                || opposite(imuLogo.name()).equals(imuUsb.name())) {
            throw new IllegalArgumentException("IMU logo and USB directions must be perpendicular");
        }
        JsonObject limelight = object(root, "limelight");
        requireKnownKeys(limelight, "name", "pipeline", "aimDirection", "lensFromBackCm",
                "lensFromLeftCm", "lensHeightCm", "yawDegrees", "pitchUpDegrees", "rollDegrees");
        limelightName = text(limelight, "name");
        limelightPipeline = integer(limelight, "pipeline");
        limelightFromBackCm = number(limelight, "lensFromBackCm");
        limelightFromLeftCm = number(limelight, "lensFromLeftCm");
        limelightHeightCm = number(limelight, "lensHeightCm");
        limelightYawDegrees = number(limelight, "yawDegrees");
        limelightPitchUpDegrees = number(limelight, "pitchUpDegrees");
        limelightRollDegrees = number(limelight, "rollDegrees");
        if (limelightHeightCm < 0) {
            throw new IllegalArgumentException("Limelight height must not be negative");
        }
        if (!"launcher".equals(text(limelight, "aimDirection"))) {
            throw new IllegalArgumentException("Only a Limelight facing the launcher is supported");
        }
        if (limelightPipeline < 0 || limelightPipeline > 9) {
            throw new IllegalArgumentException("Limelight pipeline must be 0-9");
        }
        JsonObject otos = object(root, "otos");
        requireKnownKeys(otos, "name", "rawAxes", "opticalCenterFromBackCm",
                "opticalCenterFromLeftCm", "linearScalar", "angularScalar");
        otosName = text(otos, "name");
        if (!"forward-negative-x-right-positive-y-clockwise-negative"
                .equals(text(otos, "rawAxes"))) {
            throw new IllegalArgumentException("Unsupported OTOS field-navigation axes");
        }
        otosBackCm = number(otos, "opticalCenterFromBackCm");
        otosLeftCm = number(otos, "opticalCenterFromLeftCm");
        otosLinearScalar = number(otos, "linearScalar");
        otosAngularScalar = number(otos, "angularScalar");
        if (otosBackCm < 0 || otosBackCm > chassisLengthCm
                || otosLeftCm < 0 || otosLeftCm > chassisWidthCm) {
            throw new IllegalArgumentException("OTOS optical center must lie within the chassis");
        }
        if (otosLinearScalar < 0.872 || otosLinearScalar > 1.127
                || otosAngularScalar < 0.872 || otosAngularScalar > 1.127) {
            throw new IllegalArgumentException("OTOS scalars must be between 0.872 and 1.127");
        }
    }

    public static RobotProfile loadActive(String requiredRobot) {
        return ActiveRobotProfileSource.load(requiredRobot);
    }

    public static RobotProfile load(File directory, String active, String requiredRobot) {
        if (active == null || !active.matches("[A-Za-z0-9][A-Za-z0-9 _.-]*")) {
            throw new IllegalStateException("Select a valid named FTC hardware configuration");
        }
        File path = new File(directory, active + ".profile.json");
        try (Reader reader = new FileReader(path)) {
            return read(active, reader, requiredRobot);
        } catch (FileNotFoundException exception) {
            throw new IllegalStateException("Missing profile for active configuration " + active
                    + ": " + path, exception);
        } catch (IOException | RuntimeException exception) {
            throw new IllegalStateException("Invalid robot profile " + path + ": "
                    + exception.getMessage(), exception);
        }
    }

    static RobotProfile read(String name, Reader reader, String requiredRobot) {
        RobotProfile profile = parse(name, reader);
        if (requiredRobot != null && !requiredRobot.equals(profile.robot)) {
            throw new IllegalArgumentException("Profile is for " + profile.robot
                    + ", not " + requiredRobot);
        }
        return profile;
    }

    private static RobotProfile parse(String name, Reader reader) {
        JsonElement element = new JsonParser().parse(reader);
        if (!element.isJsonObject()) {
            throw new IllegalArgumentException("Profile must be a JSON object");
        }
        return new RobotProfile(name, element.getAsJsonObject());
    }

    private static JsonObject object(JsonObject parent, String key) {
        JsonElement value = parent.get(key);
        if (value == null || !value.isJsonObject()) {
            throw new IllegalArgumentException("Missing object: " + key);
        }
        return value.getAsJsonObject();
    }

    private static void requireKnownKeys(JsonObject object, String... allowed) {
        Set<String> names = new HashSet<>(Arrays.asList(allowed));
        for (java.util.Map.Entry<String, JsonElement> entry : object.entrySet()) {
            if (!names.contains(entry.getKey())) {
                throw new IllegalArgumentException("Unsupported profile setting: " + entry.getKey());
            }
        }
    }

    private static String text(JsonObject parent, String key) {
        JsonElement value = parent.get(key);
        if (value == null || !value.isJsonPrimitive() || !value.getAsJsonPrimitive().isString()
                || value.getAsString().trim().isEmpty()) {
            throw new IllegalArgumentException("Missing string: " + key);
        }
        return value.getAsString();
    }

    private static double number(JsonObject parent, String key) {
        JsonElement value = parent.get(key);
        if (value == null || !value.isJsonPrimitive() || !value.getAsJsonPrimitive().isNumber()
                || !Double.isFinite(value.getAsDouble())) {
            throw new IllegalArgumentException("Missing finite number: " + key);
        }
        return value.getAsDouble();
    }

    private static int integer(JsonObject parent, String key) {
        double value = number(parent, key);
        if (value != Math.rint(value) || value < Integer.MIN_VALUE || value > Integer.MAX_VALUE) {
            throw new IllegalArgumentException("Expected integer: " + key);
        }
        return (int) value;
    }

    private static String opposite(String direction) {
        switch (direction) {
            case "UP": return "DOWN";
            case "DOWN": return "UP";
            case "LEFT": return "RIGHT";
            case "RIGHT": return "LEFT";
            case "FORWARD": return "BACKWARD";
            case "BACKWARD": return "FORWARD";
            default: throw new IllegalArgumentException("Invalid orientation: " + direction);
        }
    }
}
