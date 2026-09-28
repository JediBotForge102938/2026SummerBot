package local_simulator_config;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.xml.XMLConstants;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.xml.sax.SAXException;

public final class RobotHardwareConfig {
    private RobotHardwareConfig() {
    }

    public enum DeviceType {
        MOTOR,
        SERVO
    }

    public record Device(String name, int port, DeviceType type) {
    }

    public record Motor(String name, int port) {
    }

    public static List<Motor> loadMotors(String resourcePath) {
        return loadDevices(resourcePath).stream()
                .filter(device -> device.type() == DeviceType.MOTOR)
                .map(device -> new Motor(device.name(), device.port()))
                .toList();
    }

    public static List<Device> loadDevices(String resourcePath) {
        try (InputStream input = RobotHardwareConfig.class.getResourceAsStream(resourcePath)) {
            if (input == null) {
                throw new IOException("Robot configuration resource not found: " + resourcePath);
            }

            DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
            factory.setFeature(XMLConstants.FEATURE_SECURE_PROCESSING, true);
            factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
            factory.setFeature("http://xml.org/sax/features/external-general-entities", false);
            factory.setFeature("http://xml.org/sax/features/external-parameter-entities", false);
            factory.setXIncludeAware(false);
            factory.setExpandEntityReferences(false);

            Document document = factory.newDocumentBuilder().parse(input);
            NodeList nodes = document.getElementsByTagName("*");
            List<Device> devices = new ArrayList<>();
            Set<String> names = new HashSet<>();
            Map<Node, Set<Integer>> motorPortsByModule = new HashMap<>();
            Map<Node, Set<Integer>> servoPortsByModule = new HashMap<>();

            for (int i = 0; i < nodes.getLength(); i++) {
                Node node = nodes.item(i);
                if (!(node instanceof Element element)) {
                    continue;
                }
                String tagName = element.getTagName().toLowerCase();
                DeviceType type;
                if (tagName.contains("motor")) {
                    type = DeviceType.MOTOR;
                } else if (tagName.contains("servo")) {
                    type = DeviceType.SERVO;
                } else {
                    continue;
                }

                String name = element.getAttribute("name").trim();
                String portValue = element.getAttribute("port").trim();
                if (name.isEmpty() || portValue.isEmpty()) {
                    throw new IllegalArgumentException("Motor entries must have name and port: " + element.getTagName());
                }

                int port;
                try {
                    port = Integer.parseInt(portValue);
                } catch (NumberFormatException exception) {
                    throw new IllegalArgumentException("Invalid motor port '" + portValue + "' for " + name, exception);
                }

                if (!names.add(name)) {
                    throw new IllegalArgumentException("Duplicate device name in " + resourcePath + ": " + name);
                }
                Node module = element.getParentNode();
                Map<Node, Set<Integer>> portsByModule =
                        type == DeviceType.MOTOR ? motorPortsByModule : servoPortsByModule;
                Set<Integer> ports = portsByModule.computeIfAbsent(module, ignored -> new HashSet<>());
                if (!ports.add(port)) {
                    throw new IllegalArgumentException(
                            "Duplicate " + type.name().toLowerCase() + " port in the same module in "
                                    + resourcePath + ": " + port);
                }
                devices.add(new Device(name, port, type));
            }

            if (devices.isEmpty()) {
                throw new IllegalArgumentException("No motors or servos found in " + resourcePath);
            }
            return List.copyOf(devices);
        } catch (IllegalArgumentException exception) {
            throw exception;
        } catch (IOException | ParserConfigurationException | SAXException exception) {
            throw new IllegalStateException("Unable to load robot configuration " + resourcePath, exception);
        }
    }
}
