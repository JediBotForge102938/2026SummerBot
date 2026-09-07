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
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.xml.sax.SAXException;

public final class RobotHardwareConfig {
    private RobotHardwareConfig() {
    }

    public record Motor(String name, int port) {
    }

    public static List<Motor> loadMotors(String resourcePath) {
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
            List<Motor> motors = new ArrayList<>();
            Set<String> names = new HashSet<>();
            Set<Integer> ports = new HashSet<>();

            for (int i = 0; i < nodes.getLength(); i++) {
                Node node = nodes.item(i);
                if (!(node instanceof Element element) || !element.getTagName().toLowerCase().contains("motor")) {
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
                    throw new IllegalArgumentException("Duplicate motor name in " + resourcePath + ": " + name);
                }
                if (!ports.add(port)) {
                    throw new IllegalArgumentException("Duplicate motor port in " + resourcePath + ": " + port);
                }
                motors.add(new Motor(name, port));
            }

            if (motors.isEmpty()) {
                throw new IllegalArgumentException("No motors found in " + resourcePath);
            }
            return List.copyOf(motors);
        } catch (IllegalArgumentException exception) {
            throw exception;
        } catch (IOException | ParserConfigurationException | SAXException exception) {
            throw new IllegalStateException("Unable to load robot configuration " + resourcePath, exception);
        }
    }
}
