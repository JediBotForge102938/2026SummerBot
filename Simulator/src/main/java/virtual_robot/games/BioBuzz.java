package virtual_robot.games;

import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Circle;
import org.dyn4j.geometry.Rectangle;
import virtual_robot.controller.Filters;
import virtual_robot.controller.Game;
import virtual_robot.controller.VirtualField;

/**
 * BioBuzz field collision geometry.
 *
 * The background image supplies the field artwork; these static bodies keep the
 * perimeter flowers and the two north-south hive bars from being driven through.
 */
public final class BioBuzz extends Game {
    private static final double FLOWER_RADIUS_INCHES = 2.5;
    private static final double HIVE_BAR_WIDTH_INCHES = 1.5;
    private static final double HIVE_BAR_LENGTH_INCHES = 40.5;

    private static final double[][] FLOWER_CENTERS_INCHES = {
            {-24.0, 69.0},
            {69.0, 24.0},
            {24.0, -69.0},
            {-69.0, -24.0}
    };

    private static final double[] HIVE_BAR_X_POSITIONS_INCHES = {-24.5, 24.5};

    @Override
    public void initialize() {
        super.initialize();

        for (double[] center : FLOWER_CENTERS_INCHES) {
            addStaticCircle(center[0], center[1], FLOWER_RADIUS_INCHES);
        }
        for (double x : HIVE_BAR_X_POSITIONS_INCHES) {
            addStaticRectangle(x, 0, HIVE_BAR_WIDTH_INCHES, HIVE_BAR_LENGTH_INCHES);
        }
    }

    private void addStaticCircle(double xInches, double yInches, double radiusInches) {
        Circle circle = new Circle(toMeters(radiusInches));
        circle.translate(toMeters(xInches), toMeters(yInches));
        addStaticBody(circle);
    }

    private void addStaticRectangle(
            double xInches, double yInches, double widthInches, double heightInches) {
        Rectangle rectangle = new Rectangle(toMeters(widthInches), toMeters(heightInches));
        rectangle.translate(toMeters(xInches), toMeters(yInches));
        addStaticBody(rectangle);
    }

    private void addStaticBody(org.dyn4j.geometry.Convex shape) {
        Body body = new Body();
        body.addFixture(shape).setFilter(Filters.WALL_FILTER);
        body.setMass(org.dyn4j.geometry.MassType.INFINITE);
        world.addBody(body);
    }

    private static double toMeters(double inches) {
        return inches * VirtualField.conversionFactor(
                VirtualField.Unit.INCH, VirtualField.Unit.METER);
    }

    @Override
    public void resetGameElements() {
    }

    @Override
    public boolean hasHumanPlayer() {
        return false;
    }

    @Override
    public boolean isHumanPlayerAuto() {
        return false;
    }

    @Override
    public void setHumanPlayerAuto(boolean selected) {
    }

    @Override
    public void updateHumanPlayerState(double millis) {
    }
}
