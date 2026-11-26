package virtual_robot.games;

import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Rectangle;
import org.firstinspires.ftc.teamcode.control.BioBuzzFieldMap;
import org.firstinspires.ftc.teamcode.subsystems.FieldMap;
import virtual_robot.controller.Filters;
import virtual_robot.controller.Game;
import virtual_robot.controller.VirtualField;

/**
 * BioBuzz field collision geometry.
 *
 * The background image supplies the field artwork; these static bodies keep the
 * perimeter flowers and the central frame from being driven through.
 */
public final class BioBuzz extends Game {
    @Override
    public void initialize() {
        super.initialize();

        for (FieldMap.Region flower : BioBuzzFieldMap.flowerObstacles()) {
            addStaticRectangle((flower.minX + flower.maxX) / 2,
                    (flower.minY + flower.maxY) / 2,
                    flower.maxX - flower.minX, flower.maxY - flower.minY);
        }
        FieldMap.Region frame = BioBuzzFieldMap.frameObstacle();
        addStaticRectangle(0, 0, frame.maxX - frame.minX, frame.maxY - frame.minY);
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
