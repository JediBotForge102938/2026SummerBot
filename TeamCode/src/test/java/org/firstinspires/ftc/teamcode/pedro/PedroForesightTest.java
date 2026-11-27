package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.*;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import java.io.File;
import org.firstinspires.ftc.teamcode.subsystems.RobotProfile;
import org.junit.Test;

public class PedroForesightTest {
    @Test
    public void physicalProfilesHaveFiniteSignedTranslationBrakingTargets() {
        File profiles = new File("../robot-configs");
        for (String robot : new String[] {"starterbot", "competitionbot"}) {
            RobotProfile profile = RobotProfile.load(profiles, "biobuzz-" + robot, robot);
            Foresight foresight = new Foresight(PedroConstants.foresightConfig(profile));
            for (Vector2D direction : new Vector2D[] {Vector2D.iHat(), Vector2D.jHat()}) {
                for (double distance : new double[] {-48, -4, 0, 4, 48, 144}) {
                    double velocity = foresight.getVelocityToBrakeInTime(distance, direction, 0);
                    assertTrue(robot + ": invalid braking target at " + distance,
                            Double.isFinite(velocity));
                    assertEquals(Math.signum(distance), Math.signum(velocity), 0);
                }
            }
            Pose predicted = foresight.getBrakeDisplacement(new Twist(10, 5, 1), 0);
            assertTrue(Double.isFinite(predicted.x()));
            assertTrue(Double.isFinite(predicted.y()));
            assertTrue(Double.isFinite(predicted.heading()));
        }
    }
}
