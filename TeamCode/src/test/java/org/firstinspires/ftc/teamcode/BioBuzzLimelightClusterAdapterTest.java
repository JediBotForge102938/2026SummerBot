package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.Alliance;
import org.firstinspires.ftc.teamcode.BioBuzzLimelightClusterAdapter.ClusterDetection;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

public class BioBuzzLimelightClusterAdapterTest {
    @Test
    public void identifiesClusterFromOneVisibleTag() {
        LLResult result = mock(LLResult.class);
        LLResultTypes.FiducialResult tag = mock(LLResultTypes.FiducialResult.class);
        when(result.isValid()).thenReturn(true);
        when(result.getFiducialResults()).thenReturn(Collections.singletonList(tag));
        when(tag.getFiducialId()).thenReturn(38);
        when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, 12, 3, 100, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)));

        List<ClusterDetection> clusters =
                new BioBuzzLimelightClusterAdapter().detect(result, Alliance.BLUE);

        assertEquals(1, clusters.size());
        ClusterDetection cluster = clusters.get(0);
        assertEquals("BLUE AUDIENCE", cluster.name);
        assertEquals(Collections.singletonList(38), cluster.visibleTagIds);
        assertEquals(Collections.singletonList(38), cluster.visibleTagIdsLeftToRight);
        assertEquals(25, cluster.percentClusterFound);
        assertTrue(cluster.hasPose());
    }
}
