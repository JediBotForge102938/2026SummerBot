public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.23849136707372778);
            Controller secondaryTranslationalForward = Controller.proportional(0.08811618752704822);
            Controller primaryTranslationalLateral = Controller.proportional(0.38335818601720417);
            Controller secondaryTranslationalLateral = Controller.proportional(0.14164060621396896);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.014697182605137674));
            c.brake.set(Controller.proportionalFeedforward(0.012492605214367023));

            c.headingFeedback.set(Controller.proportional(3.8842560385598235));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.8600445150401718, -0.3043591868677641));

            c.linearBrakeCoefficients.set(Matrix.diag(0.13188971715603842, 0.047777694467240193));
            c.quadraticBrakeCoefficients.set(Matrix.diag(-7.919242486844212E-4, 0.0017580708733732505));

            c.maxAchievableForwardVelocity.set(65.72126044392431);
            c.maxAchievableStrafeVelocity.set(52.788203832778166);
            c.naturalForwardDeceleration.set(45.59893906558101);
            c.naturalStrafeDeceleration.set(83.63033039067729);
        }
);

public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
    c.name.set("otos");
    c.linearScalar.set(1.0903587772925765);
    c.angularScalar.set(0.9939636606303289);
    c.offset.set(new Pose(-0.0, -0.0, Math.toRadians(-90.0)));
    c.linearUnit.set(DistanceUnit.INCH);
});