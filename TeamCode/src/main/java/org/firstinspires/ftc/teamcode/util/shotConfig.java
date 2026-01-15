package org.firstinspires.ftc.teamcode.util;

public class shotConfig {
    public final double rpm;
    public final double hoodPos;
    public final double turretOffset;

    public shotConfig(double rpm, double hoodPos, double turretOffset) {
        this.rpm = rpm;
        this.hoodPos = hoodPos;
        this.turretOffset = turretOffset;
    }

    public static shotConfig lerp(shotConfig a, shotConfig b, double t) {
        double rpm = a.rpm + (b.rpm - a.rpm) * t;
        double hood = a.hoodPos + (b.hoodPos - a.hoodPos) * t;
        double turretOffset = a.turretOffset + (b.turretOffset - a.turretOffset) * t;
        return new shotConfig(rpm, hood, turretOffset);
    }
}
