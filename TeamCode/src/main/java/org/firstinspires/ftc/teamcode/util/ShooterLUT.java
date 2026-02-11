package org.firstinspires.ftc.teamcode.util;

import java.util.Map;
import java.util.TreeMap;

public class ShooterLUT {
    private final TreeMap<Double, shotConfig> table = new TreeMap<>();
    private final RobotHardware robot;
    public ShooterLUT() {
        this.robot = RobotHardware.getInstance();
        table.put(15.0, new shotConfig(0.55, 0.6, 0.0));
        table.put(5.0, new shotConfig(0.6, 0.45, 0.0));
        table.put(0.0, new shotConfig(0.74, 0.27, 0.0));
        table.put(-2.7, new shotConfig(0.82, 0.2, 0.0));
    }

    public shotConfig getForTargetY(double y) {

        Map.Entry<Double, shotConfig> floor = table.floorEntry(y);
        Map.Entry<Double, shotConfig> ceil = table.ceilingEntry(y);

        if (floor == null) return ceil.getValue();
        if (ceil == null) return floor.getValue();
        if (floor.getKey().equals(ceil.getKey())) return floor.getValue();

        double d0 = floor.getKey();
        double d1 = ceil.getKey();
        double t = (y - d0) / (d1 - d0);

        return shotConfig.lerp(floor.getValue(), ceil.getValue(), t);
    }
    public void getReadyToShoot(){
        shotConfig config = getForTargetY(robot.targetY);

        robot.intake.setShooterVelocity(config.rpm);
        robot.intake.setHoodAngle(config.hoodPos);
    }
}
