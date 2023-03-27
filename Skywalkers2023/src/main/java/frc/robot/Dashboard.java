package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Dashboard {
    public static final class Swerve {
        public static final class Debugging extends BaseDashboard {}
        public static final class Driver extends BaseDashboard {}
    }
    public static final class Arm {
        public static final class Debugging extends BaseDashboard {}
        public static final class Driver extends BaseDashboard {}
    }
    public static final class Elevator {
        public static final class Debugging extends BaseDashboard {}
        public static final class Driver extends BaseDashboard {}
    }
    public static final class Intake {
        public static final class Debugging extends BaseDashboard {}
        public static final class Driver extends BaseDashboard {}
    }
    public static final class Auto {
        public static final class Debugging extends BaseDashboard {}
        public static final class Driver extends BaseDashboard {}
    }
    public static final class Tele {
        public static final class Debugging extends BaseDashboard {}
        public static final class Driver extends BaseDashboard {}
    }
    public static final class Limelight {
        public static final class Debugging extends BaseDashboard {}
        public static final class Driver extends BaseDashboard {}
    }
}

class BaseDashboard {
    static boolean run;

    static public void set(boolean value) {
        run = value;
    }

    static public void putNumber(String key, double value) {
        if(run == true) {
            SmartDashboard.putNumber(key, value);
        }
    }

    static public void putNumberArray(String key, double[] value) {
        if(run == true) {
            SmartDashboard.putNumberArray(key, value);
        }
    }

    public static void putString(String key, String value) {
        if(run == true) {
            SmartDashboard.putString(key, value);
        }
    }

    public void putStringArray(String key, String[] value) {
        if(run == true) {
            SmartDashboard.putStringArray(key, value);
        }
    }

    public static void putBoolean(String key, boolean value) {
        if(run == true) {
            SmartDashboard.putBoolean(key, value);
        }
    }

    public void putBooleanArray(String key, boolean[] value) {
        if(run == true) {
            SmartDashboard.putBooleanArray(key, value);
        }
    }
}