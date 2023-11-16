package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class driveConstants {
    public static class swerveConstants {
        public static final int driveLeftFront = 8;
        public static final int driveRightFront = 7;
        public static final int driveLeftBack = 10;
        public static final int driveRightBack = 9;

        public static final int steerLeftFront = 20;
        public static final int steerRightFront = 17;
        public static final int steerLeftBack = 18;
        public static final int steerRightBack = 19;

        public static final int encoderLeftFront = 4;
        public static final int encoderRightFront = 6;
        public static final int encoderLeftBack = 5;
        public static final int encoderRightBack = 3;

        public static final double thetaOffsetLeftFront = 126.47469 + 180;
        public static final double thetaOffsetRightFront = 55.195313;
        public static final double thetaOffsetLeftBack = 72.333984 + 90;
        public static final double thetaOffsetRightBack = 248.642578 + 90;

        // public static final double thetaOffsetLeftFront = -161.806621125;
        // public static final double thetaOffsetRightFront = -51.943359375;
        // public static final double thetaOffsetLeftBack =-162.158203125;
        // public static final double thetaOffsetRightBack = 21.44532012939453;
      

        private static final double XOffset = Units.inchesToMeters(26.25);
        private static final double YOffset = Units.inchesToMeters(26.25);

       
        public static final Translation2d xyOffsetLeftFront = new Translation2d(-XOffset / 2, -YOffset / 2);
        public static final Translation2d xyOffsetRightFront = new Translation2d(-XOffset / 2, YOffset / 2);
        public static final Translation2d xyOffsetLeftBack = new Translation2d(XOffset / 2, -YOffset / 2); 
        public static final Translation2d xyOffsetRightBack = new Translation2d(XOffset / 2,YOffset / 2);


        public static final double driveGearRatio = 8.41;
        public static final double steerGearRatio = 21.4285714286;
        
        public static final double wheelDiameter = 4.0;

    }
}
