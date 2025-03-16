package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LLLeds {
    public static Command on(String limelight) {
        return Commands.runOnce(() -> {
            LimelightHelpers.setLEDMode_ForceOn(limelight);
        });
    }

    public static Command off(String limelight) {
        return Commands.runOnce(() -> {
            LimelightHelpers.setLEDMode_ForceOff(limelight);
        });
    }

    public static Command blink(String limelight) {
        return Commands.runOnce(() -> {
            LimelightHelpers.setLEDMode_ForceBlink(limelight);
        });
    }

    public static Command shortBlink(String limelight) {
        return on(limelight).withTimeout(1).andThen(off(limelight));
    }
}
