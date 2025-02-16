package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralArm extends SubsystemBase {
    // motors
    // coralpivotmotor
    // wheelmotor
    // SparkMaxConfig coralWristConfig;
    // SparkMaxConfig coralWheelConfig;
    SparkClosedLoopController coralWristController;

    // sensor coralSensor = new Sensor();

    private double coralWristSetpoint;

    // SparkMax coralWheel = new SparkMax(100, MotorType.kBrushless);
    // SparkMax coralWrist = new SparkMax(1001, MotorType.kBrushless);
    TalonSRX coralWheel = new TalonSRX(40);
    SparkMax coralWrist = new SparkMax(55, MotorType.kBrushed);

    private final PIDController coralWristPID = new PIDController(0.1, 0, 0);

    public CoralArm() {
        Shuffleboard.getTab("PID").add("Coral", coralWristPID);
        SparkMaxConfig configWrist = new SparkMaxConfig();
        configWrist
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configWrist.encoder.positionConversionFactor(1.0 / 100.0);
        configWrist.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(0.75, 0, 0)
                .outputRange(-0.45, 0.45);
        coralWrist.configure(configWrist, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        Shuffleboard.getTab("Debug").addDouble("CoralEncoder", () -> coralWrist.getAbsoluteEncoder().getPosition());

        // SparkMaxConfig configWheel = new SparkMaxConfig();
        // configWheel
        // .inverted(false)
        // .idleMode(SparkMaxConfig.IdleMode.kBrake);
        // configWheel.encoder.positionConversionFactor(1/10);
        // coralWheel.configure(configWheel, ResetMode.kResetSafeParameters,
        // PersistMode.kNoPersistParameters);
        // Constants.configPIDMotor(coralWrist, false, 0, 0,0);
        // Constants.configMotor(coralWheel, true);

        coralWristController = coralWrist.getClosedLoopController();
        coralWristController.setReference(0, ControlType.kMAXMotionPositionControl);
    }

    // @Override
    // public void periodic() {
    // coralWrist.set(coralWristPID.calculate(coralWrist.getSelectedSensorPosition()));
    // }

    public void intakeCoral() {
        coralWheel.set(TalonSRXControlMode.PercentOutput, 0.5);
    }

    public void releaseCoral() {
        coralWheel.set(TalonSRXControlMode.PercentOutput, -0.5);
    }

    public void stopCoralRoller() {
        coralWheel.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void stopCoralWrist() {
        coralWrist.set(0);
    }

    public void setCoralWristSetpoint(double setpoint) {
        coralWristController.setReference(setpoint, ControlType.kPosition);
    }

    public boolean hasCoral() {
        // true if has coral
        // false if not
        return true;
    }

    public Command l1() {
        return run(() -> setCoralWristSetpoint(Constants.SetpointConstants.CoralPivotAngles.l1));
    }

    // out
    public Command lmid() {
        return run(() -> setCoralWristSetpoint(Constants.SetpointConstants.CoralPivotAngles.lmid));
    }

    public Command l4() {
        return run(() -> setCoralWristSetpoint(Constants.SetpointConstants.CoralPivotAngles.l4));
    }

    public Command stopWrist() {
        return run(() -> setCoralWristSetpoint(0));
    }

    // public Command backwards() {
    // return run(() -> setCoralWristSetpoint(-1));
    // }
    public Command intakeCoralCommand() {
        return run(() -> intakeCoral());
    }

    public Command outtakeCoral() {
        return run(() -> releaseCoral());
    }

    public Command stopCoralSpin() {
        return run(() -> stopCoralRoller());
    }
}
