package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.souffle.Souffle;

public class AlgaeArm extends SubsystemBase {
    // motors
    SparkMax algaeWrist = new SparkMax(56, MotorType.kBrushless);
    SparkMax algaeSpinner = new SparkMax(41, MotorType.kBrushless);
    // SparkMax algaeSpinMotor = new SparkMax(0, MotorType.kBrushless);
    TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(30, 10));
    private SparkClosedLoopController algaeWristController;
    private double algaeWristSetpoint = 0;
    SparkLimitSwitch algaeLimitSwitch = algaeSpinner.getForwardLimitSwitch();
    // pincher1
    // pivotmotor
    // DigitalInput topLimitSwitch = new DigitalInput(0);
    // DigitalInput bottomLimitSwitch = new DigitalInput(1);
    // private trapezoidSetpoint;
    TrapezoidProfile.State trapezoidSetpoint = new TrapezoidProfile.State();

    public AlgaeArm() {

        SparkMaxConfig configAlgaeWrist = new SparkMaxConfig();
        configAlgaeWrist
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kCoast);
        configAlgaeWrist.absoluteEncoder
                .inverted(true)
                .zeroCentered(true);
        // .zeroOffset(0.72);
        configAlgaeWrist.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs(100);
        configAlgaeWrist.encoder
                .positionConversionFactor(1.0 / 125.0)
                .velocityConversionFactor((1.0 / 125.0) / 60.0);
        configAlgaeWrist.closedLoop
                .pid(4.2, 0, 0)
                .outputRange(-0.6, 0.6);
        algaeWrist.configure(configAlgaeWrist, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configAlgaeSpinMotor = new SparkMaxConfig();
        configAlgaeSpinMotor
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
        algaeSpinner.configure(configAlgaeSpinMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        algaeWrist.getEncoder().setPosition(algaeWrist.getAbsoluteEncoder().getPosition());

        // Constants.configPIDMotor(algaeWrist,true, 0,0,0);
        // Constants.configMotor(algaeSpinMotor, false);

        // SparkMaxConfig configAlgaeSpin = new SparkMaxConfig();
        // configAlgaeSpin
        // .inverted(false)
        // .idleMode(SparkMaxConfig.IdleMode.kBrake);
        // configAlgaeSpin.encoder.positionConversionFactor(1/10);
        // algaeSpinMotor.configure(configAlgaeSpin, ResetMode.kResetSafeParameters,
        // PersistMode.kNoPersistParameters);

        algaeWristController = algaeWrist.getClosedLoopController();
        algaeWristController.setReference(0, ControlType.kPosition);

        // algaeWrist.getEncoder().setPosition(0);

        // Shuffleboard.getTab("Debug").add("P", 0.0);
        Souffle.record("Debug/Algae Wrist Setpoint", () -> algaeWristSetpoint);
        Souffle.record("Debug/Algae Wrist Current", () -> algaeWrist.getEncoder().getPosition());
        Souffle.record("Debug/Algae Wrist Absolute",
                () -> algaeWrist.getAbsoluteEncoder().getPosition());
        Souffle.record("Debug/Algae Wrist Power", () -> algaeWrist.getAppliedOutput());
        Souffle.record("Debug/has Algae", () -> hasAlgae());

    }

    // ok plan a
    // methods
    // grab()
    // pincher1.set(0.5)
    // pincher2.set(0.5)
    public void grab() {
        algaeSpinner.set(0.9);
    }

    // // grab for amount of time or until we have an algae
    // // thatll be in commands though

    public void release() {
        algaeSpinner.set(-0.9);
    }

    // //
    public void stop() {
        algaeSpinner.set(0);
    }

    public void stopWrist() {
        algaeWrist.set(0);
    }

    public void setAlgaeSetpoint(double setpoint) {
        // algaeWristController.setReference(setpoint, ControlType.kPosition);
        algaeWristSetpoint = setpoint;
    }

    public boolean isReady() {
        double algaePosition = algaeWrist.getEncoder().getPosition();
        return algaePosition > (algaeWristSetpoint - 0.1) && algaePosition < (algaeWristSetpoint + 0.1);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            algaeWristSetpoint = algaeWrist.getEncoder().getPosition();
            trapezoidSetpoint = new TrapezoidProfile.State(
                    algaeWrist.getEncoder().getPosition(),
                    algaeWrist.getEncoder().getVelocity());
            algaeWristController.setReference(trapezoidSetpoint.position, ControlType.kPosition);
        }

        trapezoidSetpoint = trapezoidProfile.calculate(0.02,
                trapezoidSetpoint,
                new TrapezoidProfile.State(algaeWristSetpoint, 0));
        // System.out.println("Setpoint " + setpoint + " goal "+ trapezoidSetpoint);
        NetworkTableInstance.getDefault().getEntry("/Shuffleboard/Debug/AlgaeWristGoal")
                .setDouble(trapezoidSetpoint.position);
        algaeWristController.setReference(trapezoidSetpoint.position, ControlType.kPosition);
    }

    // public boolean hasAlgae() {
    // // if sensor detects algae
    // // return true
    // // if sensor doesnt detect algae
    // // return false
    // if (topLimitSwitch.get()) {
    // return true;
    // } else {
    // return false;
    // }

    // }

    public boolean hasAlgae() {
        return algaeLimitSwitch.isPressed();
    }

    public boolean hasNoAlgae() {
        return !algaeLimitSwitch.isPressed();
    }

    public Command algaeArmDown() {
        return runOnce(() -> setAlgaeSetpoint(Constants.SetpointConstants.AlgaeArmAngles.down.get()));
    }

    public Command algaeArmUp() {
        return runOnce(() -> setAlgaeSetpoint(Constants.SetpointConstants.AlgaeArmAngles.up.get()));
    }

    public Command algaeArmStop() {
        return runOnce(() -> stopWrist());
    }

    public class AlgaeSpinCommandTest extends Command {
        private AlgaeArm algaeArm;

        public AlgaeSpinCommandTest(AlgaeArm algaeArm) {
            this.algaeArm = algaeArm;
            addRequirements(algaeArm);
        }

        @Override
        public void initialize() {
            System.out.println("Inside spin");
            algaeArm.grab();
        }

        @Override
        public void end(boolean interrupted) {
            System.out.println("end spin");
            algaeArm.stop();
        }

        @Override
        public boolean isFinished() {
            boolean v = algaeArm.hasAlgae();
            System.out.println("has algae " + v);
            return v;
        }
    }

    public Command algaeSpinIn() {
        return run(() -> grab());
        // return Commands.print("Inside spin").andThen(Commands.startEnd(() -> grab(),
        // () -> stop(), this)).until(this::hasAlgae).finallyDo((f) ->
        // System.err.println("algae spin stop " + f));
    }

    public Command algaeSpinOut() {
        // return runOnce(this::release).until(this::hasNoAlgae).finallyDo(this::stop);
        return run(() -> release());
    }

    public Command algaeSpinStop() {
        return run(() -> stop());
    }

    // public Command algaeArmStop(){
    // return run(() -> stop());
    // }

    // release()
    // pincher2.set(-0.5)
    // pincher1.set(-0.5)
    // stop()
    // pincher1.set(0)
    // pincher2.set(0)

}
