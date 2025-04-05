package frc.robot.subsystems;

import java.util.EnumSet;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSub extends SubsystemBase {
    // this is the climb system :0
    // idek man
    // guy waas here

    // Motor pushArm = new Motor();
    // SparkMax climbMotorOne = new SparkMax(102, MotorType.kBrushless);
    // SparkMax climbMotorTwo = new SparkMax(103, MotorType.kBrushless);

    double climbSetpoint1 = 0;
    double climbSetpoint2 = 0;
    SparkClosedLoopController climbController1;
    SparkClosedLoopController climbController2;
    private final SparkMax climb1 = new SparkMax(57, MotorType.kBrushless); // short arm
    // private final SparkMax climb2 = new SparkMax(57, MotorType.kBrushless);
    // //long arm
    TrapezoidProfile climbTrapezoidProfile1 = new TrapezoidProfile(new Constraints(.2, 40));
    TrapezoidProfile.State climbTrapezoidSetpoint1 = new TrapezoidProfile.State();
    // TrapezoidProfile climbTrapezoidProfile2 = new TrapezoidProfile(new
    // Constraints(.2, 40));
    // TrapezoidProfile.State climbTrapezoidSetpoint2 = new
    // TrapezoidProfile.State();

    boolean enabled = false;

    // either have the non pid motor follow the pid one or have 2 pid controllers
    // idk

    // setpoint = Constants.SetpointConstants.coralSetpointArray[index]
    public ClimbSub() {
        // Constants.configPIDMotor(climbMotorOne,false, 0,0,0);

        SparkMaxConfig configLeader = new SparkMaxConfig();
        configLeader
                .openLoopRampRate(.5)
                .inverted(true)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configLeader.closedLoop
                .pid(5, 0, 0)
                .outputRange(0, 1);
        configLeader.smartCurrentLimit(100, 100);
        configLeader.secondaryCurrentLimit(100);
        configLeader.encoder.positionConversionFactor(1.0 / 135.0);
        climb1.configure(configLeader, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // SparkMaxConfig config2 = new SparkMaxConfig();
        // config2
        // .openLoopRampRate(.5)
        // .inverted(false)
        // .idleMode(SparkMaxConfig.IdleMode.kCoast);
        // config2.closedLoop
        // .pid(5, 0, 0)
        // .outputRange(0, 1);
        // config2.encoder.positionConversionFactor(1.0 / 135.0);
        // config2.smartCurrentLimit(100, 100);
        // config2.secondaryCurrentLimit(100);
        // // config2.follow(climbMotorOne);
        // climb2.configure(config2, ResetMode.kResetSafeParameters,
        // PersistMode.kNoPersistParameters);

        // climbController1 = climb1.getClosedLoopController();
        // climbController2 = climb2.getClosedLoopController();

        // climbMotorTwo.setControl(new Follower(climbMotorOne.getDeviceId(),true));
        // climbController1.setReference(0, ControlType.kPosition);
        // climbController2.setReference(0, ControlType.kPosition);

        // Shuffleboard.getTab("Debug").addDouble("Climb 1 Position", () ->
        // climb1.getEncoder().getPosition());
        // Shuffleboard.getTab("Debug").addDouble("Climb 2 Position", () ->
        // climb2.getEncoder().getPosition());
        Shuffleboard.getTab("Debug").add("Rezero climb encoders", Commands.runOnce(() -> {
            climb1.getEncoder().setPosition(0);
            // climb2.getEncoder().setPosition(0);
            climbSetpoint1 = 0;
            // climbSetpoint2 = 0;
            climbTrapezoidSetpoint1 = new TrapezoidProfile.State();
            // climb1.getEncoder().setPosition(0);
            // climbTrapezoidSetpoint2 = new TrapezoidProfile.State();
        }));
    }

    public void periodic() {
        // climbTrapezoidSetpoint1 = climbTrapezoidProfile1.calculate(0.02,
        // climbTrapezoidSetpoint1,
        // new TrapezoidProfile.State(climbSetpoint1, 0));
        // climbTrapezoidSetpoint2 = climbTrapezoidProfile2.calculate(0.02,
        // climbTrapezoidSetpoint2,
        // new TrapezoidProfile.State(climbSetpoint2, 0));
        // // System.out.println("Setpoint " + setpoint + " goal "+ trapezoidSetpoint);
        // NetworkTableInstance.getDefault().getEntry("/Tune/Climb/Goal1").setDouble(climbTrapezoidSetpoint1.position);
        // NetworkTableInstance.getDefault().getEntry("/Tune/Climb/Goal2").setDouble(climbTrapezoidSetpoint2.position);
        // NetworkTableInstance.getDefault().getEntry("/Tune/Climb/Setpoint1").setDouble(climbSetpoint1);
        // NetworkTableInstance.getDefault().getEntry("/Tune/Climb/Setpoint2").setDouble(climbSetpoint2);
        // NetworkTableInstance.getDefault().getEntry("/Tune/Climb/Pos1").setDouble(climb1.getEncoder().getPosition());
        // NetworkTableInstance.getDefault().getEntry("/Tune/Climb/Pos2").setDouble(climb2.getEncoder().getPosition());
        // NetworkTableInstance.getDefault().getEntry("/Tune/Climb/Applied1").setDouble(climb1.getAppliedOutput());
        // NetworkTableInstance.getDefault().getEntry("/Tune/Climb/Applied2").setDouble(climb2.getAppliedOutput());
        // if (!enabled) {
        // climbController1.setReference(0, ControlType.kDutyCycle);
        // climbController2.setReference(0, ControlType.kDutyCycle);
        // return;
        // }
        // climbController1.setReference(climbTrapezoidSetpoint1.position,
        // ControlType.kPosition);
        // climbController2.setReference(climbTrapezoidSetpoint2.position,
        // ControlType.kPosition);

        // var entryp1 = NetworkTableInstance.getDefault().getEntry("/Tune/ClimbPID/P");
        // entry.setDouble(f.getDouble(thisClass));
        // NetworkTableInstance.getDefault().addListener(entry,
        // EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
        // e -> {
        // try {
        // f.set(thisClass, e.valueData.value.getDouble());
        // } catch (IllegalArgumentException | IllegalAccessException e1) {
        // e1.printStackTrace();
        // }
        // });

        DogLog.log("climb encoder", climb1.getEncoder().getPosition());

    }

    public void climbLetGo(boolean overridden) {
        // set climb motors to down
        if (overridden) {
            climb1.set(-1);
        } else {
            if (climb1.getEncoder().getPosition() <= -4.9) {
                climb1.set(0);
            } else {
                climb1.set(-0.95);

            }
        }
        // climb2.set(0.9);
    }

    public void climbDown(boolean overridden, boolean overoverridden) {
        if (!overridden && !overoverridden) {
            if (climb1.getEncoder().getPosition() >= 0.0) {
                climb1.set(0);
            } else {
                climb1.set(0.9);
            }
        } else if (overridden && !overoverridden) {
            if (climb1.getEncoder().getPosition() >= 0.4) {
                climb1.set(0);
            } else {
                climb1.set(0.9);
            }
        } else if (!overridden && overoverridden) {
            climb1.set(0.85);
        }
    }

    public void climbDownSlow() {
        // set climb motors to down
        climb1.set(0.333);
        // climb2.set(0.333);
    }
    // public void climbUp() {
    // // set climb motors up??
    // climb1.set(-0.8);
    // climb2.set(-0.8);
    // }

    public void climbStop() {
        climb1.set(0);
        // climb2.set(0);
        // enabled = false;
        // climbSetpoint1 = 0;
        // climbSetpoint2 = 0;
        // climbController1.setReference(0, ControlType.kDutyCycle);
        // climbController2.setReference(0, ControlType.kDutyCycle);
    }

    private void setClimbSetpoint1(double setpoint) {
        climbSetpoint1 = setpoint;
    }

    private void setClimbSetpoint2(double setpoint) {
        climbSetpoint2 = setpoint;
    }
    // i wonder still
    // we could have it push down until it reaches a setpoint
    // or have the pull up initiated as the driver presses a button then have it
    // freeze in place when not being moved
    // that way the drivers can tell it when to stop

    // public Command climb(){
    // return run(() -> {
    // enabled = true;
    // setClimbSetpoint1(0.4);
    // setClimbSetpoint2(0.4);
    // });
    // }

    public Command climb() {
        return run(() -> climbDown(false, false));
    }

    public Command climbOverridden() {
        return run(() -> climbDown(true, false));
    }

    public Command climbRelease() {
        return run(() -> climbLetGo(false));
    }

    public Command climbReleaseOverridden() {
        return run(() -> climbLetGo(true));
    }

    public Command climbOverOverridden() {
        return run(() -> climbDown(false, true));
    }

    public Command climbSlow() {
        return run(() -> climbDownSlow());
    }

    public Command climbStopManual() {
        return run(() -> climbStop());
    }

    public Command climbStopCommand() {
        return run(() -> {
            setClimbSetpoint1(0);
            setClimbSetpoint2(0);
        });
    }

    public void climbWithSpeed(double rightTriggerAxis) {
        climb1.set(rightTriggerAxis);
        // climb2.set(rightTriggerAxis);
    }
}