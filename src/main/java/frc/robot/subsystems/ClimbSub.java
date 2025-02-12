package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSub extends SubsystemBase {
    // this is the climb system :0
    // idek man
    // guy waas here

    // Motor pushArm = new Motor();
    //SparkMax climbMotorOne = new SparkMax(102, MotorType.kBrushless);
    //SparkMax climbMotorTwo = new SparkMax(103, MotorType.kBrushless);

    double climbSetpoint;
    SparkClosedLoopController climbController;

    // either have the non pid motor follow the pid one or have 2 pid controllers
    // idk

    // setpoint = Constants.SetpointConstants.coralSetpointArray[index]
    public ClimbSub() {
        // Constants.configPIDMotor(climbMotorOne,false, 0,0,0);

        SparkMaxConfig configLeader = new SparkMaxConfig();
        configLeader
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configLeader.smartCurrentLimit(30, 30);
        configLeader.secondaryCurrentLimit(35);
        configLeader.encoder.positionConversionFactor(1/60);
        configLeader.closedLoop
                .pid(0, 0, 0)
                .outputRange(-1, 1);
        //climbMotorOne.configure(configLeader, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configFollower = new SparkMaxConfig();
        configFollower
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configFollower.encoder.positionConversionFactor(1/60);
        configFollower.smartCurrentLimit(30, 30);
        configFollower.secondaryCurrentLimit(35);
        //configFollower.follow(climbMotorOne);
        //climbMotorTwo.configure(configFollower, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        //climbController = climbMotorOne.getClosedLoopController();

        // climbMotorTwo.setControl(new Follower(climbMotorOne.getDeviceId(),true));
        //climbController.setReference(0, ControlType.kPosition);
    }

    public void pushUp() {
        // set climb motors to down
        climbSetpoint = 0;
    }

    public void lowerBot() {
        // set climb motors up?
        climbSetpoint = 0;
    }
    // i wonder still
    // we could have it push down until it reaches a setpoint
    // or have the pull up initiated as the driver presses a button then have it
    // freeze in place when not being moved
    // that way the drivers can tell it when to stop
}