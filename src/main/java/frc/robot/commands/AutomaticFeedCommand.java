package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed.FeedSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;


public class AutomaticFeedCommand extends CommandBase {

    private final FeedSubsystem feedSubsystem;
    private final BooleanSupplier readyToLaunch;
    private State state;
    private double timeTillOneBallBottom;
    public AutomaticFeedCommand(FeedSubsystem feedSubsystem, BooleanSupplier readyToLaunch) {
        this.feedSubsystem = feedSubsystem;
        this.readyToLaunch = readyToLaunch;
        this.state = State.Empty;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.feedSubsystem);
    }

    @Override
    public void initialize() {
    }

    public void resetEmpty() {
        state = State.Empty;
    }

    @Override
    public void execute() {
        if (state == State.Empty) {
            if (feedSubsystem.getBottomSensor()) {
                feedSubsystem.spinFeed(-1);
                state = State.IntakingOneBall;
            } else {
                feedSubsystem.spinFeed(0);
            }
        } else if (state == State.IntakingOneBall) {
            if (feedSubsystem.getBottomSensor()) {
                feedSubsystem.spinFeed(-1);
            } else {
                feedSubsystem.spinFeed(0);
                state = State.OneBallBottom;
                timeTillOneBallBottom = Timer.getFPGATimestamp();
            }
        } else if (state == State.OneBallBottom) {
            if (feedSubsystem.getBottomSensor()) {
                feedSubsystem.spinFeed(-1);
                state = State.TwoBallsTraveling;
            } else if (Timer.getFPGATimestamp() - timeTillOneBallBottom < 0.1) {
                feedSubsystem.spinFeed(-1);
            } else {
                feedSubsystem.spinFeed(0);
            }
        } else if (state == State.TwoBallsTraveling) {
            if (feedSubsystem.getTopSensor()) {
                feedSubsystem.spinFeed(0);
                state = State.TwoBallsTop;
            } else {
                feedSubsystem.spinFeed(-1);
            }
        } else if (state == State.TwoBallsTop) {
            if (readyToLaunch.getAsBoolean()) {
                feedSubsystem.spinFeed(-1);
            } else {
                feedSubsystem.spinFeed(0);
            }
            if (!feedSubsystem.getTopSensor()) {
                state = State.OneBallTop;
            }
        } else if (state == State.OneBallTop) {
            if (readyToLaunch.getAsBoolean() || !feedSubsystem.getTopSensor()) {
                feedSubsystem.spinFeed(-1);
            } else {
                feedSubsystem.spinFeed(0);
            }
            if (feedSubsystem.getTopSensor()) {
                state = State.LaunchingFinalBall;
            }
        } else if (state == State.LaunchingFinalBall) {
            if (readyToLaunch.getAsBoolean()) {
                feedSubsystem.spinFeed(-1);
            } else {
                feedSubsystem.spinFeed(0);
            }
            if (!feedSubsystem.getTopSensor()) {
                state = State.Empty;
            }
        } else if (state == State.OneBallTraveling) {
            if (feedSubsystem.getTopSensor()) {
                feedSubsystem.spinFeed(0);
                state = State.OneBallTop;
            } else {
                feedSubsystem.spinFeed(-1);
            }
        }

        Logger.getInstance().recordOutput("Feed/state", state.name());
        Logger.getInstance().recordOutput("Feed/readyToLaunch", readyToLaunch.getAsBoolean()); 

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    

    }

    public State getStateOfBall() {
        return state;
    }

    public void moveToTop() {
        feedSubsystem.spinFeed(-1);
        state = State.OneBallTraveling;
    }

    public enum State {
        Empty, IntakingOneBall, OneBallBottom, TwoBallsTraveling, OneBallTraveling, TwoBallsTop, OneBallTop, LaunchingFinalBall
    }
}
