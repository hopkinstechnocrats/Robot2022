package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed.FeedSubsystem;

import java.util.function.BooleanSupplier;


public class AutomaticFeedCommand extends CommandBase {

    public enum State {
        Empty, IntakingOneBall, OneBallBottom, TwoBallsTraveling, TwoBallsTop, OneBallTop, LaunchingFinalBall
    }

    private final FeedSubsystem feedSubsystem;
    private final BooleanSupplier readyToLaunch;
    private State state;

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
            }
        } else if (state == State.OneBallBottom) {
            if (feedSubsystem.getBottomSensor()) {
                feedSubsystem.spinFeed(-1);
                state = State.TwoBallsTraveling;
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
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
