package lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DelayedSequentialCommandGroup extends SequentialCommandGroup {

    public DelayedSequentialCommandGroup(JoystickButton button, Command... commands) {
        Command[] newCommands = new Command[commands.length];
        for (int i = 0; i < commands.length; i++) {
            newCommands[i] = (new SequentialCommandGroup(commands[i], new WaitCommand(button)));
        }
        addCommands(newCommands);
    }

}