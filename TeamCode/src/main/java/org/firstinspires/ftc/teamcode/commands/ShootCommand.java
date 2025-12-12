package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Beltway;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class ShootCommand extends CommandBase {

    private Beltway beltway;
    private Intake intake;

    private int runtimeInMilliseconds;


    private ElapsedTime timer = new ElapsedTime();

    public ShootCommand(Beltway beltway, Intake intake, int runtimeInMilliseconds) {
        this.beltway = beltway;
        this.intake = intake;
        this.runtimeInMilliseconds = runtimeInMilliseconds;
        addRequirements(beltway, intake);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        beltway.forward();
        intake.slowRoll();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= runtimeInMilliseconds;
    }



    @Override
    public void end(boolean interrupted) {
        beltway.stop();
        intake.stop();
    }
}
