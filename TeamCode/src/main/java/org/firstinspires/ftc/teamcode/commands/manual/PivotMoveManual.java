package org.firstinspires.ftc.teamcode.commands.manual;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;

import java.util.function.Supplier;

public class PivotMoveManual extends CommandBase {
    private final Pivot pivot;
    private final Supplier<Double> doubleSupplier;
    public PivotMoveManual(Pivot pivot, Supplier<Double> doubleSupplier) {
        this.pivot = pivot;
        this.doubleSupplier = doubleSupplier;
        addRequirements(pivot);
    }
    @Override
    public void execute() {
        double position = doubleSupplier.get();
        if (Math.abs(position) > 0.1) {
            pivot.setSetPoint(pivot.getSetPoint() + position * 19, false);
        }
    }
}
