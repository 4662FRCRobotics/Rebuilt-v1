package frc.robot.libraries;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj2.command.button.*;

/** Add your docs here. */
public class ConsoleAuto extends GenericHID {

    private static final int kROV_SW_0 = 0;
    private static final int kROV_SW_1 = 1;

    public ConsoleAuto(final int port) {
        super(port);
    }

    public int getROT_SW_0() {
        return this.getPOV(kROV_SW_0) / 45;
    }

    public int getROT_SW_1() {
        return this.getPOV(kROV_SW_1) / 45;
    }

    public boolean getButton(int button) {
        return this.getRawButton(button);
    }

    public BooleanSupplier getSwitchSupplier(int button) {
        return () -> this.getRawButton(button);
    }

}