package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Rumble extends SubsystemBase {
    public enum Controller {
        DRIVER, OPERATOR
    }

    public static class RumbleRequest {
        protected final Controller controller;
        protected final boolean overrideCurrentRequest;
        protected final double lengthInSeconds;
        protected final double value;
        protected final double frequency;
        protected final long timestampStarted;

        public RumbleRequest(Controller controller, boolean overrideCurrentRequest, double lengthInSeconds, double value) {
            this.controller = controller;
            this.overrideCurrentRequest = overrideCurrentRequest;
            this.lengthInSeconds = lengthInSeconds;
            this.value = value;
            this.frequency = -1;

            this.timestampStarted = RobotController.getTime();
        }

        public RumbleRequest(Controller controller, boolean overrideCurrentRequest, double lengthInSeconds, double value, double frequency) {
            this.controller = controller;
            this.overrideCurrentRequest = overrideCurrentRequest;
            this.lengthInSeconds = lengthInSeconds;
            this.value = value;
            this.frequency = frequency;

            this.timestampStarted = RobotController.getTime();
        }
    }
    private static Rumble INSTANCE;

    private CommandXboxController driver;
    private CommandXboxController operator;

    private ArrayList<RumbleRequest> driverRequests = new ArrayList<>();
    private Optional<RumbleRequest> currentDriverRequest = Optional.empty();
    private static double currentDriverRumble = 0;

    private ArrayList<RumbleRequest> operatorRequests = new ArrayList<>();
    private Optional<RumbleRequest> currentOperatorRequest = Optional.empty();
    private static double currentOperatorRumble = 0;


    public Rumble(CommandXboxController driver, CommandXboxController operator) {
        this.driver = driver;
        this.operator = operator;

        INSTANCE = this;
    }


    public static Rumble getInstance() {
        return INSTANCE;
    }


    public void addRequest(RumbleRequest request) {
        if (request.controller == Controller.DRIVER) {
            if (currentDriverRequest.isPresent()) {
                driverRequests.add(request);
                return;
            }
            currentDriverRequest = Optional.of(request);

        } else {
            if (currentOperatorRequest.isPresent()) {
                operatorRequests.add(request);
                return;
            }
            currentOperatorRequest = Optional.of(request);
        }

    }


    private void setDriverRumble(double value) {
        currentDriverRumble = value;
        driver.setRumble(RumbleType.kBothRumble, value);
    }


    private void setOperatorRumble(double value) {
        currentOperatorRumble = value;
        operator.setRumble(RumbleType.kBothRumble, value);
    }


    private void handleDriverRequests() {
        // If the current request has timed out
        if(
            currentDriverRequest.isPresent() && 
            currentDriverRequest.get().timestampStarted + currentDriverRequest.get().lengthInSeconds * 1e+6 < RobotController.getTime()
        ) {
            currentDriverRequest = Optional.empty();
        }
        
        // If the current request is empty and there's no requests left
        if(currentDriverRequest.isEmpty() && driverRequests.size() == 0) {
            if(currentDriverRumble != 0.0) setDriverRumble(0);
            return;
        }

        // If the current request is empty, get the next one
        if(currentDriverRequest.isEmpty()) currentDriverRequest = Optional.of(driverRequests.remove(0));

        double value = currentDriverRequest.get().value;
        if (currentDriverRequest.get().frequency > 0) {
            long elapsed = RobotController.getTime() - currentDriverRequest.get().timestampStarted;
            double cyclePosition = (elapsed / 1e6 * currentDriverRequest.get().frequency) % 1.0;
            value = cyclePosition < 0.5 ? value : 0.0;
        }
        if (currentDriverRumble != value) setDriverRumble(value);

    }


    private void handleOperatorRequests() {
        // If the current request has timed out
        if(
            currentOperatorRequest.isPresent() && 
            currentOperatorRequest.get().timestampStarted + currentOperatorRequest.get().lengthInSeconds * 1e+6 < RobotController.getTime()
        ) {
            currentOperatorRequest = Optional.empty();
        }
        
        // If the current request is empty and there's no requests left
        if(currentOperatorRequest.isEmpty() && operatorRequests.size() == 0) {
            if(currentOperatorRumble != 0.0) setOperatorRumble(0.0);
            return;
        }

        // If the current request is empty, get the next one
        if(currentOperatorRequest.isEmpty()) currentOperatorRequest = Optional.of(operatorRequests.remove(0));

        double value = currentOperatorRequest.get().value;
        if (currentOperatorRequest.get().frequency > 0) {
            long elapsed = RobotController.getTime() - currentOperatorRequest.get().timestampStarted;
            double cyclePosition = (elapsed / 1e6 * currentOperatorRequest.get().frequency) % 1.0;
            value = cyclePosition < 0.5 ? value : 0.0;
        }
        if (currentOperatorRumble != value) setDriverRumble(value);
    }


    @Override
    public void periodic() {
        handleDriverRequests();
        handleOperatorRequests();
    }
}
