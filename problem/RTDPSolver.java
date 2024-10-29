package problem;

import simulator.Simulator;
import simulator.State;
import simulator.Step;
import java.util.Map;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.HashSet;

public class RTDPSolver {
    private ProblemSpec problemSpec;
    private Simulator simulator;
    private double discountFactor; // discount factor
    private State initialState; // initial state
    private Map<State, Double> stateValues; // state values
    private Set<State> solvedStates = new HashSet<>();// solved states

    public RTDPSolver(ProblemSpec spec, Simulator simulator, State initialState) {
        this.problemSpec = spec;
        this.simulator = simulator;
        this.discountFactor = problemSpec.getDiscountFactor();
        this.initialState = initialState;
        this.stateValues = new HashMap<>();
        stateValues.put(initialState, 0.0); // initialize the value of the initial state, V(s0) = 0
    }


    /**
     * Run the labeled RTDP algorithm
     */
    public void runLabeledRTDP() {
        State currentState = initialState; // initialize the current state
        while (!simulator.isGoalState(currentState) ) {  // while the current state is not the goal state

            // if current state is labeled as solved, break the loop
            if (solvedStates.contains(currentState)) {
                // if the current state is solved, break the loop
                break;
            }
            // greedy choose the best action
            Action bestAction = getGreedyAction(currentState);
            // System.out.println("RTDPSolver---runRTDP: best action: " + bestAction.getActionType());
            
            // perform the best action and get the next state
            State nextState = simulator.step(bestAction);

            // update the value of the current state
            updateValue(currentState, bestAction);

            // check if the current state is solved, if so, add it to the solved states
            if (isSolved(currentState)) {
                solvedStates.add(currentState);  // add the current state to the solved states
            }

            // update the current state
            currentState = nextState;
        }
    }


        /**
     * Run the normal RTDP algorithm
     */
    public void runRTDP() {
        State currentState = initialState; // initialize the current state
        while (!simulator.isGoalState(currentState) ) {  // while the current state is not the goal state

            // greedy choose the best action
            Action bestAction = getGreedyAction(currentState);
            // System.out.println("RTDPSolver---runRTDP: best action: " + bestAction.getActionType());
            
            // perform the best action and get the next state
            State nextState = simulator.step(bestAction);

            // update the value of the current state
            updateValue(currentState, bestAction);

            // update the current state
            currentState = nextState;
        }
    }

    /**
     * Check if the state is solved
     * @param state
     * @return
     */
    private boolean isSolved(State state) {
        // get the old value and the new value of the current state
        double oldValue = stateValues.getOrDefault(state, 0.0);
        double newValue = computeActionValue(state, getGreedyAction(state));
        
        // judge if the value change is less than the threshold epsilon
        // if it is less than the threshold epsilon, return true
        // otherwise, return false
        double epsilon = 0.01;  // threshold epsilon
        return Math.abs(newValue - oldValue) < epsilon;
    }
    

    /**
     * Get the greedy action, which is the action with the maximum expected value
     * 
     * @param state
     * @return
     */
    private Action getGreedyAction(State state) {
        double maxValue = Double.NEGATIVE_INFINITY;
        Action bestAction = null;

        List<ActionType> actionTypes = problemSpec.getLevel().getAvailableActions();
        // System.out.println("RTDPSolver---getGreedyAction: fqq actionTypes size: " + actionTypes.size());
        double expectedValue = 0.0;
        Action action = null;

        // tour all the available actions, performA1 to performA8
        // get the expected value of each action, and choose the action with the maximum expected value
        for (ActionType actionType: actionTypes) {

            // calculate Q(s, a) = R(s, a) + γ ∑ P(s'|s,a) * V(s')
            // System.out.println("RTDPSolver---getGreedyAction: fqq current action: " + actionType);
            if (actionType == ActionType.MOVE ) {
                if (!simulator.isValidFuelToMove(state.getFuel())){
                    expectedValue = Double.NEGATIVE_INFINITY;
                    action = new Action(actionType);
                    continue;
                }
                // System.out.println("RTDPSolver---getGreedyAction: fqq actionType: " + actionType);
                // calculate the expected value of the action, A1
                action = new Action(actionType);
                expectedValue = computeActionValue(state, action);

            } else if (actionType == ActionType.CHANGE_CAR) {
                // calculate the expected value of the action, A2
                // with different car types
                for (String carType : problemSpec.getCarOrder()) {
                    // skip the current car type
                    if (carType.equals(state.getCarType())) {
                        continue;
                    }
                    // System.out.println("RTDPSolver---getGreedyAction: fqq actionType: " + actionType + ", carType: " + carType);
                    action = new Action(actionType, carType);
                    expectedValue = computeActionValue(state, action);
                    // choose the action with the maximum expected value
                    if (expectedValue > maxValue) {
                        maxValue = expectedValue;
                        bestAction = action;
                    }
                }

            } else if (actionType == ActionType.CHANGE_DRIVER) {
                // calculate the expected value of the action, A3
                // with different drivers
                for (String driver : problemSpec.getDriverOrder()) {
                    // skip the current driver
                    if (driver.equals(state.getDriver())) {
                        continue;
                    }
                    // System.out.println("RTDPSolver---getGreedyAction: fqq actionType: " + actionType + ", driver: " + driver);
                    action = new Action(actionType, driver);
                    expectedValue = computeActionValue(state, action);
                    // choose the action with the maximum expected value
                    if (expectedValue > maxValue) {
                        maxValue = expectedValue;
                        bestAction = action;
                    }
                }

            } else if (actionType == ActionType.CHANGE_TIRES) {
                // System.out.println("RTDPSolver---getGreedyAction: fqq actionType: " + actionType + ", tire: " + problemSpec.getTireOrder());
                // calculate the expected value of the action, A4
                // with different tire models
                for (Tire tire : problemSpec.getTireOrder()) {
                    // skip the current tire model
                    if (tire.equals(state.getTireModel())) {
                        continue;
                    }
                    // System.out.println("RTDPSolver---getGreedyAction: fqq actionType: " + actionType + ", tire: " + tire);
                    action = new Action(actionType, tire);
                    expectedValue = computeActionValue(state, action);
                    // choose the action with the maximum expected value
                    if (expectedValue > maxValue) {
                        maxValue = expectedValue;
                        bestAction = action;
                    }
                }
            } else if (actionType == ActionType.ADD_FUEL) {
                // System.out.println("RTDPSolver---getGreedyAction: fqq actionType: " + actionType);
                // calculate the expected value of the action, A5
                // with different fuel amounts which not exceed the maximum fuel, , 10, 20, 30, 40, 50
                int currentFuel = state.getFuel();
                int addFuel = 10;
                while (currentFuel + addFuel <= ProblemSpec.FUEL_MAX) {
                    // System.out.println("RTDPSolver---getGreedyAction: fqq actionType: " + actionType + ", addFuel: " + addFuel);
                    action = new Action(actionType, addFuel);
                    expectedValue = computeActionValue(state, action);
                    // choose the action with the maximum expected value
                    if (expectedValue > maxValue) {
                        maxValue = expectedValue;
                        bestAction = action;
                    }
                    addFuel += 10;
                }

            } else if (actionType == ActionType.CHANGE_PRESSURE) {
                // System.out.println("RTDPSolver---getGreedyAction: fqq actionType: " + actionType);
                // calculate the expected value of the action, A6
                // with different tire pressures
                for (TirePressure tirePressure : TirePressure.values()) {
                    // skip the current tire pressure
                    if (tirePressure.equals(state.getTirePressure())) {
                        continue;
                    }
                    // System.out.println("RTDPSolver---getGreedyAction: fqq actionType: " + actionType + ", tirePressure: " + tirePressure);
                    action = new Action(actionType, tirePressure);
                    expectedValue = computeActionValue(state, action);
                    // choose the action with the maximum expected value
                    if (expectedValue > maxValue) {
                        maxValue = expectedValue;
                        bestAction = action;
                    }
                }
            } else if (actionType == ActionType.CHANGE_CAR_AND_DRIVER) {
                // System.out.println("RTDPSolver---getGreedyAction: fqq performA7: CHANGE_CAR_AND_DRIVER,  todo");
                // calculate the expected value of the action, A7
                // with different car types and drivers
                for (String carType : problemSpec.getCarOrder()) {
                    for (String driver : problemSpec.getDriverOrder()) {
                        // skip the current car type and driver
                        if (carType.equals(state.getCarType()) && driver.equals(state.getDriver())) {
                            continue;
                        }
                        // System.out.println("RTDPSolver---getGreedyAction: fqq actionType: " + actionType + ", carType: " + carType + ", driver: " + driver);
                        action = new Action(actionType, carType, driver);
                        expectedValue = computeActionValue(state, action);
                        // choose the action with the maximum expected value
                        if (expectedValue > maxValue) {
                            maxValue = expectedValue;
                            bestAction = action;
                        }
                    }
                }
            } else if (actionType == ActionType.CHANGE_TIRE_FUEL_PRESSURE) {
                // System.out.println("RTDPSolver---getGreedyAction: fqq performA8: CHANGE_CAR_AND_DRIVER,  todo");
            }
            
            // choose the action with the maximum expected value
            if (expectedValue > maxValue) {
                maxValue = expectedValue;
                bestAction = action;
            }
        }
        // System.out.println("RTDPSolver---getGreedyAction: greedy action: " + bestAction.getActionType());

        return bestAction;  // return the action with the maximum expected value
    }

    private double computeActionValue(State currentState, Action action) {
        // System.out.println("================= compute action value ==================");
        // System.out.println("RTDPSolver---computeActionValue: current state: " + currentState.toString());
        // System.out.println("RTDPSolver---computeActionValue: current action: " + action.getText());
        // get the reward of the current state and action, R(s, a)
        double reward = simulator.getReward(currentState, action);  
        // System.out.println("RTDPSolver---computeActionValue: reward: " + reward);
    
        // copy the simulator and future value
        // System.out.println("RTDPSolver---computeActionValue: copy simulator and future value");
        Simulator simulatedSimulator = simulator.copySimulator();
        // perform the action in the simulator and get the next state
        State nextState = simulatedSimulator.step(action);
        if (simulatedSimulator.isGoalState(nextState) ){
            // System.out.println("===================================");
            return 100;// if the next state is the goal state, return 100
        }
        // if the next state is null, return negative infinity
        if (nextState == null) {
            // System.out.println("===================================");
            return Double.NEGATIVE_INFINITY;
        }
        // System.out.println("RTDPSolver---computeActionValue: simulator after step: " + nextState.toString());
        // print simulator after step
        // System.out.println("RTDPSolver---computeActionValue: simulator after step: " + simulatedSimulator.getCurrentState().toString());
        // get the future value of the next state, ∑ P(s'|s,a) * V(s')
        double futureValue = simulatedSimulator.getFutureValue(nextState, action);  
        // System.out.println("RTDPSolver---computeActionValue: futureValue: " + futureValue);

        // System.out.println("RTDPSolver---computeActionValue: action value: " + (reward + discountFactor * futureValue) + ", current action: " + action.getText());
        // System.out.println("===================================");
        return reward + discountFactor * futureValue;  // Q(s, a) = R(s, a) + γ ∑ P(s'|s,a) * V(s')
    }

    // update the value of the current state using Bellman equation
    private void updateValue(State currentState, Action action) {
        // System.out.println("RTDPSolver---updateValue: update value: " + currentState.toString());
        double value = computeActionValue(currentState, action);
        stateValues.put(currentState, value);
    }

}
