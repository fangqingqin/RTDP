package simulator;

import problem.*;

import java.io.*;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * This class is the simulator for the problem.
 * The simulator takes in an action and returns the next state.
 */
public class Simulator {

    /** Problem spec for the current problem **/
    private ProblemSpec ps;
    /** The current state of the environment **/
    private State currentState;
    /** The number of steps taken **/
    private int steps;
    /** Whether to print progress messages or not
     * Feel free to change this if you don't want messages printed **/
    private boolean verbose = true;
    /** A container to store steps for output **/
    private List<Step> stepRecord;
    /** path and name for output file **/
    private String outputFile;
    /** timer **/
    private long startTime;
    private long lastStepTime;

    /**
     * Construct a new simulator instance from the given problem spec
     *
     * @param ps the ProblemSpec
     * @param outputFile the path for output file
     */
    public Simulator(ProblemSpec ps, String outputFile) {
        System.out.println("Simulator: THIS IS THE OFFICIAL SUPPORT CODE");
        this.ps = ps;
        this.outputFile = outputFile;
        reset();
    }

    /**
     * Construct a new simulator instance from the given input file
     *
     * @param inputFile path to input file
     * @param outputFile the path for output file
     * @throws IOException if can't find file or there is a format error
     */
    public Simulator(String inputFile, String outputFile) throws IOException {
        this(new ProblemSpec(inputFile), outputFile);
    }

    /**
     * Reset the simulator and return initial state
     *
     * @return the start state
     */
    public State reset() {
        steps = 0;
        currentState = State.getStartState(ps.getFirstCarType(),
                ps.getFirstDriver(), ps.getFirstTireModel());
        stepRecord = new ArrayList<>();
        stepRecord.add(new Step(-1, currentState.copyState(), null));
        if (verbose) {
            System.out.println("Simulator: Resetting simulator");
            System.out.println("Simulator: \tStart " + currentState.toString());
            printTime();
        }
        startTime = System.currentTimeMillis();
        lastStepTime = startTime;
        return currentState.copyState();
    }

    /**
     * Perform an action against environment and receive the next state.
     *
     * @param a the action to perform
     * @return the next state or null if max time steps exceeded for problem
     */
    public State step(Action a) throws IllegalArgumentException {

        State nextState;

        if (!actionValidForLevel(a)) {
            throw new IllegalArgumentException("ActionType A"
                    + a.getActionType().getActionNo()
                    + " is an invalid action for problem level "
                    + ps.getLevel());
        }

        if (steps > ps.getMaxT()) {
            if (verbose) {
                System.out.println("Simulator: Max time steps exceeded: " + steps + " > "
                        + ps.getMaxT());
            }
            outputSteps(false);
            return null;
        }

        if (verbose) {
            System.out.println("Simulator: Step " + steps +": performing A"
                    + a.getActionType().getActionNo());
            printTimeSinceLastStep();
        }

        switch(a.getActionType().getActionNo()) {
            case 1:
                nextState = performA1();
                break;
            case 2:
                nextState = performA2(a);
                break;
            case 3:
                nextState = performA3(a);
                break;
            case 4:
                nextState = performA4(a);
                break;
            case 5:
                nextState = performA5(a);
                break;
            case 6:
                nextState = performA6(a);
                break;
            case 7:
                nextState = performA7(a);
                break;
            default:
                nextState = performA8(a);
        }

        // add step to record for outputting
        stepRecord.add(new Step(steps, nextState, a));

        // handle slip and breakdown cases, we do this now so we can generate
        // correct output format
        if (nextState.isInSlipCondition()) {
            // remain in same state but certain number of steps pass
            // -1 since we add 1 later
            steps += ps.getSlipRecoveryTime() - 1;
            nextState = nextState.changeSlipCondition(false);
        } else if (nextState.isInBreakdownCondition()) {
            steps += ps.getRepairTime() - 1;
            nextState = nextState.changeBreakdownCondition(false);
        }

        steps += 1;
        currentState = nextState.copyState();

        if (verbose) {
            System.out.println("Simulator: \tNext " + nextState.toString());
        }

        if (isGoalState(nextState)) {
            if (verbose) {
                System.out.println("Simulator: Goal reached after " + steps + " steps.");
            }
            outputSteps(true);
        }

        lastStepTime = System.currentTimeMillis();

        return nextState;
    }

    /**
     * Checks if given action is valid for the current problem level
     *
     * @param a the action being performed
     * @return True if action is value, False otherwise
     */
    private boolean actionValidForLevel(Action a) {
        return ps.getLevel().isValidActionForLevel(a.getActionType());
    }

    /**
     * Perform CONTINUE_MOVING action
     *
     * @return the next state
     */
    private State performA1() {

        State nextState;

        // check there is enough fuel to make move in current state
        int fuelRequired = getFuelConsumption();
        int currentFuel = currentState.getFuel();
        if (fuelRequired > currentFuel) {
            return currentState;
        }

        // Sample move distance
        int moveDistance = sampleMoveDistance();

        // handle slip and breakdown cases, addition of steps handled in step method
        if (moveDistance == ProblemSpec.SLIP) {
            if (verbose) {
                System.out.println("Simulator: \tSampled move distance=SLIP");
            }
            nextState = currentState.changeSlipCondition(true);
        } else if (moveDistance == ProblemSpec.BREAKDOWN) {
            if (verbose) {
                System.out.println("Simulator: \tSampled move distance=BREAKDOWN");
            }
            nextState = currentState.changeBreakdownCondition(true);
        } else {
            if (verbose) {
                System.out.println("Simulator: \tSampled move distance=" + moveDistance);
            }
            nextState = currentState.changePosition(moveDistance, ps.getN());
        }

        // handle fuel usage for level 2 and above
        if (ps.getLevel().getLevelNumber() > 1) {
            nextState = nextState.consumeFuel(fuelRequired);
        }

        return nextState;
    }

    /**
     * Return the move distance by sampling from conditional probability
     * distribution.
     *
     * N.B. this formula is not at all optimized for performance, so be wary if
     * trying to use it for finding a policy
     *
     * @return the move distance in range [-4, 5] or SLIP or BREAKDOWN
     */
    private int sampleMoveDistance() {

        double[] moveProbs = getMoveProbs();

        double p = Math.random();
        double pSum = 0;
        int move = 0;
        for (int k = 0; k < ProblemSpec.CAR_MOVE_RANGE; k++) {
            pSum += moveProbs[k];
            if (p <= pSum) {
                move = ps.convertIndexIntoMove(k);
                break;
            }
        }
        return move;
    }

    /**
     * Calculate the conditional move probabilities for the current state.
     *
     *          P(K | C, D, Ti, Te, Pressure)
     *
     * @return list of move probabilities
     */
    private double[] getMoveProbs() {

        // get parameters of current state
        Terrain terrain = ps.getEnvironmentMap()[currentState.getPos() - 1];
        int terrainIndex = ps.getTerrainIndex(terrain);
        String car = currentState.getCarType();
        // System.out.println("*************car: " + car);
        // print state
        // System.out.println("ProblemSpec---getMoveProbs: fqq currentState: " + currentState.toString());
        String driver = currentState.getDriver();
        Tire tire = currentState.getTireModel();

        // calculate priors
        double priorK = 1.0 / ProblemSpec.CAR_MOVE_RANGE;
        double priorCar = 1.0 / ps.getCT();
        double priorDriver = 1.0 / ps.getDT();
        double priorTire = 1.0 / ProblemSpec.NUM_TYRE_MODELS;
        double priorTerrain = 1.0 / ps.getNT();
        double priorPressure = 1.0 / ProblemSpec.TIRE_PRESSURE_LEVELS;

        // get probabilities of k given parameter
        double[] pKGivenCar = ps.getCarMoveProbability().get(car);
        // System.out.println("*************");
        // System.out.println("Simulator---getMoveProbs: fqq pKGivenCar: " + pKGivenCar[0]);
        // System.out.println("*************");
        double[] pKGivenDriver = ps.getDriverMoveProbability().get(driver);
        double[] pKGivenTire = ps.getTireModelMoveProbability().get(tire);
        double pSlipGivenTerrain = ps.getSlipProbability()[terrainIndex];
        double[] pKGivenPressureTerrain = convertSlipProbs(pSlipGivenTerrain);

        // use bayes rule to get probability of parameter given k
        double[] pCarGivenK = bayesRule(pKGivenCar, priorCar, priorK);
        double[] pDriverGivenK = bayesRule(pKGivenDriver, priorDriver, priorK);
        double[] pTireGivenK = bayesRule(pKGivenTire, priorTire, priorK);
        double[] pPressureTerrainGivenK = bayesRule(pKGivenPressureTerrain,
                (priorTerrain * priorPressure), priorK);

        // use conditional probability formula on assignment sheet to get what
        // we want (but what is it that we want....)
        double[] kProbs = new double[ProblemSpec.CAR_MOVE_RANGE];
        double kProbsSum = 0;
        double kProb;
        for (int k = 0; k < ProblemSpec.CAR_MOVE_RANGE; k++) {
            kProb = magicFormula(pCarGivenK[k], pDriverGivenK[k],
                    pTireGivenK[k], pPressureTerrainGivenK[k], priorK);
            kProbsSum += kProb;
            kProbs[k] = kProb;
        }

        // Normalize
        for (int k = 0; k < ProblemSpec.CAR_MOVE_RANGE; k++) {
            kProbs[k] /= kProbsSum;
        }

        return kProbs;
    }

    /**
     * Convert the probability of slipping on a given terrain with 50% tire
     * pressure into a probability list, of move distance versus current
     * terrain and tire pressure.
     *
     * @param slipProb probability of slipping on current terrain and 50%
     *                 tire pressure
     * @return list of move probabilities given current terrain and pressure
     */
    private double[] convertSlipProbs(double slipProb) {

        // Adjust slip probability based on tire pressure
        TirePressure pressure = currentState.getTirePressure();
        if (pressure == TirePressure.SEVENTY_FIVE_PERCENT) {
            slipProb *= 2;
        } else if (pressure == TirePressure.ONE_HUNDRED_PERCENT) {
            slipProb *= 3;
        }
        // Make sure new probability is not above max
        if (slipProb > ProblemSpec.MAX_SLIP_PROBABILITY) {
            slipProb = ProblemSpec.MAX_SLIP_PROBABILITY;
        }

        // for each terrain, all other action probabilities are uniform over
        // remaining probability
        double[] kProbs = new double[ProblemSpec.CAR_MOVE_RANGE];
        double leftOver = 1 - slipProb;
        double otherProb = leftOver / (ProblemSpec.CAR_MOVE_RANGE - 1);
        for (int i = 0; i < ProblemSpec.CAR_MOVE_RANGE; i++) {
            if (i == ps.getIndexOfMove(ProblemSpec.SLIP)) {
                kProbs[i] = slipProb;
            } else {
                kProbs[i] = otherProb;
            }
        }

        return kProbs;
    }

    /**
     * Apply bayes rule to all values in cond probs list.
     *
     * @param condProb list of P(B|A)
     * @param priorA prior probability of parameter A
     * @param priorB prior probability of parameter B
     * @return list of P(A|B)
     */
    private double[] bayesRule(double[] condProb, double priorA, double priorB) {

        double[] swappedProb = new double[condProb.length];

        for (int i = 0; i < condProb.length; i++) {
            swappedProb[i] = (condProb[i] * priorA) / priorB;
        }
        return swappedProb;
    }

    /**
     * Conditional probability formula from assignment 2 sheet
     *
     * @param pA P(A | E)
     * @param pB P(B | E)
     * @param pC P(C | E)
     * @param pD P(D | E)
     * @param priorE P(E)
     * @return numerator of the P(E | A, B, C, D) formula (still need to divide
     *      by sum over E)
     */
    private double magicFormula(double pA, double pB, double pC, double pD,
                               double priorE) {
        return pA * pB * pC * pD * priorE;
    }

    /**
     * Get the fuel consumption of moving given the current state
     *
     * @return move fuel consumption for current state
     */
    private int getFuelConsumption() {

        // get parameters of current state
        Terrain terrain = ps.getEnvironmentMap()[currentState.getPos() - 1];
        String car = currentState.getCarType();
        TirePressure pressure = currentState.getTirePressure();

        // get fuel consumption
        int terrainIndex = ps.getTerrainIndex(terrain);
        int carIndex = ps.getCarIndex(car);
        int fuelConsumption = ps.getFuelUsage()[terrainIndex][carIndex];

        if (pressure == TirePressure.FIFTY_PERCENT) {
            fuelConsumption *= 3;
        } else if (pressure == TirePressure.SEVENTY_FIVE_PERCENT) {
            fuelConsumption *= 2;
        }
        return fuelConsumption;
    }

    /**
     * Perform CHANGE_CAR action
     *
     * @param a a CHANGE_CAR action object
     * @return the next state
     */
    private State performA2(Action a) {

        if (currentState.getCarType().equals(a.getCarType())) {
            // changing to same car type does not change state but still costs a step
            // no cheap refill here, muhahaha
            return currentState;
        }

        return currentState.changeCarType(a.getCarType());
    }

    /**
     * Perform CHANGE_DRIVER action
     *
     * @param a a CHANGE_DRIVER action object
     * @return the next state
     */
    private State performA3(Action a) { return currentState.changeDriver(a.getDriverType()); }

    /**
     * Perform the CHANGE_TIRES action
     *
     * @param a a CHANGE_TIRES action object
     * @return the next state
     */
    private State performA4(Action a) {
        return currentState.changeTires(a.getTireModel());
    }

    /**
     * Perform the ADD_FUEL action
     *
     * @param a a ADD_FUEL action object
     * @return the next state
     */
    private State performA5(Action a) {
        // calculate number of steps used for refueling (minus 1 since we add
        // 1 in main function
        int stepsRequired = (int) Math.ceil(a.getFuel() / (float) 10);
        steps += (stepsRequired - 1);
        return currentState.addFuel(a.getFuel());
    }

    /**
     * Perform the CHANGE_PRESSURE action
     *
     * @param a a CHANGE_PRESSURE action object
     * @return the next state
     */
    private State performA6(Action a) {
        return currentState.changeTirePressure(a.getTirePressure());
    }

    /**
     * Perform the CHANGE_CAR_AND_DRIVER action
     *
     * @param a a CHANGE_CAR_AND_DRIVER action object
     * @return the next state
     */
    private State performA7(Action a) {

        if (currentState.getCarType().equals(a.getCarType())) {
            // if car the same, only change driver so no sneaky fuel exploit
            return currentState.changeDriver(a.getDriverType());
        }
        return currentState.changeCarAndDriver(a.getCarType(),
                a.getDriverType());
    }

    /**
     * Perform the CHANGE_TIRE_FUEL_PRESSURE action
     *
     * @param a a CHANGE_TIRE_FUEL_PRESSURE action object
     * @return the next state
     */
    private State performA8(Action a) {
        // calculate number of steps used for refueling (minus 1 since we add
        // 1 in main function
        int stepsRequired = (int) Math.ceil(a.getFuel() / (float) 10);
        steps += (stepsRequired - 1);
        return currentState.changeTireFuelAndTirePressure(a.getTireModel(),
                a.getFuel(), a.getTirePressure());
    }

    /**
     * Check whether a given state is the goal state or not
     *
     * @param s the state to check
     * @return True if s is goal state, False otherwise
     */
    public Boolean isGoalState(State s) {
        if (s == null) {
            return false;
        }
        return s.getPos() >= ps.getN();
    }

    /**
     * Get the current number of steps taken in latest simulation
     *
     * @return steps taken in latest simulation
     */
    public int getSteps() {
        return steps;
    }

    /**
     * Get the current state of the environment
     *
     * @return the current state
     */
    public State getCurrentState() {
        return currentState.copyState();
    }

    /**
     * Write the step record to the output file
     *
     * @param goalReached whether the goal was reached
     */
    private void outputSteps(boolean goalReached) {

        System.out.println("Simulator: Writing steps to output file");
        printTime();
        System.out.println("Simulator: Total time taken: " + ((System.currentTimeMillis() - startTime) / 1000) + " seconds");

        try (BufferedWriter output = new BufferedWriter(new FileWriter(outputFile))) {

            for (Step s: stepRecord) {
                output.write(s.getOutputFormat());
            }

            if (goalReached) {
                output.write("Goal reached, you bloody ripper!");
            } else {
                output.write("Computer says no. Max steps reached: max steps = " + ps.getMaxT());
            }

        } catch (IOException e) {
            System.out.println("Simulator: Error with output file");
            System.out.println(e.getMessage());
            System.out.println("Simulator: Vomiting output to stdout instead");
            for (Step s: stepRecord) {
                System.out.print(s.getOutputFormat());
            }

            if (goalReached) {
                System.out.println("Simulator: Goal reached, you bloody ripper!");
            } else {
                System.out.println("Simulator: Computer says no. Max steps reached: max steps = " + ps.getMaxT());
            }
        }
    }

    private void printTime() {
        Date today = new Date();
        System.out.println("Simulator: Time is:" +today.toLocaleString());
    }

    private void printTimeSinceLastStep() {
        long timePassed = (System.currentTimeMillis() - lastStepTime) / 1000;
        System.out.println("Simulator: Time since last action: " + timePassed + " seconds");
    }

    public boolean isValidFuelToMove(int fuel) {
        // check there is enough fuel to make move in current state
        return fuel >= getFuelConsumption();
    }
    
    /**
     * calculate the expected reward for a given state and action
     * @param currentState current state
     * @param action current action
     * @return immediate reward
     */
    public double getReward(State currentState, Action action) {
        // System.out.println("Simulator---getReward: fqq getReward");
        // calculate the immediate reward for the given state and action
        double reward = 0.0;

        // if the current state is the goal state, set the reward to 100
        if (isGoalState(currentState)) {
            // System.out.println("Simulator---getReward: fqq isGoalState");
            reward = 100.0;
        } 
        // else if the current state is not the goal state, calculate the reward based on the action
        else {
            switch (action.getActionType()) {
                case MOVE:
                    // for move action, A1, calculate the expected reward
                    reward = calculateMoveReward(currentState, action);
                    break;
                    
                case CHANGE_CAR:
                case CHANGE_DRIVER:
                case CHANGE_TIRES:
                case CHANGE_PRESSURE:
                    // for change car, change driver, change tires, change pressure actions, A2, A3, A4, A6, set the reward to -5
                    reward = -5.0;
                    break;
                
                case ADD_FUEL:
                    // for add fuel action, A5, calculate the expected reward, calculate the waiting time, -5.0*ceil(fuel/10)
                    reward = -5.0 * Math.ceil(action.getFuel() / 10.0);
                    break;
                
                case CHANGE_CAR_AND_DRIVER:
                    // for change car and driver action, A7, set the reward to -5
                    reward = -5.0; 
                    break;

                case CHANGE_TIRE_FUEL_PRESSURE:
                    // for change tire, fuel and pressure action, A8, set the reward to the maximum of time cost 
                    reward = Math.max(-5, -5.0 * Math.ceil(action.getFuel() / 10));
                    break;

                default:
                    // for other actions, set the reward to 0, should not happen
                    reward = 0.0;
                    break;
            }
        }
        // System.out.println("Simulator---getReward: fqq reward: " + reward);
        return reward;
    }

    /** 
     * calculate the expected reward for a given state and action
     * @param currentState current state
     * @param action current action
     * @return double move reward
     */
    public double calculateMoveReward(State currentState, Action action) {
        // get every possible move distance probability, P(k)
        double[] moveProbs = getMoveProbs();
        // // print moveProbs
        // for (int i = 0; i < moveProbs.length; i++) {
        //     System.out.println("Simulator---calculateMoveReward: fqq moveProbs[" + i + "]: " + moveProbs[i]);
        // }

        double expectedReward = 0.0;

        // traverse all possible move distances, calculate the cost of each distance
        for (int k = 0; k < moveProbs.length; k++) {
            int distance = k + ProblemSpec.CAR_MIN_MOVE; // get the move distance k
            if (distance <= ProblemSpec.CAR_MAX_MOVE) {
                // System.out.println("Simulator---calculateMoveReward: fqq car moving with k: " + (k+ProblemSpec.CAR_MIN_MOVE));
                double moveProb = moveProbs[k];  // get the probability P(k) of moving k steps
                // calculate the cost of moving k steps
                double moveCost = calculateMoveCost(currentState, distance);
                // calculate the expected reward (negative means punishment)
                expectedReward += moveProb * (distance*5 -moveCost);
                
            }else if (distance == ProblemSpec.SLIP) {
                // slip 
                // System.out.println("Simulator---calculateMoveReward: fqq slip with k: " + k);
                expectedReward += -10.0 * moveProbs[k] * ps.getSlipRecoveryTime();
            } else if (distance == ProblemSpec.BREAKDOWN) {
                // breakdown
                // System.out.println("Simulator---calculateMoveReward: fqq breakdown with k: " + k);
                expectedReward += -10.0 * moveProbs[k] * ps.getRepairTime();
                
            }
        }

        // System.out.println("Simulator---calculateMoveReward: fqq expectedReward: " + expectedReward);
        return expectedReward;
    }

    /**
     * Calculate the cost of moving k steps, such as fuel consumption or time penalty
     * @param currentState current state
     * @param moveDistance move distance k
     * @return double move cost
     */
    private double calculateMoveCost(State currentState, int moveDistance) {

        // calculate the base cost of moving k steps
        int baseCost = getFuelConsumption();
        // System.out.println("Simulator---calculateMoveCost: fqq baseCost: " + baseCost);

        // calculate the cost of moving k steps based on the tire pressure
        TirePressure pressure = currentState.getTirePressure();
        if (pressure == TirePressure.FIFTY_PERCENT) {
            baseCost *= 3;  // 50% pressure fuel consumption is 3 times normal
        } else if (pressure == TirePressure.SEVENTY_FIVE_PERCENT) {
            baseCost *= 2;  // 75% pressure fuel consumption is 2 times normal
        }

        // System.out.println("Simulator---calculateMoveCost: fqq baseCost: " + baseCost);
        return baseCost;
    }

    /**
     * Get the future value of the current state and action
     * @param currentState current state
     * @param action current action
     * @return double future value
     */
    public double getFutureValue(State currentState, Action action) {
        // System.out.println("Simulator---getFutureValue: fqq getFutureValue start");
        // System.out.println("Simulator---getFutureValue: fqq currentState: " + currentState.toString());
        // get the expected reward for the current state and action, ∑ P(s'|s,a) * V(s')
        double futureValue = 0.0;

        
        // calculate the move probability for the current state
        double[] moveProbs = getMoveProbs();
        // print moveProbs
        // for (int i = 0; i < moveProbs.length; i++) {
        //     System.out.println("Simulator---getFutureValue: fqq moveProbs[" + i + "]: " + moveProbs[i]);
        // }

        // traverse all possible move distances, calculate the cost of each distance
        for (int k = 0; k < moveProbs.length; k++) {
            int distance = k + ProblemSpec.CAR_MIN_MOVE;
            if (distance <= ProblemSpec.CAR_MAX_MOVE) {
                // System.out.println("Simulator---getFutureValue: fqq car moving with k: " + (k+ProblemSpec.CAR_MIN_MOVE));
                double moveProb = moveProbs[k];  // get the probability P(k) of moving k steps
                // calculate the cost of moving k steps 
                double moveCost = calculateMoveCost(currentState, distance);

                // calculate the expected reward 
                futureValue += moveProb * (distance*6-moveCost);

                // if is negative move, add extra cost
                if (distance < 0) {
                    futureValue += 4.0* distance;
                }
                
            }else{
                // slip or breakdown
                // System.out.println("Simulator---getFutureValue: fqq slip or breakdown with k: " + k);
                futureValue += -20.0 * moveProbs[k];
            }
        }
        // System.out.println("Simulator---getFutureValue: fqq futureValue: " + futureValue);

        return futureValue;
    }

    /** 
     * copy current simulator
     * @return Simulator
     *
     */
    public Simulator copySimulator() {
        Simulator newSimulator = new Simulator(ps, outputFile);
        // copy variables
        newSimulator.ps = this.ps;
        /** path and name for output file **/
        newSimulator.outputFile = this.outputFile;
        /** The current state of the environment **/
        newSimulator.currentState = this.currentState;
        /** The number of steps taken **/
        newSimulator.steps = this.steps;
        /** Whether to print progress messages or not
         * Feel free to change this if you don't want messages printed **/
        newSimulator.verbose = this.verbose;
        /** copy the store steps for output **/
        newSimulator.stepRecord =  new ArrayList<>(this.stepRecord);
        /** timer **/
        newSimulator.startTime = this.startTime;
        newSimulator.lastStepTime = this.lastStepTime;
        return newSimulator;
    }


}
