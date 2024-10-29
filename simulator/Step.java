package simulator;

import problem.Action;

/**
 * An immutable class for storing the history for a given step used for output.
 * Keeps track of:
 *  1. step
 *  2. State - at end of step
 *  3. Action performed
 */
public class Step {

    /** The step number **/
    private int step;
    /** The state at end of step **/
    private State state;
    /** The action performed during step **/
    private Action action;

    /**
     * Construct a new step instance
     *
     * @param step the step number (-1 for initial state)
     * @param state the state at end of step
     * @param action the action taken (null if no action taken)
     */
    public Step(int step, State state, Action action) {
        this.step = step;
        this.state = state;
        this.action = action;
    }

    /**
     * Return the step instance as a string in the format specified by assignment 2
     * spec.
     *
     *      step;(pos,slip,breakdown,car,driver,tire,tirePressure);(actionNumber:actionValues)
     *
     * @return string representation of step as per assignment 2 spec
     */
    public String getOutputFormat() {
        StringBuilder sb = new StringBuilder();

        if (step == -1) {
            sb.append("start");
        } else {
            sb.append(step);
        }
        // add state tuple
        sb.append(";(");
        sb.append(state.getPos()).append(",");
        sb.append(booleanToInt(state.isInSlipCondition())).append(",");
        sb.append(booleanToInt(state.isInBreakdownCondition())).append(",");
        sb.append(state.getCarType()).append(",");
        sb.append(state.getDriver()).append(",");
        sb.append(state.getTireModel().asString()).append(",");
        sb.append(state.getFuel()).append(",");
        sb.append(state.getTirePressure().asString()).append(",)");
        // add action
        sb.append(";(");
        if (action == null) {
            sb.append("n.a.");
        } else {
            sb.append(action.getText());
        }
        sb.append(")\n");

        return sb.toString();
    }


    private static int booleanToInt(boolean value) {
        // Convert true to 1 and false to 0.
        return value ? 1 : 0;
    }
}
