package problem;

import java.io.IOException;

import simulator.Simulator;
import simulator.State;
import simulator.Step;

public class Main {

    public static void main(String[] args) {

        // test file paths, absolute path
        String currentWorkingDir = System.getProperty("user.dir");
        System.out.println("Current working directory: " + currentWorkingDir);
        String inputFile_1_4 = currentWorkingDir + "/input/input_lvl1_4.txt";
        String inputFile_2_2 = currentWorkingDir + "/input/input_lvl2_2.txt";
        String inputFile_3_3 = currentWorkingDir + "/input/input_lvl3_3.txt";
        String inputFile_4_2 = currentWorkingDir + "/input/input_lvl4_2.txt";
        String inputFile_5_2 = currentWorkingDir + "/input/input_lvl5_2.txt";

        String outputFile_1_4 = currentWorkingDir + "/output/output_lvl1_4.txt";
        String outputFile_2_2 = currentWorkingDir + "/output/output_lvl2_2.txt";
        String outputFile_3_3 = currentWorkingDir + "/output/output_lvl3_3.txt";
        String outputFile_4_2 = currentWorkingDir + "/output/output_lvl4_2.txt";
        String outputFile_5_2 = currentWorkingDir + "/output/output_lvl5_2.txt";

        // run the process for one time
        int goalSteps = singleProcess(inputFile_5_2, outputFile_5_2);
        System.out.println("The number of steps to reach the goal state is: " + goalSteps);

        // run the process for multiple times(runTimes) to get the statistics
        // int runTimes = 50;
        // int [][] results = repeatProcess(runTimes, inputFile_5_2, outputFile_5_2);
        // int[] resultRTDP = results[0];
        // int[] resultLRTDP = results[1];
        
        // statistics(results);
    }

    /**
     * create a method to run the process for one time
     * @param inputFile
     * @param outputFile
     * @return the number of steps to reach the goal state, if not reach, return -1
     */
    private static int singleProcess(String inputFile, String outputFile) {
        ProblemSpec ps;
        try {
            ps = new ProblemSpec(inputFile);
            Simulator simulator = new Simulator(ps, outputFile);
            State initialState = simulator.getCurrentState();
            RTDPSolver solver = new RTDPSolver(ps, simulator, initialState);
            solver.runLabeledRTDP();
            if (simulator.isGoalState(simulator.getCurrentState())) {
                return simulator.getSteps();
            } else {
                return -1;
            }
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
            System.exit(1);
        }
        return -1;

    }

    /**
     * create a method to repeat the process for runTimes
     * @param runTimes
     * @return an array of int runTimes elements, 
     * each element is the number of steps to reach the goal state, if not reach, return -1
     */
    private static int[][] repeatProcess(int runTimes, String inputFile, String outputFile) {
        int[] resultRTDP = new int[runTimes];
        int[] resultLRTDP = new int[runTimes];
        for (int i = 0; i < runTimes; i++) {
            try {
                // RTDP
                ProblemSpec ps = new ProblemSpec(inputFile);
                Simulator simulator = new Simulator(ps, outputFile);
                State initialState = simulator.getCurrentState();
                RTDPSolver solver = new RTDPSolver(ps, simulator, initialState);
                solver.runRTDP();
                if (simulator.isGoalState(simulator.getCurrentState())) {
                    resultRTDP[i] = simulator.getSteps();
                } else {
                    resultRTDP[i] = -1;
                }

                // LRTDP
                ps = new ProblemSpec(inputFile);
                simulator = new Simulator(ps, outputFile);
                initialState = simulator.getCurrentState();
                solver = new RTDPSolver(ps, simulator, initialState);
                solver.runLabeledRTDP();
                if (simulator.isGoalState(simulator.getCurrentState())) {
                    resultLRTDP[i] = simulator.getSteps();
                } else {
                    resultLRTDP[i] = -1;
                }
            } catch (IOException e) {
                System.out.println("IO Exception occurred");
                System.exit(1);
            }
        }
        System.out.println("The number of steps to reach the goal state for RTDP is: ");
        for (int i = 0; i < runTimes; i++) {
            System.out.print(resultRTDP[i] + " ");
        }
        System.out.println("The number of steps to reach the goal state for LRTDP is: ");
        for (int i = 0; i < runTimes; i++) {
            System.out.print(resultLRTDP[i] + " ");
        }
        return new int[][] {resultRTDP, resultLRTDP};
    }

    /** 
     * statistics for the results, calculate mean, median, standard deviation
     * @param result
     * @return an array of double, the first element is the mean, the second element is the median, the third element is the standard deviation
     */
    private static double[][] statistics(int[][] result) {
        double[] statisticsRTDP = new double[3];
        double[] statisticsLRTDP = new double[3];
        double sumRTDP = 0;
        double sumLRTDP = 0;
        for (int i = 0; i < result.length; i++) {
            sumRTDP += result[0][i];
            sumLRTDP += result[1][i];
        }
        statisticsRTDP[0] = sumRTDP / result.length;
        statisticsLRTDP[0] = sumLRTDP / result.length;

        double medianRTDP;
        double medianLRTDP;
        if (result.length % 2 == 0) {
            medianRTDP = (result[0][result.length / 2 - 1] + result[0][result.length / 2]) / 2;
            medianLRTDP = (result[1][result.length / 2 - 1] + result[1][result.length / 2]) / 2;
        } else {
            medianRTDP = result[0][result.length / 2];
            medianLRTDP = result[1][result.length / 2];
        }
        statisticsRTDP[1] = medianRTDP;
        statisticsLRTDP[1] = medianLRTDP;

        double sumOfSquaresRTDP = 0;
        double sumOfSquaresLRTDP = 0;
        for (int i = 0; i < result.length; i++) {
            sumOfSquaresRTDP += Math.pow(result[0][i] - statisticsRTDP[0], 2);
            sumOfSquaresLRTDP += Math.pow(result[1][i] - statisticsLRTDP[0], 2);
        }
        statisticsRTDP[2] = Math.sqrt(sumOfSquaresRTDP / result.length);
        statisticsLRTDP[2] = Math.sqrt(sumOfSquaresLRTDP / result.length);

        System.out.println("The mean, median, and standard deviation for RTDP are: ");
        for (int i = 0; i < 3; i++) {
            System.out.print(statisticsRTDP[i] + " ");
        }
        System.out.println("The mean, median, and standard deviation for LRTDP are: ");
        for (int i = 0; i < 3; i++) {
            System.out.print(statisticsLRTDP[i] + " ");
        }

        return new double[][] {statisticsRTDP, statisticsLRTDP};
    }

}