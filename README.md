### Overview
Real-Time Dynamic Programming (RTDP) and Labeled RTDP implementation.

### Implementation

1. **Single Run**: The system executes either RTDP or Labeled RTDP once, calculating the number of steps to reach the goal.
2. **Multiple Runs**: The system runs both RTDP and Labeled RTDP multiple times to gather performance statistics such as mean, median, and standard deviation.

- **RTDP**: The standard Real-Time Dynamic Programming algorithm is run to solve the MDP.
![RTDP Pseudocode](/RTDP_pseudocode.png)
- **Labeled RTDP**: This variant of RTDP labels states that have converged, reducing unnecessary updates to speed up the computation.
![Labeled RTDP Pseudocode](/Labeled_RTDP_pseudocode.png)


## Input and Output

### Input


The input file follows a structured format, with each line containing specific information as described below:

1. **Game Level**: The first line contains a single integer (1-5) that specifies the level of the game.
   
2. **Discount Factor, Slip Recovery Time, Breakdown Repair Time**: The second line contains three numbers separated by spaces:
   - The **discount factor** (γ).
   - The **time required to recover from a slip**.
   - The **time required to repair the car after a breakdown**.

3. **Number of Cells and Maximum Time Steps**: The third line contains two numbers separated by a space:
   - The number of **cells in the map** (N).
   - The **maximum number of time steps** (MaxT) allowed to reach the goal.

4. **Terrain Map**: Each of the following lines describes a terrain type and the corresponding cell indices, formatted as `terrain_type: cell_indices`. Cell indices can be individual numbers or ranges of numbers, separated by commas.

5. **Car Types**: After the terrain map, a line specifies the number of car types (CT), followed by CT lines where each line contains:
   - A **car type name** followed by 12 numbers. These numbers represent the probabilities for different movement outcomes (ranging from -4 to 5, plus slip and breakdown) when this car type is used.

6. **Driver Types**: After the car types, a line specifies the number of driver types (DT), followed by DT lines where each line contains:
   - A **driver name** followed by 12 numbers. These numbers represent the probabilities for different movement outcomes (ranging from -4 to 5, plus slip and breakdown) when this driver is used.

7. **Tire Models**: Next, there are four lines where each line describes a tire model, followed by 12 numbers representing the probabilities for different movement outcomes.

8. **Fuel Usage**: The next line contains NT × CT numbers, where each number represents the fuel usage for a combination of terrain type and car type. NT is the number of terrain types, and CT is the number of car types.

9. **Slip Probabilities**: The next line contains NT numbers representing the slip probability when the tire pressure is at 50% capacity, for each terrain type.

---

### Output 

The output file consists of a sequence of lines, each following this format:

1. **Time-step**: The current time-step. If the car slips or breaks down, the time-step will jump accordingly to reflect the recovery or repair time.

2. **Car Status and Actions**: Each line contains two parts separated by a semicolon (`;`):
   - The **first part** is a tuple with the following 8 components, separated by commas:
     1. The current **cell index** where the car is located.
     2. The **slip status** (1 if slipping, 0 if not).
     3. The **breakdown status** (1 if broken down, 0 if not).
     4. The current **car type**.
     5. The current **driver**.
     6. The current **tire model**.
     7. The remaining **fuel level**.
     8. The current **tire pressure**.
   - The **second part** is a tuple that contains the **action(s) performed at the current time-step**. If no action is performed, it will be marked as `(n.a.)`.

