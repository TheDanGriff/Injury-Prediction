# %% Directories, paths, and imports. You should not need to change anything.
import os
import sys
import numpy as np
import pandas as pd

baseDir = os.getcwd()
opensimADDir = os.path.join(baseDir, 'UtilsDynamicSimulations', 'OpenSimAD')
sys.path.append(baseDir)
sys.path.append(opensimADDir)

from utilsOpenSimAD import processInputsOpenSimAD, plotResultsOpenSimAD
from mainOpenSimAD import run_tracking

# Function to save data to CSV
def save_to_csv(data, filename, output_dir):
    df = pd.DataFrame(data)
    df.to_csv(os.path.join(output_dir, filename), index=False)

# %% User inputs.
session_id = "4c262e12-931b-45fd-97cc-86b3655bb656"  # Replace with your actual session ID
case = 'jump_analysis'  # Name your case

# Set to True to solve the optimal control problem.
solveProblem = True
# Set to True to analyze the results of the optimal control problem.
analyzeResults = True

# Path to where you want the data to be downloaded.
dataFolder = os.path.join(baseDir, 'Data')

# Trials to analyze
trials = [
    {"trial_name": "jump2", "time_window": [6.07, 6.84]},
    {"trial_name": "jump3", "time_window": [6.35, 7.3]},
]

# %% Setup. 

for trial in trials:
    trial_name = trial["trial_name"]
    time_window = trial["time_window"]
    motion_type = 'jumping'  # Assuming the motion type is jumping

    settings = processInputsOpenSimAD(baseDir, dataFolder, session_id, trial_name, 
                                      motion_type, time_window)

    # %% Simulation.
    run_tracking(baseDir, dataFolder, session_id, settings, case=case, 
                  solveProblem=solveProblem, analyzeResults=analyzeResults)

    # %% Save Results to CSV.
    results_file = os.path.join(dataFolder, session_id, 'OpenSimData', 'Dynamics', trial_name, 'optimaltrajectories.npy')
    if not os.path.exists(results_file):
        raise FileNotFoundError(f"Results file not found: {results_file}")

    results = np.load(results_file, allow_pickle=True).item()

    # Create an output directory for the CSV files
    output_dir = os.path.join(dataFolder, session_id, 'CSV_Outputs', trial_name)
    os.makedirs(output_dir, exist_ok=True)

    # List of keys to save (normalized versions where applicable)
    keys_to_save = [
        'coordinate_values_toTrack', 'coordinate_values', 
        'coordinate_speeds_toTrack', 'coordinate_speeds', 
        'coordinate_accelerations_toTrack', 'coordinate_accelerations', 
        'torques_BWht', 'powers', 'GRF_BW', 'GRM_BWht', 
        'COP', 'freeM', 'coordinates', 'coordinates_power', 
        'rotationalCoordinates', 'GRF_labels', 'GRM_labels', 
        'COP_labels', 'time', 'muscles', 'passive_limit_torques', 
        'muscle_driven_joints', 'limit_torques_joints', 
        'KAM_BWht', 'KAM_labels', 'MCF_BW', 'MCF_labels', 
        'muscle_activations', 'muscle_forces', 'passive_muscle_torques'
    ]

    # Save the selected data to CSV
    for key in keys_to_save:
        if key in results['0']:
            save_to_csv(results['0'][key], f"{key}.csv", output_dir)
        else:
            print(f"Key '{key}' not found in the results dictionary for {trial_name}.")

    print(f"CSV files have been successfully created and saved for {trial_name} in the output directory.")

# %% Plots (if needed).
for trial in trials:
    trial_name = trial["trial_name"]
    plotResultsOpenSimAD(dataFolder, session_id, trial_name, settings, cases=[case])
