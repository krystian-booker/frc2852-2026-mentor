import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy as np

def analyze_and_plot(pid_file, bangbang_file):
    try:
        pid_df = pd.read_csv(pid_file)
        bb_df = pd.read_csv(bangbang_file)
    except Exception as e:
        print(f"Error reading CSV files: {e}")
        return

    # To calculate precision and accuracy properly, we should isolate the "steady state"
    # To do this, we'll wait until the RPM first crosses 95% of target RPM
    
    def calculate_stats(df, name):
        if len(df) == 0:
            print(f"{name} has no data.")
            return

        target_rpm = df['target_rpm'].iloc[0]
        
        # Find where it first reaches 95% of target
        threshold = target_rpm * 0.95
        reached_target_idx = df.index[df['current_rpm'] >= threshold].tolist()
        
        if not reached_target_idx:
            print(f"{name}: NEVER reached 95% of target RPM ({threshold}). Cannot calculate steady state stats.")
            return

        steady_state_df = df.iloc[reached_target_idx[0]:]
        
        if len(steady_state_df) < 10:
            print(f"{name}: Not enough data after reaching target to calculate stats.")
        
        mae = np.mean(np.abs(steady_state_df['current_rpm'] - target_rpm))
        std_dev = np.std(steady_state_df['current_rpm'])
        
        print(f"--- {name} Results ---")
        print(f"Target RPM: {target_rpm}")
        print(f"Accuracy (Mean Absolute Error): {mae:.2f} RPM (Lower is better)")
        print(f"Precision (Standard Deviation): {std_dev:.2f} RPM (Lower is better)")
        print()

    print("Analyzing Steady State Performance (after first reaching target)...\n")
    calculate_stats(pid_df, "PID Control")
    calculate_stats(bb_df, "Bang-Bang Control")

    # Plotting
    plt.figure(figsize=(10, 6))
    
    # Normalize time to start at 0
    pid_time = pid_df['timestamp'] - pid_df['timestamp'].iloc[0]
    bb_time = bb_df['timestamp'] - bb_df['timestamp'].iloc[0]

    plt.plot(pid_time, pid_df['current_rpm'], label='PID Current RPM', color='blue')
    plt.plot(bb_time, bb_df['current_rpm'], label='Bang-Bang Current RPM', color='orange')
    
    plt.axhline(y=pid_df['target_rpm'].iloc[0], color='red', linestyle='--', label='Target RPM')

    plt.title('Flywheel Control Loop Comparison: PID vs Bang-Bang')
    plt.xlabel('Time (seconds)')
    plt.ylabel('RPM')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compare_flywheel.py <path_to_pid.csv> <path_to_bangbang.csv>")
        sys.exit(1)
        
    analyze_and_plot(sys.argv[1], sys.argv[2])
