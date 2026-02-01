import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. Configuration
FILE_NAME = 'pid7_performance.csv'
STABILITY_THRESHOLD = 10  # Tolerance band for Settling Time (pixels)

# 2. Data Loading
# We assume the CSV header is correct since the C++ code was updated
try:
    data = pd.read_csv(FILE_NAME)
except FileNotFoundError:
    print(f"Error: {FILE_NAME} not found.")
    exit()

# Extracting Final Scores
final_iae = data['IAE'].iloc[-1]
final_ise = data['ISE'].iloc[-1]
total_time = data['Time'].iloc[-1]

# 3. Reliability Calculation (Peaks > 100px)
critical_peaks = data[data['Error'].abs() > 100]
num_peaks = len(critical_peaks)
reliability_score = ((len(data) - num_peaks) / len(data)) * 100

# 4. Average Settling Time Calculation
def calculate_settling_time(df, tolerance):
    """
    Calculates the time the robot takes to return and stay within 
    the stability zone after a disturbance.
    """
    settling_times = []
    is_disturbed = False
    start_time = 0
    
    for i in range(1, len(df)):
        err = abs(df['Error'].iloc[i])
        t = df['Time'].iloc[i]
        
        # Detected leaving the tolerance band
        if err > tolerance and not is_disturbed:
            is_disturbed = True
            start_time = t
        # Detected returning to the tolerance band
        elif err <= tolerance and is_disturbed:
            # Check stability over the next 5 data points
            if df['Error'].iloc[i:i+5].abs().max() <= tolerance:
                settling_times.append(t - start_time)
                is_disturbed = False
                
    return np.mean(settling_times) if settling_times else 0

avg_settling_time = calculate_settling_time(data, STABILITY_THRESHOLD)

# 5. Console Output
print(f"\n" + "="*45)
print(f" FINAL PERFORMANCE ANALYSIS: {FILE_NAME}")
print(f"="*45)
print(f" Total Traversal Time : {total_time:.2f} s")
print(f" IAE (Accuracy)       : {final_iae:.2f}")
print(f" ISE (Stability)      : {final_ise:.2f}")
print(f" Reliability (>100px) : {reliability_score:.2f}% ({num_peaks} peaks)")
print(f" Avg. Settling Time   : {avg_settling_time:.3f} s")
print(f"="*45 + "\n")

# 6. Professional Visualization
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

# --- Error Analysis Plot ---
ax1.plot(data['Time'], data['Error'], color='#c0392b', label='Tracking Error (px)', linewidth=1.5)
ax1.axhline(0, color='black', linewidth=1)
ax1.axhline(STABILITY_THRESHOLD, color='green', linestyle=':', alpha=0.6, label=f'Stability Zone (±{STABILITY_THRESHOLD}px)')
ax1.axhline(-STABILITY_THRESHOLD, color='green', linestyle=':', alpha=0.6)
ax1.axhline(100, color='orange', linestyle='--', alpha=0.4, label='Critical Threshold (100px)')
ax1.axhline(-100, color='orange', linestyle='--', alpha=0.4)

ax1.set_title(f'PID Performance Report - {FILE_NAME}\nIAE: {final_iae:.0f} | ISE: {final_ise:.0f} | Reliability: {reliability_score:.1f}%', fontsize=14)
ax1.set_ylabel('Centroid Offset (Pixels)')
ax1.grid(True, linestyle='--', alpha=0.5)
ax1.legend(loc='upper right')

# --- Motor Speeds & BaseSpeed Plot ---
ax2.plot(data['Time'], data['LeftSpeed'], label='Left Motor', color='#2980b9')
ax2.plot(data['Time'], data['RightSpeed'], label='Right Motor', color='#f39c12')
if 'baseSpeed' in data.columns:
    ax2.plot(data['Time'], data['baseSpeed'], label='Target Base Speed', color='black', linestyle='--', alpha=0.3)

ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Velocity (rad/s)')
ax2.set_title('Actuator Commands')
ax2.grid(True, linestyle='--', alpha=0.5)
ax2.legend(loc='upper right')

# Final formatting and save
plt.tight_layout()
output_filename = FILE_NAME.replace('.csv', '_report.png')
plt.savefig(output_filename, dpi=300)
print(f"Report saved as: {output_filename}")
plt.show()