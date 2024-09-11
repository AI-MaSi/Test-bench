import numpy as np
import matplotlib.pyplot as plt

# Constants
NUM_POINTS = 1000
GAMMA_VALUES = [0.3, 1.0, 3.0]  # More pronounced gamma effect
DEAD_ZONE = 20  # 20% dead zone


def apply_gamma(x, gamma):
    return x ** gamma


def find_input_for_output(output_values, target_output):
    return np.interp(target_output, output_values, input_values)


# Generate input values
input_values = np.linspace(0, 100, NUM_POINTS)

# Create figure with two subplots (main plot and info box)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), height_ratios=[3, 1], gridspec_kw={'hspace': 0.3})

# Plot on the main axis (ax1)
ax1.axhspan(0, DEAD_ZONE, color='lightgray', alpha=0.5, label='Dead Zone')

for gamma in GAMMA_VALUES:
    output_values = apply_gamma(input_values / 100, gamma) * 100
    ax1.plot(input_values, output_values, label=f'Gamma = {gamma}', linewidth=2)

    # Find input value for DEAD_ZONE output
    input_at_deadzone = find_input_for_output(output_values, DEAD_ZONE)
    ax1.axvline(x=input_at_deadzone, color='red', linestyle='--', alpha=0.5)
    # Move the text label down to just above the 100% line
    ax1.text(input_at_deadzone, 101, f'{input_at_deadzone:.1f}%',
             rotation=90, verticalalignment='bottom', horizontalalignment='right')

ax1.set_xlabel('Input %')
ax1.set_ylabel('Output %')
ax1.set_title('Gamma Correction: Effect on Input-Output Relationship with Dead Zone')
ax1.grid(True)
ax1.set_xlim(0, 100)
ax1.set_ylim(0, 110)  # Kept at 110 to maintain space above the plot

# Add annotation for wasted input range
ax1.annotate('Wasted input range for linear response',
             xy=(20, 20), xytext=(30, 40),
             arrowprops=dict(facecolor='black', shrink=0.05))

# Use the second axis (ax2) for the legend and info box
ax2.axis('off')  # Turn off axis for the info box

# Add legend to ax2
ax1.get_legend_handles_labels()
ax2.legend(*ax1.get_legend_handles_labels(), loc='upper left', bbox_to_anchor=(0, 1), ncol=4)

# Add info text to ax2
info_text = "Steeper curve = more precise control in usable range\n" \
            "Lower gamma allows skipping dead zone with less input"
ax2.text(0.5, 0.5, info_text, ha='center', va='center', bbox=dict(facecolor='white', edgecolor='black', alpha=0.7))

# Adjust layout and save
plt.tight_layout()
plt.savefig('gamma_correction_plot_with_deadzone_adjusted.png', dpi=300, bbox_inches='tight')
plt.close()

print("Plot saved as gamma_correction_plot_with_deadzone_adjusted.png")