import os
import re

def clean_and_split_txt(input_file, base_output_dir):
    """
    Cleans a text file, splits data from each trial into separate files,
    and organizes them into a nested directory structure.

    Args:
        input_file (str): Path to the input text file.
        base_output_dir (str): Base directory to save cleaned trial files.
    """
    # Extract numbers from the filename using regex
    filename = os.path.basename(input_file)
    match = re.match(r"(\d+)pw-(\d+)b\.txt", filename)
    if not match:
        raise ValueError(f"Filename '{filename}' does not match the required format ##pw-##b.txt")

    first_number, second_number = match.groups()

    # Create the nested directory structure
    output_dir = os.path.join(base_output_dir, f"{second_number}b", f"{first_number}pw")
    os.makedirs(output_dir, exist_ok=True)

    # Read the input file
    with open(input_file, 'r') as f:
        lines = f.readlines()

    trial_data = []
    current_trial = []

    for line in lines:
        line = line.strip()  # Remove leading/trailing whitespace

        if line == "==================================":
            if current_trial:
                # Save the previous trial's data if it exists
                trial_data.append(current_trial)
                current_trial = []  # Reset for new trial
        elif line.isdigit() or line.replace('.', '', 1).isdigit():
            # Include the line if it's a number (integer or float)
            current_trial.append(line)

    # Append the last trial's data if any
    if current_trial:
        trial_data.append(current_trial)

    # Write each trial to its own file
    for i, trial in enumerate(trial_data, start=1):
        trial_filename = f"{filename}_trial_{i}.txt"
        output_file = os.path.join(output_dir, trial_filename)
        with open(output_file, 'w') as f:
            f.write('\n'.join(trial))
    
    print(f"Processed {len(trial_data)} trials. Files saved in '{output_dir}'.")

# Example usage
input_file = "40pw-0b.txt"       # Replace with your input file name
base_output_dir = "organized_trials"  # Replace with your desired base output directory
clean_and_split_txt(input_file, base_output_dir)

