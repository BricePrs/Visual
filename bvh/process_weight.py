#!/usr/bin/env python3

def main():
    # Define the columns you want to keep in the output file
    columns_to_keep = ["hips", "l_hip", "l_thigh", "l_knee", "l_foot", "l_toes", "l_toes_End", "r_hip", "r_thigh", "r_knee", "r_foot", "r_toes", "r_toes_End", "spine1", "spine2", "spine3", "neck", "head", "head_End", "l_clavicle", "l_shoulder", "l_elbow", "l_hand", "l_hand_End", "r_clavicle", "r_shoulder", "r_elbow", "r_hand", "r_hand_End"]

    # Input and output file paths
    input_file_path = 'weights.txt'
    output_file_path = 'modified_weights.txt'

    # Read data from the input file
    with open(input_file_path, 'r') as infile:
        lines = infile.readlines()

    # Extract header and data from the lines
    header = lines[0].split()
    data_lines = [line.split() for line in lines[1:]]

    # Find indices of columns to keep
    columns_to_keep_indices = [header.index(column) for column in columns_to_keep]

    # Write modified data to the output file
    with open(output_file_path, 'w') as outfile:
        # Write the header with selected columns
        outfile.write('id '+' '.join(columns_to_keep) + '\n')

        # Write the modified data lines
        for data_line in data_lines:
            sum_weight = 0
            for i in columns_to_keep_indices:
                sum_weight += float(data_line[i])
            modified_data = [data_line[0]] + [f"{float(data_line[i]) / sum_weight:.5f}" for i in columns_to_keep_indices]
            outfile.write(' '.join(modified_data) + '\n')

    print(f"Modified weights written to {output_file_path}")

if __name__ == "__main__":
    main()
