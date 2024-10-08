input_file = 'f_dataset-MH01_monoldso.txt'
output_file = 'f_dataset-MH01_monoldso_seconds.txt'

with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
    for line in infile:
        parts = line.split()
        if parts:
            timestamp = float(parts[0]) * 1e9  # Convert nanoseconds to seconds
            new_line = f"{timestamp:.9f} " + " ".join(parts[1:]) + "\n"
            outfile.write(new_line)