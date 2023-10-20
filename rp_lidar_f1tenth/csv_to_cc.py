import csv

# Input CSV file containing lidar data
csv_filename = 'lidar_data.csv'

# Output C++ source file
cc_filename = 'lidar_data.cc'

# Name of the C array
array_name = 'lidar_data'

# Open the CSV file for reading
with open(csv_filename, 'r') as csvfile:
    csvreader = csv.reader(csvfile)

    # Read the CSV data into a list
    data = []
    for i, row in enumerate(csvreader):
        data.append(row)
        if i >= 9:  # Read only the first 10 rows
            break

# Check if the data is non-empty
if not data:
    print("CSV file is empty.")
    exit(1)

# Extract the number of elements and column count
num_elements = len(data)
num_columns = len(data[0])

# Open the C++ source file for writing
with open(cc_filename, 'w') as cfile:
    # Write the C++ code for the array
    cfile.write("#include <lidar_data.h>\n")
    cfile.write("#include <array>\n\n")
    cfile.write("const std::array<std::array<float, {}>, {}> {} = {{\n".format(num_columns, num_elements, array_name))

    for row in data:
        cfile.write("    {{")
        for i, val in enumerate(row):
            cfile.write("{:.6f}".format(float(val)))  # Format as float (adjust as needed)
            if i < num_columns-1:
                cfile.write(", ")
        cfile.write("}},\n")

    cfile.write("};\n")

    # Write the variable for the length of the array
    cfile.write("const int {}_length = {};\n".format(array_name, num_elements))

print(f'First 10 rows of data from {csv_filename} have been converted and saved to {cc_filename}.')

