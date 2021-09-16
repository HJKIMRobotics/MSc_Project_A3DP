import os
import io
import numpy as np
import matplotlib.pyplot as plt

# Put your directory files
work_dir = "/home/hyungjoo/output_files"

# How many output do you have?
num_of_output = 129
# Because output.txt file starts 0th
num_of_output += 1

each_output = []
list_of_lists = []
for index in range(0, num_of_output):
    name = "output{index}.txt".format(index=index)
    path = os.path.join(work_dir, name)
    with io.open(path, mode="r", encoding="utf-8") as fd:
        output = fd.read().splitlines()[1:]  # Remove "Maps:" first row
        output_array = np.array(output)
        each_output.append(output_array)
    
    for line in each_output[index]:
        stripped_line = line.strip()
        line_list = stripped_line.split()
        list_of_lists.append(line_list)

# Check the array length and first array        
print(len(list_of_lists[0]))
print(list_of_lists[0])



# If you set the node over 3000, the system will be killed.
# So, please set below than 3000.
op = []
out_op = []   
final_array = []
for node in range(0, 3000):
    for time_step in range(0, num_of_output):
        x = list_of_lists[time_step][node]
        op.append(x)
        out = np.array(op)

    out_op.append(out)
    k = out_op[node][num_of_output*node:]
    z = np.array(k, dtype = np.float32)
    final_array.append(z)
    
# Check the final array
print(final_array)

# Make a x-axis = number of timestep
timesteps = []
for i in range(0, num_of_output):
    timesteps.append(i)


# Select the order of node
ord_node = 30

# Calculate average the probabilites for each node
avg_op_array = np.sum(final_array[ord_node]) / num_of_output
avg_op = [avg_op_array] * num_of_output
avg_op = np.array(avg_op)
print(avg_op_array)

# Make the num of timestep into array
timesteps = []
for i in range(0, num_of_output):
    timesteps.append(i)

# Make a plot
plt.figure(figsize=(20, 8))
plt.xlabel("Number of Timesteps", fontsize=25)
plt.ylabel("Occupancy Probability", fontsize=20)
plt.title("Occupancy Probabilities for each node during Timesteps", fontsize=20)
plt.plot(timesteps, avg_op, label="avg_op", linestyle="--", color="green", linewidth=5.0)
plt.plot(timesteps, final_array[ord_node], label="occupancy probability", color="blue", linewidth=2.0)
plt.show()
    
    
    
    
