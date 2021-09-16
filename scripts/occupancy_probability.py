import os
import io
import numpy as np
import matplotlib.pyplot as plt


# Put your directory files
work_dir = "/home/hyungjoo/output_files/better_stay"

# How many output do you have?
num_of_output = 81
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

print(len(list_of_lists[30]))
x_o = []
x_ooo = []

# output_files = 127, 130 # better_stay = 73, 75
node = 73    
for time_step in range(0, num_of_output):
    x = list_of_lists[time_step][node]
    x_o.append(x)
    time_step += 1
    x_ooo.append(x_o)

x_o = np.array(x_o, dtype=np.float32)
x_ooo = np.array(x_ooo)
print(x_o)
print(x_o.shape)
print(len(x_o))

timesteps = []
for i in range(0, num_of_output):
    timesteps.append(i)

plt.figure(figsize=(20, 8))
plt.xlabel("Number of Timesteps", fontsize=25)
plt.ylabel("Occupancy Probability", fontsize=20)
plt.title("Occupancy Probabilities for each node during Timesteps", fontsize=20)
# plt.plot(timesteps, avg_op, label="avg_op", linestyle="--", color="green", linewidth=5.0)
plt.plot(timesteps, x_o, label="occupancy probability", color="blue", linewidth=2.0)
plt.show()


