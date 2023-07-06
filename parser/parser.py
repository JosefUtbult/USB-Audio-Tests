import argparse
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter

def generate_delta_time_plot(args, ax):
    # Read the CSV file
    raw_file = open(args.filename)
    # Split the contents up into a list of lists
    data = [instance.split(',') for instance in raw_file.read().split('\n')]
    # Trim the first and last instances of the file, to get the data

    if int(data[1][1]) % 2 == 0:
        data = data[2:-2]
    else:
        data = data[1:-2]

    for instance in data:
        index = data.index(instance)
        # Check so the format of every instance is correct
        if len(instance) < 2:
            print(f"Error: Instance - {instance}")
        # Make sure the data is osculating between zero and one
        elif int(instance[1]) != (index + 1) % 2:
            print(f"Error: Value - {instance[1]}, Index: {index}")
        # Convert to floats and change to milliseconds
        instance[0] = float(instance[0]) * 1000

    # Calculate the time between clock cycles in milliseconds
    delta_time = []
    for i in range(1, len(data)):
        delta_time.append(data[i][0] - data[i-1][0])

    # Create an array of timing when a sample was made, for the x-axis
    time = [instance[0] for instance in data[1:]]

    # Calculate a trendline
    z = np.polyfit(time, delta_time, 0)
    p = np.poly1d(z)

    # Add a scatter plot of all readings
    ax.scatter(time, delta_time, s=3, c='#3175ce')

    # Plot a trendline
    ax.plot(time, p(time), c='#b6c3c1')

    # Make the diagram go between 0 - 4 ms, as this is the range we
    # are interested in
    # ax.set_ylim(1.6, 2.4)

    # Set labels
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Clock speed (ms)")
    ax.set_title("Clock speed over time")


def generate_packet_size_plot(args, ax):
    packet_size_file = open(args.packet_size_file)
    raw_packet_size_data = [instance.split(',') for instance in packet_size_file.read().split('\n')]
    raw_packet_size_data = raw_packet_size_data[1:-1]

    packet_size_data = []
    timestamp = 0
    msg = ''
    for instance in raw_packet_size_data:
        if msg == '':
            timestamp = float(instance[0]) * 1000
        if instance[1] != '\\n':
            msg += instance[1]
        else:
            packet_size_data.append([timestamp, int(msg)])
            msg = ''

    # Create arrays for x and y axis
    size = [instance[1] for instance in packet_size_data]
    time = [instance[0] for instance in packet_size_data]

    # Calculate a trendline
    z = np.polyfit(time, size, 0)
    p = np.poly1d(z)

    # Add a scatter plot of all readings
    ax.scatter(time, size, s=3, c='#f4a80b')

    # Plot a trendline
    # ax.plot(time, p(time), c='#efd810')

    # Make the diagram go between 0 - 4 ms, as this is the range we
    # are interested in
    ax.set_ylim(280, 295)

    # Set labels
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Packet size")
    ax.set_title("Packet size over time")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='Clock-speed parser',
        description='Helper program for calculating clock speed from samples'
    )

    parser.add_argument('filename')

    parser.add_argument('-s', '--packet_size_file', required=False, help="Optional path to file that contains the packet sizes for a sample run")

    args = parser.parse_args()

    plt.rcParams["font.family"] = "FreeMono"

    # If a packet size file is present, open it and parse every message to get the size of each packet
    if args.packet_size_file:
        fig, axes = plt.subplots(2)
        generate_delta_time_plot(args, axes[0])
        generate_packet_size_plot(args, axes[1])
    else:
        fig, ax = plt.subplots()
        generate_delta_time_plot(args, ax)
            
    plt.show()

