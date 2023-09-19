import argparse
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter

def generate_delta_time_plot(args, ax):
    # Read the CSV file
    raw_file = open(args.clock_speed_file)
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
    # ax.set_title("Clock speed over time")


def generate_general_plot(filename, ax, format, color, trendline=False, trendline_color=None):
    file = open(filename)
    raw_data = [instance.split(',') for instance in file.read().split('\n')]
    raw_data = raw_data[1:-1]

    resulting_data = []
    timestamp = 0
    msg = ''
    for instance in raw_data:
        if msg == '':
            timestamp = float(instance[0]) * 1000
        if instance[1] != '\\n':
            msg += instance[1]
        else:
            try:
                resulting_data.append([timestamp, format(msg)])
            except ValueError as e:
                print(str(e) + f" at line {len(resulting_data)}")
            msg = ''

    # Create arrays for x and y axis
    size = [instance[1] for instance in resulting_data]
    time = [instance[0] for instance in resulting_data]

    # Calculate a trendline
    z = np.polyfit(time, size, 1)
    p = np.poly1d(z)

    # Add a scatter plot of all readings
    ax.scatter(time, size, s=3, c=color)

    # Plot a trendline
    ax.plot(time, p(time), c=trendline_color if trendline_color is not None else color)


def generate_packet_size_plot(args, ax):
    generate_general_plot(args.packet_size_file, ax, int, '#f4a80b', trendline=True, trendline_color='#efd810')

    # Make the diagram go between 0 - 4 ms, as this is the range we
    # are interested in
    # ax.set_ylim(280, 295)
    # ax.set_ylim(92, 100)
    
    # Set labels
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Packet size")
    # ax.set_title("Packet size over time")


def generate_frame_rate_plot(args, ax):
    generate_general_plot(args.frame_rate_file, ax, float, '#25da81', trendline=True, trendline_color='#199256')
    
    # Make the diagram go between 0 - 4 ms, as this is the range we
    # are interested in
    # ax.set_ylim(280, 295)
    ax.set_ylim(46.5, 49.5)

    # Set labels
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Simulated sample rate (KHz)")
    # ax.set_title("Packet size over time")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='Clock-speed parser',
        description='Helper program for calculating clock speed from samples'
    )

    parser.add_argument('-c', '--clock-speed-file', required=False, help="Optional path to the file that contains the speed of the USB packet clock")

    parser.add_argument('-s', '--packet-size-file', required=False, help="Optional path to file that contains the packet sizes for a sample run")

    parser.add_argument('-f', '--frame-rate-file', required=False, help="Optional path to the file that contains the current simulated rate of the sample clock")

    args = parser.parse_args()

    plt.rcParams["font.family"] = "FreeMono"

    # Make a plot with the specified number of arguments
    plt_size = len([arg for arg in vars(args).items() if arg[1] != None])
    fig, axes = plt.subplots(plt_size)

    index = 0
    # Parse the different plots from their specified file
    if args.clock_speed_file:
        generate_delta_time_plot(args, axes[index] if plt_size > 1 else axes)
        index += 1
    if args.packet_size_file:
        generate_packet_size_plot(args, axes[index] if plt_size > 1 else axes)
        index += 1
    if args.frame_rate_file:
        generate_frame_rate_plot(args, axes[index] if plt_size > 1 else axes)
        index += 1
            
    plt.show()

