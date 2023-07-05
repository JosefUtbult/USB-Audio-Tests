import argparse
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='Clock-speed parser',
        description='Helper program for calculating clock speed from samples'
    )

    parser.add_argument('filename')

    args = parser.parse_args()

    # Read the CSV file
    raw_file =  open(args.filename)
    # Split the contents up into a list of lists
    data = [instance.split(',') for instance in raw_file.read().split('\n')]
    # Trim the first and last instances of the file, to get the data
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
        else:
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

    plt.rcParams["font.family"] = "FreeMono"

    fig, ax = plt.subplots()

    # Add a scatter plot of all readings
    ax.scatter(time, delta_time, s=3, c='#b6c3c1')

    # Plot a trendline
    ax.plot(time, p(time), c='#3175ce')


    # Make the diagram go between 0 - 4 ms, as this is the range we
    # are interested in
    # ax.set_ylim(1.6, 2.4)

    # Set labels
    plt.xlabel("Time (ms)")
    plt.ylabel("Clock speed (ms)")
    plt.title("Clock speed over time")

    plt.show()

