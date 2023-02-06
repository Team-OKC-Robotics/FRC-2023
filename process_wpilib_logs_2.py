import tkinter
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# big list to contain all the individual logs
logs = []

#TODO
# pretend this is a CSV file
log = None

# for each combination in the CSV
for index, line in enumerate(log):
    # if it's a blank line
    if line.strip() == "":
        # skip it
        continue

    # try to extract our data
    try:
        timestamp, id, data = line.split(",")
    # if there's an exception
    except Exception:
        # print it for debugging and keep going
        print(line)
        continue
    
    #TODO do something about actually processing the data

# plot all the data
for log in logs:
    plt.figure()
    plt.plot(log[0], log[1])
    plt.xlabel("time (seconds)")
    plt.ylabel("data (units)")
    
    #TODO
    plt.legend("")

    plt.show()