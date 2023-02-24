import tkinter
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# big dict to contain all the individual logs
logs = {}

# working log specification that I made up one night:
# logs = {
#   "swerve/left-front":[
#       ["setpoint", "output", "sensor-reading"]
#       [[timestamps], [data]],
#       [[timestamps], [data]],
#       # so on so forth
#   ],
#   "arm/lift":[
#       ["setpoint", "output"]
#       [[timestamps], [data]]
#   ],
#    "arm/extend":[
#        ["setpoint", "output"],
#        [[timestamps], [data]]
#        [[timestamps], [data]]
#    ]
# }

#TODO
# pretend this is a CSV file
log = None
with open(path, "r") as f:
    log = path.read().split("\n")

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
    
    try:
        # split the name of the data log entry into its component parts
        subsystem, specific_part, data_name = id.split("/")
        
        # if the name of the specific datam is in the legend already
        if data_name in logs[subsystem + "/" + specific_part][0]:
            
            # get its index, and that's what we append the data to
            idx = logs[subsystem + "/" + specific_part].index()
        else:
            # append specific data name to legend
            logs[subsystem + "/" + specific_part][0].append(data_name)
            
            # and the index is then what we just appended, so len() - 1 I think
            idx = len(logs[subsystem + "/" + specific_part][0]) - 1
    # if there's an exception
    except Exception as e:
        # print the exception that was raised
        print(e)
        
        # print the offending line
        print(line)

        # keep going through the log
        continue
    
    # so then append the data to the specific list
    logs[subsystem + "/" + specific_part][idx][0].append(timestamp)
    logs[subsystem + "/" + specific_part][idx][1].append(data)

    # and keep going


# plot all the data
for key, data in logs:
    plt.figure()
    plt.plot(log[0], log[1])
    plt.xlabel("time (seconds)")
    plt.ylabel("data (units)")
    
    #TODO
    plt.legend("")

    plt.show()