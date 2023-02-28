import tkinter
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# all for the left front steer motors
liftEncLog = [[], []]
liftOutputLog = [[], []]
liftSetpointLog = [[], []]

extendEncLog = [[], []]
extendOutputLog = [[], []]
extendSetpointLog = [[], []]

steerEncLog = [[], []]
outputLog = [[], []]
setpointLog = [[], []]
errorLog = [[], []]

with open(r"C:\Users\isasq\Documents\GitHub\FRC-2023\logs\good logs\FRC_20230228_014850.csv") as f:
    log = f.read().split("\n")

for index, line in enumerate(log):
    if line.strip() == "":
        continue
    try:
        timestamp, id, data = line.split(",")
    except Exception:
        print(line)
        continue
    # print(timestamp, id, data)
    if (id == '"/arm/lift_output"'):
        liftEncLog[0].append(float(timestamp))
        liftEncLog[1].append(float(data))
    elif id == '"/arm/lift_enc"':
        liftOutputLog[0].append(float(timestamp))
        liftOutputLog[1].append(float(data)) # so we can actually see it on the graph
    elif id == '"/arm/lift_setpoint"':
        liftSetpointLog[0].append(float(timestamp))
        liftSetpointLog[1].append(float(data))

    if (id == '"/arm/extend_output"'):
        extendOutputLog[0].append(float(timestamp))
        extendOutputLog[1].append(float(data)) # so we can actually see it on the graph
    elif id == '"/arm/extend_enc"':
        extendEncLog[0].append(float(timestamp))
        extendEncLog[1].append(float(data))
    elif id == '"/arm/extend_setpoint"':
        extendSetpointLog[0].append(float(timestamp))
        extendSetpointLog[1].append(float(data))

for index, timestamp in enumerate(setpointLog[0]):
    errorLog[0].append(timestamp)
    errorLog[1].append(setpointLog[1][index] - steerEncLog[1][index])

plt.figure()
plt.plot(liftEncLog[0], liftEncLog[1])
plt.plot(liftOutputLog[0], liftOutputLog[1])
plt.plot(liftSetpointLog[0], liftSetpointLog[1])
plt.xlabel("Time (sec)")
plt.ylabel("data")
plt.legend(("lift enc", "output", "setpoint"))

plt.figure()
plt.plot(extendEncLog[0], extendEncLog[1])
plt.plot(extendOutputLog[0], extendOutputLog[1])
plt.plot(extendSetpointLog[0], extendSetpointLog[1])
plt.xlabel("Time (sec)")
plt.ylabel("data")
plt.legend(("extend enc", "output", "setpoint"))

plt.figure()
plt.plot(steerEncLog[0], steerEncLog[1])
plt.plot(outputLog[0], outputLog[1])
plt.plot(setpointLog[0], setpointLog[1])
plt.plot(errorLog[0], errorLog[1])
plt.xlabel("Time (sec)")
plt.ylabel("data")
plt.legend(("steer enc", "output", "setpoint", "error"))
plt.show()