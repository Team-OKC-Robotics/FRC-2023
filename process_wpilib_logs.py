import tkinter
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# all for the left front steer motors
steerEncLog = [[], []]
outputLog = [[], []]
setpointLog = [[], []]
errorLog = [[], []]

with open(r"C:\Users\isasq\Documents\GitHub\FRC-2023\logs\good logs\FRC_20230206_230635.csv") as f:
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
    if (id == '"/swerve/steer_enc"'):
        steerEncLog[0].append(float(timestamp))
        steerEncLog[1].append(float(data))
    elif id == '"/swerve/output"':
        outputLog[0].append(float(timestamp))
        outputLog[1].append(float(data)) # so we can actually see it on the graph
    elif id == '"/swerve/setpoint"':
        setpointLog[0].append(float(timestamp))
        setpointLog[1].append(float(data))

for index, timestamp in enumerate(setpointLog[0]):
    errorLog[0].append(timestamp)
    errorLog[1].append(setpointLog[1][index] - steerEncLog[1][index])

# print(errorLog)

# plt.figure()
# plt.title("just a title")
# plt.plot(outputLog[0], outputLog[1])
# plt.xlabel("Time (sec)")
# plt.ylabel("output (% max)")

# plt.figure()
# plt.plot(rpmLog[0], rpmLog[1])
# plt.xlabel("Time (sec)")
# plt.ylabel("RPM (RPM per 200 ms or something)")

# plt.figure()
# plt.plot(setpointLog[0], setpointLog[1])
# plt.xlabel("Time (sec)")
# plt.ylabel("setpoint (RPM-ish)")

# plt.figure()
# plt.plot(pidLog[0], pidLog[1])
# plt.xlabel("Time (sec)")
# plt.ylabel("pid output (calculated)")
# plt.show()

plt.figure()
plt.plot(steerEncLog[0], steerEncLog[1])
plt.plot(outputLog[0], outputLog[1])
plt.plot(setpointLog[0], setpointLog[1])
plt.plot(errorLog[0], errorLog[1])
plt.xlabel("Time (sec)")
plt.ylabel("data")
plt.legend(("steer enc", "output", "setpoint", "error"))
plt.show()