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

with open(r"C:\Users\teamo\Documents\github\FRC-2023\logs\good logs\FRC_20230224_003626.csv") as f:
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
        extendEncLog[0].append(float(timestamp))
        extendEncLog[1].append(float(data))
    elif id == '"/arm/extend_enc"':
        extendOutputLog[0].append(float(timestamp))
        extendOutputLog[1].append(float(data)) # so we can actually see it on the graph
    elif id == '"/arm/extend_setpoint"':
        extendSetpointLog[0].append(float(timestamp))
        extendSetpointLog[1].append(float(data))


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
plt.show()