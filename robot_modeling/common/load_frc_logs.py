
import numpy as np

def load_sysid_logs(log_filename, sysid_data):
    with open(log_filename) as f:
        log = f.read().split("\n")

    for index, line in enumerate(log):
        if line.strip() == "":
            continue
        try:
            timestamp, id, data = line.split(",")
        except Exception:
            print(line)
            continue
        
        try:
            if id in set(sysid_data.keys()):
                sysid_data[id][0].append(float(timestamp))
                sysid_data[id][1].append(float(data))
        except Exception:
            continue

    return sysid_data
