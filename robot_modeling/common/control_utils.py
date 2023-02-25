
def clamp_update(t,x,u,params):
    return [0]

def clamp(t,x,u,params):
    # Clamp the control input

    output = u
    if output < -1.0:
        output = -1.0
    elif output > 1.0:
        output = 1.0
    
    return output