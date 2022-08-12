from PID.PID import PID
P = 1
I = 0.0001
D = 0.001

pid = PID(P, I, D)
pid.setSampleTime(0.01)

while(True):
    # Get feeback value
    encoder = 1
    # Set ideal sensor value
    pid.SetPoint = 0.0
    # Update PID loop and get response value
    pid.update(encoder)
    output = pid.output
    print(output)
