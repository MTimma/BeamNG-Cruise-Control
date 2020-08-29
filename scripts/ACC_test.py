import BeamNG_ACC
import time

def main():
    #Initiate testing classes and run BeamNG server instance
    beamNG = BeamNG_ACC.createBeamNG()
    beamNG.runTest()
    beamNG.close()
if __name__ == '__main__':
    main()








