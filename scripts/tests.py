import BeamNG_Cruise_Control

def main():
    #Iedarbina serveri un uzstāda, ka testus atkārtos 20 reizes ātrumiem 20, 50, 90 km/h
    beamNG = BeamNG_Cruise_Control.createBeamNG([5.55555555556, 13.8888888889, 25], 20)

    #On_Off tests
    name = "On_Off"
    beamNG.runTest([(0,0,0)], name)

    #P tests, kur P ir no 0.1 - 1 ar 0.05 soli
    p = 0.1
    name = "P"
    while p <= 1:
        beamNG.runTest([(p,0,0)], name)
        p += 0.05

    #BeamNG_Cruise_Control tests, kur P ir 0.8, 0.75, 0.7, 0.45
    #I un D ir intervālā 0 - 0.02 ar 0.005 soli
    best_of_p = [0.8,0.75,0.7,0.45]
    name = "BeamNG_Cruise_Control"
    for p in best_of_p:
    i = 0
    while round(i,2) <= 0.02:
        d = 0
        while round(d,2) <= 0.02:
             beamNG.runTest([(p,i,d)], name)
             d += 0.005
         i += 0.005

    #Aizver serveri
    beamNG.close()
if __name__ == '__main__':
    main()








