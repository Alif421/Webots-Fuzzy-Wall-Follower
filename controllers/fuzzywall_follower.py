#Impor Library
from controller import Robot, Motor, DistanceSensor
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

kecepatan_maks = 6.28

robot = Robot()

timestep = int(robot.getBasicTimeStep())

#Enable Proximity Sensor
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5','ps6', 'ps7']
for ind in range(8):
    ps.append(robot.getDistanceSensor(psNames[ind]))
    ps[ind].enable(timestep)

# Inisialisasi motor 
motor_kiri = robot.getMotor('left wheel motor')
motor_kanan = robot.getMotor('right wheel motor')
    
motor_kiri.setPosition(float('inf'))
motor_kiri.setVelocity(0.0)
    
motor_kanan.setPosition(float('inf'))
motor_kanan.setVelocity(0.0)
  
#Definisisikan Rentang Panjang sensor
kiri = ctrl.Antecedent(np.arange(0, 201, 1), 'kiri')
pjkiri = ctrl.Antecedent(np.arange(0, 201, 1), 'pjkiri')
depan = ctrl.Antecedent(np.arange(0, 201, 1), 'depan')
kmotor = ctrl.Consequent(np.arange(0, 61, 1), 'kmotor')
kamotor = ctrl.Consequent(np.arange(0, 61, 1), 'kamotor')
                
kiri['jauh']    = fuzz.trapmf(kiri.universe, [0,0,100,150])
kiri['dekat']   = fuzz.trapmf(kiri.universe, [100,150,200,200])
                
pjkiri['jauh']    = fuzz.trapmf(pjkiri.universe, [0,0,100,150])
pjkiri['dekat']   = fuzz.trapmf(pjkiri.universe, [100,150,200,200])
                
depan['jauh']    = fuzz.trapmf(depan.universe, [0,0,100,150])
depan['dekat']   = fuzz.trapmf(depan.universe, [100,150,200,200])
                
kmotor['lambat'] = fuzz.trapmf(kmotor.universe, [0,10,15,30])
kmotor['sedang']     = fuzz.trapmf(kmotor.universe, [15,30,35,45])
kmotor['cepat']       = fuzz.trapmf(kmotor.universe, [35,45,60,60])
                
kamotor['lambat'] = fuzz.trapmf(kamotor.universe, [0,10,15,30])
kamotor['sedang']     = fuzz.trapmf(kamotor.universe, [15,30,35,45])
kamotor['cepat']       = fuzz.trapmf(kamotor.universe, [35,45,60,60])
                
                
# Rule base
rule1 = ctrl.Rule(kiri['dekat']    & pjkiri['dekat']    & depan['dekat'],    kmotor['cepat'])
rule2 = ctrl.Rule(kiri['dekat']    & pjkiri['dekat']    & depan['jauh'],    kmotor['sedang'])
rule3 = ctrl.Rule(kiri['dekat']    & pjkiri['jauh']    & depan['dekat'],    kmotor['lambat'])
rule4 = ctrl.Rule(kiri['dekat']    & pjkiri['jauh']    & depan['jauh'],    kmotor['lambat'])
rule5 = ctrl.Rule(kiri['jauh']    & pjkiri['dekat']    & depan['dekat'],    kmotor['sedang'])
rule6 = ctrl.Rule(kiri['jauh']    & pjkiri['dekat']    & depan['jauh'],    kmotor['sedang'])
rule7 = ctrl.Rule(kiri['jauh']    & pjkiri['jauh']    & depan['dekat'],    kmotor['sedang'])
rule8 = ctrl.Rule(kiri['jauh']    & pjkiri['jauh']    & depan['jauh'],    kmotor['sedang'])
    
               
rule9 = ctrl.Rule(kiri['dekat']    & pjkiri['dekat']    & depan['dekat'],    kamotor['cepat'])
rule10 = ctrl.Rule(kiri['dekat']    & pjkiri['dekat']    & depan['jauh'],    kamotor['sedang'])
rule11 = ctrl.Rule(kiri['dekat']    & pjkiri['jauh']    & depan['dekat'],    kamotor['lambat'])
rule12 = ctrl.Rule(kiri['dekat']    & pjkiri['jauh']    & depan['jauh'],    kamotor['lambat'])
rule13 = ctrl.Rule(kiri['jauh']    & pjkiri['dekat']    & depan['dekat'],    kamotor['sedang'])
rule14 = ctrl.Rule(kiri['jauh']    & pjkiri['dekat']    & depan['jauh'],    kamotor['sedang'])
rule15 = ctrl.Rule(kiri['jauh']    & pjkiri['jauh']    & depan['dekat'],    kamotor['sedang'])
rule16 = ctrl.Rule(kiri['jauh']    & pjkiri['jauh']    & depan['jauh'],    kamotor['sedang'])
                
        
while robot.step(timestep) != -1:
# Read the sensors:
    nilaiSensor = []
    for ind in range(8):
        nilaiSensor.append(ps[ind].getValue())

    # Output calculation
    kmotor_control = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8])
    kecepatan = ctrl.ControlSystemSimulation(kmotor_control)
        
    kecepatan.input['kiri'] = nilaiSensor[5]
    kecepatan.input['pjkiri'] = nilaiSensor[6]
    kecepatan.input['depan'] = nilaiSensor[7]
    kecepatan.compute() 
        
    kamotor_control = ctrl.ControlSystem([rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16])
    kanankecepatan = ctrl.ControlSystemSimulation(kamotor_control)
        
    kanankecepatan.input['kiri'] = nilaiSensor[5]
    kanankecepatan.input['pjkiri'] = nilaiSensor[6]
    kanankecepatan.input['depan'] = nilaiSensor[7]
    kanankecepatan.compute()
               
    tahu = kecepatan.output['kmotor']
    tempe = kanankecepatan.output['kamotor']
    print(tahu)
    print(tempe)
      
    print("Kiri=", nilaiSensor[5],"pjKiri=", nilaiSensor[6],"Depan=", nilaiSensor[7])
            
    motor_kiri.setVelocity((tahu*6.28)/60)
    motor_kanan.setVelocity((tempe*6.28)/60)