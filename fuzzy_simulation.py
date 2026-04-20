import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

# 1. Define input/output universes 
moisture = ctrl.Antecedent(np.arange(0, 101, 1), 'moisture')   # 0–100%
temp     = ctrl.Antecedent(np.arange(10, 46,  1), 'temp')      # 10–45°C
light    = ctrl.Antecedent(np.arange(0, 101, 1), 'light')      # 0–100%

pump     = ctrl.Consequent(np.arange(0, 11, 0.1), 'pump')      # 0–10 seconds
shield   = ctrl.Consequent(np.arange(0, 3.1, 0.1), 'shield')  # 0–3 layers

# 2. Membership functions
# Moisture
moisture['dry']   = fuzz.trapmf(moisture.universe, [0,  0,  25, 45]) # WE MUST CHECK THE VALUES AND INPUT THEM OURSELVES, THESE ARE JUST PLACEHOLDERS!
moisture['moist'] = fuzz.trimf( moisture.universe, [30, 50, 70])  # WE MUST CHECK THE VALUES AND INPUT THEM OURSELVES, THESE ARE JUST PLACEHOLDERS!
moisture['wet']   = fuzz.trapmf(moisture.universe, [55, 75, 100, 100])  # WE MUST CHECK THE VALUES AND INPUT THEM OURSELVES, THESE ARE JUST PLACEHOLDERS!

# Temperature
temp['cool'] = fuzz.trapmf(temp.universe, [10, 10, 18, 25])
temp['warm'] = fuzz.trimf( temp.universe, [18, 27, 36])
temp['hot']  = fuzz.trapmf(temp.universe, [30, 38, 45, 45])

# Light
light['dim']      = fuzz.trapmf(light.universe, [0,  0,  25, 40])
light['moderate'] = fuzz.trimf( light.universe, [30, 50, 70])
light['bright']   = fuzz.trapmf(light.universe, [60, 75, 100, 100])

# Pump duration
pump['none']   = fuzz.trimf(pump.universe, [0,  0,  1])
pump['light']  = fuzz.trimf(pump.universe, [1,  2,  3])
pump['medium'] = fuzz.trimf(pump.universe, [3,  4,  6])
pump['heavy']  = fuzz.trimf(pump.universe, [5,  7,  10])

# Shield layers
shield['none']  = fuzz.trimf(shield.universe, [0,   0,   0.5])
shield['one']   = fuzz.trimf(shield.universe, [0.5, 1,   1.5])
shield['two']   = fuzz.trimf(shield.universe, [1.5, 2,   2.5])
shield['three'] = fuzz.trimf(shield.universe, [2.5, 3,   3])

#  3. Rules
rules = [
    # Pump rules
    ctrl.Rule(moisture['dry']   & temp['hot'],        pump['heavy']),
    ctrl.Rule(moisture['dry']   & temp['warm'],       pump['medium']),
    ctrl.Rule(moisture['dry']   & temp['cool'],       pump['light']),
    ctrl.Rule(moisture['moist'] & temp['hot'],        pump['light']),
    ctrl.Rule(moisture['moist'] & (temp['warm'] | temp['cool']), pump['none']),
    ctrl.Rule(moisture['wet'],                        pump['none']),

    # Shield rules
    ctrl.Rule(light['bright'] & temp['hot'],          shield['three']),
    ctrl.Rule(light['bright'] & (temp['warm'] | temp['cool']), shield['two']),
    ctrl.Rule(light['moderate'],                      shield['one']),
    ctrl.Rule(light['dim'],                           shield['none']),
]

# 4. Build system & simulate 
system = ctrl.ControlSystem(rules)
sim    = ctrl.ControlSystemSimulation(system)

# Test with example sensor readings
sim.input['moisture'] = 25    # fairly dry // THESE ARE JUST TEMPORARY TEST VALUES
sim.input['temp']     = 36    # hot // THESE ARE JUST TEMPORARY TEST VALUES
sim.input['light']    = 80    # bright // THESE ARE JUST TEMPORARY TEST VALUES

sim.compute()

print(f"Pump duration : {sim.output['pump']:.2f} seconds")
print(f"Shield layers : {round(sim.output['shield'])} layers")

# 5. Visualize membership functions (very useful for tuning)
temp.view()
light.view()
pump.view(sim=sim)
shield.view(sim=sim)
plt.show()


# !!! This is not the finished code !!!