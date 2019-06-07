from BackProp_Python_v2 import NN
from vrep_pioneer_simulation import VrepPioneerSimulation
from rdn import Pioneer
# rdn pour ROS avec le pioneer

# import rospy
from online_trainer import OnlineTrainer
import json
import threading

import math
import numpy as np
from sklearn import neural_network

robot = VrepPioneerSimulation()
# robot = Pioneer(rospy)
HL_size = 50
activation = 'logistic'

# number of neurons in hidden layer.
# deux entrées : distance entre la cible et le robot, et différence entre l'angle theta acutel et celui voulu.
# deux sorties : dq/dt et d Theta /dt
network = neural_network.MLPRegressor(hidden_layer_sizes=(HL_size, ), activation=activation)
# network = NN(2, HL_size, 2)
# maintenant 4 entrées, mais toujours deux sorties.

print("Estimateur choisis : MLPRegressor, avec pour paramètre :\n    - HL_size = %d\n    - activation = %s" %(HL_size, activation))

# training base
delta_theta_values = [i / 180 * math.pi for i in np.arange(-180, 190, 10)]
distance_values = [i for i in np.arange(0.1, 3, 0.1)] + [i for i in range(3, 20, 1)]
X_train = []
Y_train = []
for i in delta_theta_values:
    for j in distance_values:
        X_train.append([i, j])
        Y_train.append([
            (2 / (1 + math.exp(-i)) - 1),
            (1 / (1 + abs(i))) / (1 + math.exp(-4 * (j - 1)))
        ])
print("Base d'apprentissage créée, début de l'apprentissage.")
print("NB : cette phase peut prendre quelques minutes selon la base")
# apprentissage
iterations = 3
network.fit(X_train*iterations, Y_train*iterations)

trainer = OnlineTrainer(robot, network)
print("Apprentissage terminé, %d passages effectués." % (iterations))


target = input("Enter the first target : x y --> ")
target = target.split()
for i in range(len(target)):
    target[i] = float(target[i])
print('New target : [%d, %d]' % (target[0], target[1]))

continue_running = True
while(continue_running):

    thread = threading.Thread(target=trainer.train, args=(target,))
    trainer.running = True
    thread.start()

    # Ask for stop running
    input("Press Enter to stop the current training")
    trainer.running = False
    choice = ''
    while choice!= 'y' and choice != 'n':
        choice = input("Do you want to continue ? (y/n) --> ")

    if choice == 'y':
        choice_learning = ''
        while choice_learning != 'y' and choice_learning != 'n':
            choice_learning = input('Do you want to learn ? (y/n) --> ')
        if choice_learning == 'y':
            trainer.training = True
        elif choice_learning == 'n':
            trainer.training = False
        target = input("Move the robot to the initial point and enter the new target : x y radian --> ")
        target = target.split()
        for i in range(len(target)):
            target[i] = float(target[i])
        print('New target : [%d, %d, %d]'%(target[0], target[1], target[2]))
    elif choice == 'n':
        continue_running = False


print("End of simulation")
