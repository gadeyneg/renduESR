import time
import math
import numpy


def theta_s(x, y):
    if x > 0:
        return 1*math.atan(1*y)
    if x <= 0:
        return 1*math.atan(-1*y)


def norm(x):
    res = math.sqrt(math.pow(x[0], 2) + math.pow(x[1], 2))
    return res


def scalaire(u, v):
    res = u[0] * v[0] + u[1] * v[1]
    return res


def angle_oriente(u, v):
    sign = u[0] * v[1] - u[1] * v[0] #produit vectoriel pour obtenir l'orientation de l'angle
    return abs(sign)/sign * math.acos(scalaire(u, v) / (norm(u) * norm(v)))


class OnlineTrainer:
    def __init__(self, robot, NN):

        # Args:
        #     robot (Robot): a robot instance following the pattern of
        #        VrepPioneerSimulation
        #    target (list): the target position [x,y,theta]

        self.robot = robot
        self.network = NN

        self.alpha = [1/6, 1/6, 1/math.pi]  # normalition avec limite du monde cartesien = -3m + 3m

    def train(self, target):
        position = self.robot.get_position()

        network_input = [[0, 0]]
        # network_input[0] = (position[0]-target[0])*self.alpha[0]
        # network_input[1] = (position[1]-target[1])*self.alpha[1]
        # pas nécessaire d'initialiser les valeurs selon moi.
        #Teta_t = 0

        while self.running:
            command = self.network.predict(network_input)
            # on évalue les commandes à appliquer en fonction des entrées

            alpha_x = 1/3
            alpha_y = 1/3
            alpha_teta = 1.0/math.pi

            alpha_speed = 1/3

            self.robot.set_motor_velocity(command[0])  # applique vitesses roues instant t,
            time.sleep(0.050)  # attend delta t
            position = self.robot.get_position()  # obtient nvlle pos robot instant t+1
            position_tourelle = self.robot.get_tourelle_position()
            position_back = self.robot.get_back_position()

            u = [(position_tourelle[0] - position_back[0]) * 50, (position_tourelle[1] - position_back[1]) * 50]
            v = [target[0] - position_tourelle[0], target[1] - position_tourelle[1]]

            theta_actu = position[2]
            theta_voulu = angle_oriente(u, v) #problème : valeur toujours positive, entre 0 et pi

            diff_theta = theta_actu - theta_voulu

            distance = math.sqrt(math.pow((position[0] - target[0]), 2) + math.pow((position[1] - target[1]), 2))

            network_input[0][0] = diff_theta
            network_input[0][1] = distance

                
        self.robot.set_motor_velocity([0,0]) # stop  apres arret  du prog d'app
        # position = self.robot.get_position() #  obtient nvlle pos robot instant t+1
                # Teta_t=position[2]

        self.running = False
