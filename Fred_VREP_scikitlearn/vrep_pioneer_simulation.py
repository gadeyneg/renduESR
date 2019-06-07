import vrep
import math


def to_rad(deg):
    return 2*math.pi*deg/360


def to_deg(rad):
    return rad*360/(2*math.pi)


class VrepPioneerSimulation:
    def __init__(self):

        self.ip = '127.0.0.1'
        self.port = 19997
        self.scene = './fred.ttt'
        self.gain = [1, 15]
        self.initial_position = [-5, 0, to_rad(0)]

        self.r = 0.096  # wheel radius
        self.R = 0.267  # demi-distance entre les r


        print('New FRED simulation started')
        vrep.simxFinish(-1)
        self.client_id = vrep.simxStart(self.ip, self.port, True, True, 5000, 5)

        if self.client_id != -1:
            print('Connected to remote API server on %s:%s' % (self.ip, self.port))
            res = vrep.simxLoadScene(self.client_id, self.scene, 1, vrep.simx_opmode_oneshot_wait)
            res, self.fred = vrep.simxGetObjectHandle(self.client_id, 'Fred', vrep.simx_opmode_oneshot_wait)
            res, self.tourelle = vrep.simxGetObjectHandle(self.client_id, 'Plateau', vrep.simx_opmode_oneshot_wait)
            res, self.back = vrep.simxGetObjectHandle(self.client_id, 'Bras_arrire', vrep.simx_opmode_oneshot_wait)
            res, self.direction_motor = vrep.simxGetObjectHandle(self.client_id, 'Direction', vrep.simx_opmode_oneshot_wait)
            res, self.traction_motor = vrep.simxGetObjectHandle(self.client_id, 'Traction', vrep.simx_opmode_oneshot_wait)

            self.set_position(self.initial_position)
            vrep.simxStartSimulation(self.client_id, vrep.simx_opmode_oneshot_wait)

            vrep.simxSetJointTargetVelocity(self.client_id, self.traction_motor, 0,
                                            vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetVelocity(self.client_id, self.direction_motor, 0,
                                            vrep.simx_opmode_oneshot_wait)


        else:
            print('Unable to connect to %s:%s' % (self.ip, self.port))

    def set_position(self, position):
        """Set the position (x,y,theta) of the robot

        Args:
            position (list): the position [x,y,theta]
        """

        vrep.simxSetObjectPosition(self.client_id, self.fred, -1, [position[0], position[1], 0.5], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectOrientation(self.client_id, self.fred, -1, [0, 0, to_deg(position[2])], vrep.simx_opmode_oneshot_wait)

    def get_position(self):
        """Get the position (x,y,joint_theta,theta) of the robot

        Return:
            position (list): the position [x,y,joint_theta,theta]
        """
        position = []
        res, tmp = vrep.simxGetObjectPosition(self.client_id, self.fred, -1, vrep.simx_opmode_oneshot_wait)
        position.append(tmp[0])
        position.append(tmp[1])

        # on ajoute la position actuelle de la jointure de direction, en radiant.
        position.append(vrep.simxGetJointPosition
                        (self.client_id, self.direction_motor, vrep.simx_opmode_oneshot_wait)[1])

        res, tmp = vrep.simxGetObjectOrientation(self.client_id, self.fred, -1, vrep.simx_opmode_oneshot_wait)
        position.append(tmp[2])  # en radian

        return position

    def get_tourelle_position(self):
        position = []
        res, tmp = vrep.simxGetObjectPosition(self.client_id, self.tourelle, -1, vrep.simx_opmode_oneshot_wait)
        position.append(tmp[0])
        position.append(tmp[1])

        return position

    def get_back_position(self):
        position = []
        res, tmp = vrep.simxGetObjectPosition(self.client_id, self.back, -1, vrep.simx_opmode_oneshot_wait)
        position.append(tmp[0])
        position.append(tmp[1])

        return position

    def set_motor_velocity(self, control):
        """Set a target velocity on the pioneer motors, multiplied by the gain
        defined in self.gain. The gain still needs to be modified !!! (different gain for the two joints)

        Args:
            control(list): the control [direction_motor, traction_motor]
        """
        # la version actuelle permet au robot de faire tourner la tourelle directrice sur elle mêe, ce qui n'est pas cohérent avec
        # le modèle (cable à l'arrière). Je décide donc de limiter la position de la jointure de tourelle à -90,+90 °.
        # la méthode actuelle est "tout ou rien", mais j'aimerai tester linéaire, exponetiel ...

        control[0] = -control[0]
        # erreur dans la création de la base d'apprentissage, plus simple de la corriger ici

        current_value = vrep.simxGetJointPosition(self.client_id, self.direction_motor, vrep.simx_opmode_oneshot_wait)[1]
        if control[0] > 0:
            # on veut tourner a gauche
            if current_value > math.pi/3:
                # on est déjà au max à gauche
                vrep.simxSetJointTargetVelocity(self.client_id, self.direction_motor, 0,
                                                vrep.simx_opmode_oneshot_wait)
            else:
                # on peut tourner plus à gauche
                vrep.simxSetJointTargetVelocity(self.client_id, self.direction_motor, self.gain[0] * control[0],
                                                vrep.simx_opmode_oneshot_wait)
        else:
            # on veut tourner à droite
            if current_value < -math.pi/3:
                # on est au max à droite
                vrep.simxSetJointTargetVelocity(self.client_id, self.direction_motor, 0,
                                                vrep.simx_opmode_oneshot_wait)
            else:
                # on peut tourner plus à drotie
                vrep.simxSetJointTargetVelocity(self.client_id, self.direction_motor, self.gain[0] * control[0],
                                                vrep.simx_opmode_oneshot_wait)

        vrep.simxSetJointTargetVelocity(self.client_id, self.traction_motor, - self.gain[1]*control[1],
                                        vrep.simx_opmode_oneshot_wait)
        # print(vrep.simxGetJointPosition(self.client_id, self.direction_motor, vrep.simx_opmode_oneshot_wait))
        # valeur dans [-pi, pi], 0 étant la position de base. à gauche (sens trigo) : valeurs positives.
        # les valeurs positives de setjointtargetvelocity pour la direction font tourner vers la gauche

