# import math

# class PController:

#     def __init__(self, kp, max_output=None):
#         self.kp = kp
#         self.max_output = max_output
#         # self.target_orientations = {}


#     def compute(self, target, current):

#         # Calcula o menor erro entre os dois ângulos
#         error = math.atan2(math.sin(target - current), math.cos(target - current))

#         # Aplica o ganho proporcional
#         output = self.kp * error

#         # Limita a saída se um valor máximo for especificado
#         if self.max_output is not None:
#             if output > self.max_output:
#                 output = self.max_output
#             elif output < -self.max_output:
#                 output = -self.max_output

#         return output

import math

class PController:
    """
    Um controlador Proporcional simples para orientação angular.
    Ele lida corretamente com a natureza cíclica dos ângulos.
    """
    def __init__(self, kp, max_output=None):
        self.kp = kp
        self.max_output = max_output

    def compute(self, target, current):
        """
        Calcula a saída do controlador (velocidade angular).
        :param target: A orientação alvo em radianos.
        :param current: A orientação atual em radianos.
        :return: A velocidade angular comandada.
        """
        # Calcula o menor erro entre os dois ângulos
        error = math.atan2(math.sin(target - current), math.cos(target - current))
        
        # Aplica o ganho proporcional
        output = self.kp * error
        
        # Limita a saída se um valor máximo for especificado
        if self.max_output is not None:
            output = max(min(output, self.max_output), -self.max_output)
        
        return output