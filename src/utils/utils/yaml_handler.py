import yaml
import sys
from os import environ

# @author Wellington Castro <wvmcastro>

class YamlHandler:
    """ This class is made to both write and read from yml files easier """

    def __init__(self):
        pass
        
    @staticmethod
    def read(file_name, escape = False):
        """ Takes a yml file name and return a dict object with its content """
        dictionary = dict()
        try:
            params_file = open(file_name, "r")
        except IOError:
            params_file = open(environ['ROS_ARARA_ROOT']+"src/" + file_name, "r")

        try:
            dictionary = yaml.safe_load(params_file)
            params_file.close()
        except:
            e = sys.exc_info()[0]
            print("Error: ", e , sys.path[0] + file_name)

        return dictionary
    
    @staticmethod
    def write(dictionary, file_name):
        """ Writes the dictionary content in a yml file """
        sucess = True

        try:
            file = open(file_name, "w+")
        except IOError:
            file = open(environ['ROS_ARARA_ROOT'] + "src/" + file_name, "w+")

        try:
            yaml.dump(dictionary, file, indent=4, sort_keys=True)
            file.close()
        except:
            e = sys.exc_info()[0]
            print("Erro:", e)
            sucess = False

        return sucess