import yaml
import sys
from os import environ
from pathlib import Path

# @author Allan Menchik <Menchik>

class YamlHandler:
    """ This class is made to both write and read from yml files easier """

    def __init__(self):
        pass
        
    @staticmethod
    def read(file_path: Path) -> dict:
        '''
            Takes a yml file path and returns a dictionary object with its contents
            
            Args:
                file_path [Path]: Path to .yml file that will be read from.
        '''
        dictionary = dict()
        try:
            params_file = open(file_path, "r")
        except IOError:
            params_file = open(environ['ROS_ARARA_ROOT'] + "src/" + file_path, "r")

        try:
            dictionary = yaml.safe_load(params_file)
            params_file.close()
        except:
            e = sys.exc_info()[0]
            print("Error: ", e , sys.path[0] + file_path)

        return dictionary
    
    @staticmethod
    def write(dictionary: dict, file_path: Path) -> bool:
        '''
            Writes the dictionary's content to a yml file especified by file_path.\n
            Returns True if sucessful, False otherwise.
            Args:
                dictionary [dict]: Dictionary that will be written in the .yml file.
                file_path [Path]: Path to .yml file that will be read from.
        '''
        sucess = True

        try:
            file = open(file_path, "w+")
        except IOError:
            file = open(environ["ROS_ARARA_ROOT"] + "src/" + file_path, "w+")

        try:
            yaml.dump(dictionary, file, indent=4, sort_keys=True)
            file.close()
        except:
            e = sys.exc_info()[0]
            print("Erro:", e)
            sucess = False

        return sucess