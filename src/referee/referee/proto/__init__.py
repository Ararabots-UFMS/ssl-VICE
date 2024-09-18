import os
import sys

# Adicionar todos os arquivos .py no diretório 'proto'
proto_dir = os.path.dirname(__file__)
for file in os.listdir(proto_dir):
    if file.endswith(".py") and file != "__init__.py":
        module_name = file[:-3]
        __import__(module_name)
