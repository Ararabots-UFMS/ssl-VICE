from utils.yaml_handler import YamlHandler
import pytest

@pytest.fixture
def yaml_handler():
    return YamlHandler()

yaml_file_content = {
    "test": "test",
    "this": "is",
    "a": "test",
}

################### write ###################
def test_write(yaml_handler):
    success = True

    result = yaml_handler.write(yaml_file_content, "./test/test_yaml_handler.yml")
    
    assert success == result

################### read ###################
def test_read(yaml_handler):
    dictionary = yaml_handler.read("./test/test_yaml_handler.yml")

    assert yaml_file_content == dictionary
