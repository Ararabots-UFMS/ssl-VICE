import pytest
import univector.un_field as univector
from math import atan2, sqrt, sin, cos, pi
from utils.linalg import *

constructor_kr = 5
constructor_radius = 5

@pytest.fixture
def hyperbolicalSpiral():
    return univector.HyperbolicSpiral(constructor_kr, constructor_radius)

######################### Constructor #########################

def test_constructor_kr(hyperbolicalSpiral):
    assert constructor_kr == hyperbolicalSpiral.Kr

def test_constructor_radius(hyperbolicalSpiral):
    assert constructor_radius == hyperbolicalSpiral.radius

######################### update_params #########################

def test_update_params_kr(hyperbolicalSpiral):
    new_kr = 4

    hyperbolicalSpiral.update_params(new_kr, constructor_radius)

    assert new_kr == hyperbolicalSpiral.Kr

def test_update_params_radius(hyperbolicalSpiral):
    new_radius = 4

    hyperbolicalSpiral.update_params(constructor_kr, new_radius)

    assert new_radius == hyperbolicalSpiral.radius


######################### fi_h #########################

@pytest.mark.parametrize(
    "p,             radius,   cw,       expected", [
    (Vec2D(3, 2),   4.0,      True,     2.0793393515833847),       # test random positive integer values
    (Vec2D(-7, 15), 4.0,      True,     -1.7900965452039639),      # test random mixed integer values
    (Vec2D(1, 1),   None,     True,     1.6207941293704042),       # edge case: radius is None  
    (Vec2D(2, 2),   2.0,      False,    -0.9516244389739437),       # edge case: cw is False
    (Vec2D(0, 0),   1.0,      True,     0),                         # edge case: p at the origin
    (Vec2D(1, 0),   0.0,      True,     1.8325957145940457),       # edge case: radius is 0
])
def test_fi_h(hyperbolicalSpiral, p, radius, cw, expected):
    result = hyperbolicalSpiral.fi_h(p, radius, cw)
    assert expected == result

######################### n_h #########################
