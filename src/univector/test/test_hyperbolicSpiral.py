import pytest
import univector.un_field as univector
from math import atan2, pi, sqrt
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

def test_fi_h_pos_p_int_radius_cw(hyperbolicalSpiral):
    p = Vec2D(3, 2)
    radius = 4
    kr = 4
    cw = True
    hyperbolicalSpiral = univector.HyperbolicSpiral(kr, radius)

    angulo_esperado = atan2(p[1], p[0])+(pi/2)*sqrt((sqrt(p[0]**2+p[1]**2))/radius)

    angulo_fi_h = hyperbolicalSpiral.fi_h(p, radius, cw)

    assert angulo_esperado == angulo_fi_h

def test_fi_h_negative_integer_point_positive_radius_cw(hyperbolicalSpiral):
    p = Vec2D(-3, 2)
    radius = 4
    kr = 4
    cw = True
    hyperbolicalSpiral = univector.HyperbolicSpiral(kr, radius)

    theta = atan2(p[1], p[0])
    ro = sqrt(p[0]**2+p[1]**2)

    angulo_esperado = theta + (pi/2) * sqrt(ro/radius)

    angulo_fi_h = hyperbolicalSpiral.fi_h(p, radius, cw)

    assert angulo_esperado == angulo_fi_h