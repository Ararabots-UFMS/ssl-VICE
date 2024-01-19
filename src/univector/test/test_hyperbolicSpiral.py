import univector.un_field as univector
from math import atan2, pi, sqrt
from utils.linalg import *

######################### Constructor #########################

def test_constructor_kr():
    kr = 5
    radius = 5

    hyperbolicalSpiral = univector.HyperbolicSpiral(kr, radius)

    assert kr == hyperbolicalSpiral.Kr

def test_constructor_radius():
    kr = 5
    radius = 5

    hyperbolicalSpiral = univector.HyperbolicSpiral(kr, radius)

    assert radius == hyperbolicalSpiral.radius

######################### update_params #########################


def test_update_params_kr():
    kr = 5
    radius = 5

    new_kr = 4

    hyperbolicalSpiral = univector.HyperbolicSpiral(kr, radius)

    hyperbolicalSpiral.update_params(new_kr, radius)

    assert new_kr == hyperbolicalSpiral.Kr

def test_update_params_radius():
    kr = 5
    radius = 5

    new_radius = 4

    hyperbolicalSpiral = univector.HyperbolicSpiral(kr, radius)

    hyperbolicalSpiral.update_params(kr, new_radius)

    assert new_radius == hyperbolicalSpiral.radius


######################### fi_h #########################

def test_fi_h_pos_p_int_radius_cw():
    p = Vec2D(3, 2)
    radius = 4
    kr = 4
    cw = True
    hyperbolicalSpiral = univector.HyperbolicSpiral(kr, radius)

    angulo_esperado = atan2(p[1], p[0])+(pi/2)*sqrt((sqrt(p[0]**2+p[1]**2))/radius)

    angulo_fi_h = hyperbolicalSpiral.fi_h(p, radius, cw)

    assert angulo_esperado == angulo_fi_h

def test_fi_h_neg_p_int_radius_cw():
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