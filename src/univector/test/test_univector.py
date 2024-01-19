import univector.un_field as univector
import pytest
from math import exp, pi

######################### gaussian #########################
def test_gaussian_positive_integers():
    x = 3
    u = 4
    v = 5
    expected_gaussian = 0.980198673

    result_gaussian = univector.gaussian(x, u, v)

    assert expected_gaussian == round(result_gaussian,9)

def test_gaussian_negative_integers():
    x = 3
    u = -4
    v = 5
    expected_gaussian = exp(-((x-u)**2) / (2 * (v**2)))

    result_gaussian = univector.gaussian(x, u, v)

    assert expected_gaussian == result_gaussian

def test_gaussian_mixed_floats():
    x = 3.657
    u = -4
    v = 5
    expected_gaussian = exp(-((x-u)**2) / (2 * (v**2)))

    result_gaussian = univector.gaussian(x, u, v)

    assert expected_gaussian == result_gaussian

######################### wrap2pi #########################
    
def test_wrap2pi_theta_greater_than_pi():
    theta = 5.0
    expected_angle = theta - 2 * pi

    wrap2pi_angle = univector.wrap2pi(theta)

    assert expected_angle == wrap2pi_angle

def test_wrap2pi_theta_smaller_than_minus_pi():
    theta = -5.0
    expected_angle = 2 * pi + theta

    wrap2pi_angle = univector.wrap2pi(theta)

    assert expected_angle == wrap2pi_angle

def test_wrap2pi_theta_between_pi_and_minus_pi():
    theta = 2.0
    expected_angle = theta

    wrap2pi_angle = univector.wrap2pi(theta)

    assert expected_angle == wrap2pi_angle

def test_wrap2pi_theta_equal_to_pi():
    theta = pi
    expected_angle = pi

    wrap2pi_angle = univector.wrap2pi(theta)

    assert expected_angle == wrap2pi_angle

######################### norm #########################
def test_norm_positive_floats():
    #Arrange
    vec = [3.0,4.0]
    expected_norm = 5.0
    #Act
    result_norm = univector.norm(vec)
    #Assert
    assert expected_norm == result_norm

def test_norm_mixed_floats():
    vec = [-5.0, 4.0]
    expected_norm = 41.0**0.5

    result_norm = univector.norm(vec)

    assert expected_norm == result_norm

def test_norm_negative_floats():
    vec = [-1.0, -2.0]
    expected_norm = 5**0.5

    result_norm = univector.norm(vec)

    assert expected_norm == result_norm

def test_norm_zero_values():
    vec = [0.0,0.0]
    expected_norm = 0.0

    result_norm = univector.norm(vec)

    assert expected_norm == result_norm

def test_norm_empty_list():
    with pytest.raises(IndexError):
        univector.norm([])

def test_norm_half_empty_list():
    with pytest.raises(IndexError):
        univector.norm([1.0,])

def test_norm_short_list():
    with pytest.raises(IndexError):
        univector.norm([1.0])