import univector.un_field as univector
import pytest
from math import exp, sqrt, pi

######################### gaussian #########################

@pytest.fixture
def gaussian():
    return univector.gaussian 

@pytest.mark.parametrize("x, mu, sigma, expected", [
    (3, 4, 5,       0.980198673),           # standard normal distribution
    (3, -4, 5,      0.37531109885139957),   # 1 standard deviation from the mean
    (3.657, -4, 5,  0.3095632084759744),    # -1 standard deviation from the mean
    (0, 0, 1,       1),                     # standard normal distribution
    (1, 0, 1,       0.6065306597126334),    # 1 standard deviation from the mean
    (-1, 0, 1,      0.6065306597126334),    # -1 standard deviation from the mean
    (0, 0, 2,       1),                     # standard normal distribution with sigma = 2
])
def test_gaussian(gaussian, x, mu, sigma, expected):
    result = gaussian(x, mu, sigma)
    assert abs(result - expected) < 1e-9  # use a small tolerance because of floating point precision

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

@pytest.mark.parametrize("vec, expected_norm", [
    ([3.0,4.0], 5.0),           #test_norm_positive_floats
    ([-5.0, 4.0], 41.0**0.5),   #test_norm_mixed_floats
    ([-1.0, -2.0], 5**0.5),     #test_norm_negative_floats
    ([0.0,0.0], 0.0)             #test_norm_zero_values)
])  
def test_norm(vec, expected_norm):
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