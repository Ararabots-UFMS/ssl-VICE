import univector.un_field as univector
import pytest
from math import pi

######################### gaussian #########################

@pytest.fixture
def gaussian():
    return univector.gaussian 

@pytest.mark.parametrize(
    "x,     mu, sigma,  expected", [
    (3,     4,  5,      0.980198673),           # test random positive integer values
    (3,     -4, 5,      0.37531109885139957),   # test random mixed integer values
    (3.657, -4, 5,      0.3095632084759744),    # test random mixed float values
    (0,     0,  1,      1),                     # standard normal distribution
    (1,     0,  1,      0.6065306597126334),    # 1 standard deviation from the mean
    (-1,    0,  1,      0.6065306597126334),    # -1 standard deviation from the mean
    (0,     0,  2,      1),                     # standard normal distribution with sigma = 2
])
def test_gaussian(gaussian, x, mu, sigma, expected):
    result = gaussian(x, mu, sigma)
    assert abs(result - expected) < 1e-9  # use a small tolerance because of floating point precision

######################### wrap2pi #########################
    
@pytest.fixture
def wrap2pi():
    return univector.wrap2pi

@pytest.mark.parametrize(
    "theta, expected_angle", [
    (5.0,   5.0 - 2 * pi),      #test theta greater than pi
    (-5.0,  2 * pi - 5.0),      #test theta smaller than minus pi
    (2.0,   2.0),               #test theta between pi and minus pi
(pi,    pi),                    #test theta equal to pi
])
def test_wrap2pi(wrap2pi, theta, expected_angle):
    result = wrap2pi(theta)
    assert expected_angle == result

######################### norm #########################

@pytest.mark.parametrize(
    "vec,           expected_norm", [
    ([3.0,  4.0],   5.0),           #test_norm_positive_floats
    ([-5.0, 4.0],   41.0**0.5),     #test_norm_mixed_floats
    ([-1.0, -2.0],  5**0.5),        #test_norm_negative_floats
    ([0.0,  0.0],   0.0)            #test_norm_zero_values)
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