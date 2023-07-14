import numpy as np

"""

created by travis tran on July 13, 2023

"""


# problem 1
def calculate_buoyancy(v, density_fluid):
    """

    v is the volume of the object in cubic meters
    density_fluid is the density of the fluid in kg/m^3
    g is given to be 9.81m/s^2

    function returns buoyancy force in newtons given the volume of the object and the density of the fluid it is submerged in

    """

    if v <= 0 or density_fluid <= 0:
        raise ValueError

    return 9.81 * v * density_fluid


# problem 2
def will_it_float(v, mass):
    """

    determines whether an object will sink or float in water given its volume and mass

    """

    if v <= 0 or mass <= 0:
        raise ValueError

    if calculate_buoyancy(v, 1000) > 9.81 * mass:
        return True
    return False


# problem 3
def calculate_pressure(depth):
    """

    calculates the pressure (in pascals) at a given depth in water
    depth is in meters

    """
    if depth < 0:
        raise ValueError

    return 1000 * 9.81 * depth + 101325


# problem 4


def calculate_acceleration(force, mass):
    """

    calculates the acceleration of an object given the force applied to it and the mass
    force: newtons
    mass: kilograms

    """
    if mass <= 0:
        raise ValueError
    return force / mass


# problem 5


def calculate_angular_acceleration(tau, i):
    """

    calculates the angular acceleration of an object given the torque applied to it and its moment of inertia
    tau: the torque applied to the object in Newton-meters
    i: the moment of inertia of the object in kg*m^2

    """

    if i <= 0:
        raise ValueError

    return tau / i


# problem 6


def calculate_torque(f_magnitude, f_direction, r):
    """

    calculates the torque applied to an object given the force applied to it and the distance from
    the axis of rotation to the point where the force is applied
    f_magnitude: the magnitude of force applied to the object in Newtons
    f_direction: the direction of the force applied to the object in degrees
    r: the distance from the axis of rotation to the point where the force is applied in meters

    """

    if r <= 0 or f_magnitude <= 0:
        raise ValueError

    return r * f_magnitude * np.cos(f_direction * np.pi / 180)


# problem 7


def calculate_moment_of_inertia(mass, r):
    """

    calculates the moment of inertia of an object given its mass and the
    distance from the axis of rotation to the center of mass of the object
    mass: mass of the object in kilograms
    r: the distance from the axis of rotation to the center of mass of the object in meters

    """

    if r <= 0 or mass <= 0:
        raise ValueError

    return mass * r * r


# problem 8
