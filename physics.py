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

    if v < 0 or density_fluid < 0:
        raise ValueError

    return 9.81 * v * density_fluid


# problem 2
def will_it_float(v, mass):
    """

    determines whether an object will sink or float in water given its volume and mass

    """

    if v < 0 or mass < 0:
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
