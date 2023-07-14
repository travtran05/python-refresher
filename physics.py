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


# problem 8, part 1


def calculate_auv_acceleration(
    f_magnitude, f_angle, mass=100, volume=0.1, thruster_distance=0.5
):
    """

    An AUV is submerged in water. The AUV has a mass of 100kg and a volume of 0.1m^3.
    The AUV is equipped with a thruster that can apply a force up to 100N. The thruster is located
    0.5m from the center of mass of the AUV. The thruster can rotate 30 degrees in either direction of the x-axis.

    calculates the acceleration of the AUV in the 2D plane

    f_magnitude: the magnitude of force applied by the thruster in Newtons
    f_angle: the angle of the force applied by the thruster in radians. The angle is measured from the x-axis.
        Positive angles are measured in the counter-clockwise direction.
    mass: (optional): the mass of the AUV in kilograms. The default value is 100kg.
    volume: (optional): the volume of the AUV in cubic meters. The default value is 0.1m^3
    thurster_distance: (optional): the distance from the center of mass of the AUV to the thruster in meters. The default value is 0.5m

    """
    if thruster_distance < 0 or volume <= 0 or mass <= 0 or f_magnitude < 0:
        raise ValueError
    if f_magnitude > 100:
        raise ValueError
    if f_angle > 30 / 180 * np.pi or f_angle < -30 * np.pi / 180:
        raise ValueError

    force = f_magnitude / np.cos(f_angle)

    return force / mass


# problem 8, part 2


def calculate_auv_angluar_acceleration(
    f_magnitude, f_angle, inertia=1, thruster_distance=0.5
):
    """

    calculates the angular acceleration of the AUV
    f_magnitude: magnitude of force applied by the thruster in Newtons
    f_angle: angle of the force applied by the thruster in radians
    inertia: (optional): the moment of inertia of the AUV in kg*m^2. The default value is 1.
    thruster_distance: optional): the distance from the center of mass of the AUV to the thruster in meters. The default value is 0.5m.

    """
    if f_magnitude > 100:
        raise ValueError
    if f_angle > 30 / 180 * np.pi or f_angle < -30 * np.pi / 180:
        raise ValueError
    if thruster_distance < 0 or inertia <= 0 or f_magnitude < 0:
        raise ValueError

    tau = f_magnitude * np.cos(f_angle) * thruster_distance

    return tau / inertia


# problem 9, part 1


def calculate_auv2_acceleration(T, alpha, theta, mass=100):
    """

    And AUV is submerged in water. The AUV has 4 thrusters fixed to the body of the AUV with and angle alpha.
    Each thruster applies a force Ti to the AUV, where i is the thruster number. The thrusters are
    offset from the center of mass of the AUV by a distance L longitudinally and l laterally, in the robot frame.

    calculates the acceleration of the AUV in the 2D plane.

    T: an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons
    alpha: the angle of the thrusters in radians
    theta: the angle of the auv
    mass: (optional): the mass of the AUV in kilograms. The default value is 100kg

    """

    for i in range(4):
        if T[i] < 0:
            raise ValueError

    if mass <= 0:
        raise ValueError

    rotation_matrix = [[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos[theta]]]

    xycomponents = [
        [np.cos(theta), np.cos(theta), -np.cos(theta), -np.cos(theta)],
        [np.sin(theta), -np.sin(theta), -np.sin(theta), np.sin(theta)],
    ]

    forces = np.matmul(rotation_matrix, np.matmul(xycomponents, T))

    return forces / mass


# problem 9, part 2


def calculate_auv2_angular_acceleration(T, alpha, L, l, inertia=100):
    """

    calculates the angular acceleration of the AUV
    T: an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons
    alpha: the angle of the thrusters in radians
    L: the distance from the center of mass of the AUV to the thrusters in meters
    l: the distance from the center of mass of the AUV to the thrusters in meters
    inertia: (optional): the moment of inertia of the AUV in kg*m^2. The default value is 100

    """

    for i in range(4):
        if T[i] < 0:
            raise ValueError

    if L <= 0 or l <= 0 or inertia <= 0:
        raise ValueError

    return (l * np.sin(alpha) + L * np.cos(alpha)) * (
        (-1) * T[0] + (-1) * T[2] + T[1] + T[3]
    )
