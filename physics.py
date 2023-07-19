import numpy as np
import matplotlib.pyplot as plt

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
        print("1")
        raise ValueError
    if f_magnitude > 100:
        print("2")
        raise ValueError
    if f_angle > 30 / 180 * np.pi or f_angle < -30 * np.pi / 180:
        print("4")
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

    rotation_matrix = [[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]]

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

    signs_matrix = np.array([1, -1, 1, -1])
    r = np.sqrt(np.power(L,2) + np.power(l, 2))
    torque = np.sum(np.multiply(signs_matrix, T) * (L*np.sin(alpha) + l*np.cos(alpha)))
    #torque = np.dot(signs_matrix,T)
    angular_acceleration = torque / inertia

    return angular_acceleration

# problem 10, part 1

def simulate_auv2_motion(T, alpha, L, l, mass=100, inertia=100, dt=0.1, t_final=10, x0=0, y0=0, theta0=0):
    '''
    The AUV should start at the origin with an initial velocity of 0m/s
    The AUV should be able to move in any direction.
    The AUV should be able to rotate in either direction.
    The AUV should be able to move and rotate simultaneously.

    simulates the motion of the AUV in the 2D plane.

    The function should take the following arguments:
    T: an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons.
    alpha: the angle of the thrusters in radians.
    L: the distance from the center of mass of the AUV to the thrusters in meters.
    l: the distance from the center of mass of the AUV to the thrusters in meters.
    inertia (optional): the moment of inertia of the AUV in kg*m^2
    The default value is 100.
    dt (optional): the time step of the simulation in seconds. The default value is 0.1s
    t_final (optional): the final time of the simulation in seconds. The default value is 10s
    x0 (optional): the initial x-position of the AUV in meters. The default value is 0m
    y0 (optional): the initial y-position of the AUV in meters. The default value is 0m
    theta0 (optional): the initial angle of the AUV in radians. The default value is 0rad

    The function should return the following:
    t: an np.ndarray of the time steps of the simulation in seconds.
    x: an np.ndarray of the x-positions of the AUV in meters.
    y: an np.ndarray of the y-positions of the AUV in meters.
    theta: an np.ndarray of the angles of the AUV in radians.
    v: an np.ndarray of the velocities of the AUV in meters per second.
    omega: an np.ndarray of the angular velocities of the AUV in radians per second.
    a: an np.ndarray of the accelerations of the AUV in meters per second squared.
    '''

    for i in range(4):
        if T[i] < 0:
            raise ValueError

    if L<0 or l<0 or inertia<0 or dt<0 or t_final<0:
        raise ValueError

    desired_shape = (4,)

    if T.shape != desired_shape:
        raise ValueError    


    t = np.arange(0, t_final, dt)
    x = np.zeros_like(t)
    y= np.zeros_like(t)
    theta = np.zeros_like(t)
    v = np.zeros_like(t)
    omega = np.zeros_like(t)
    a = np.zeros_like(t)

    x[0] = x0
    y[0] = y0
    theta[0] = theta0

    angular_acceleration = calculate_auv2_angular_acceleration(T, alpha, L, l)

    for i in range(1, len(t)):
        x[i] = x[i-1]+v[i]*dt*np.cos(theta[i]) 
        y[i] = y[i-1]+v[i]*dt*np.sin(theta[i])
        theta[i] = theta[i-1]+dt*omega[i]
        omega[i] = omega[i-1] + angular_acceleration * dt
        current_a = calculate_auv2_acceleration(T, alpha, theta[i], mass)
        ax = current_a[0]
        ay = current_a[1]
        a[i] = np.sqrt(np.power(ax,2) + np.power(ay, 2))
        v[i] = v[i-1] + a[i] * dt

    return t,x,y,theta,v,omega,a 


def plot_auv2_motion(t,x,y,theta,v,omega,a):
    plt.plot(t, x, label="X Position")
    plt.plot(t, y, label="Y Position")
    plt.plot(t, theta, label="Angle")
    plt.plot(t, omega, label="Angular velocity")
    plt.plot(t, v, label="Velocity")
    plt.plot(t, a, label="Acceleration")
    plt.xlabel("Time (s)")
    plt.ylabel("X Position (m), Y Position (m), Angle (radians), Angular Velocity (radians/s^2), Velocity (m/s), Acceleration (m/s^2)")
    plt.legend()
    plt.show()

def plot_auv2_motion_xy(t,x,y,theta,v,omega,a):
    #plt.plot(t, x, label="X Position")
    #plt.plot(t, y, label="Y Position")
    #plt.plot(t, theta, label="Angle")
    #plt.plot(t, omega, label="Angular velocity")
    #plt.plot(t, v, label="Velocity")
    #plt.plot(t, a, label="Acceleration")
    plt.plot(x, y, label="xy graph")
    plt.xlabel("Time (s)")
    plt.ylabel("X Position (m), Y Position (m), Angle (radians), Angular Velocity (radians/s^2), Velocity (m/s), Acceleration (m/s^2)")
    plt.legend()
    plt.show()



                        