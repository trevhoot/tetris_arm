from math import *

def jointAngles(x,y,z):
    """Returns joint angles of a STR 17 ARM

    x: int, arm units
    y: int, arm units
    z: int, arm units

    returns: tuple of angles (degrees)
    """
    #Converts from arm tics to mm
    x = x*(355.6/3630)
    y = y*(355.6/3630)
    z =z*(355.6/3630)
    

    a = sqrt(x**2 + y**2)
    b = sqrt(a**2 + z**2)
    print b
    phi = atan2(z,a)
    psy = atan2(a,z)
    theta3 = acos(2 - b**2/375**2)
    chi = (pi - theta3)/2
    print phi*(180/pi)
    print psy*(180/pi)
    print chi*(180/pi)

    theta1 = atan(x/y)*(180/pi)
    theta2 = (chi + phi)*(180/pi)+90
    theta3 = theta3*(180/pi)
    theta4 = (chi + psy)*(180/pi)

    return (theta1,theta2,theta3,theta4)
def main():
    print jointAngles(4000,4000,100)


if __name__ == '__main__':
    main()