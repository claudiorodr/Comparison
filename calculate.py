import integrations


def distance(acceleration_linX, acceleration_linY, acceleration_linZ, deltat):
    """
    Returns the X, Y and Z position
    """
    acceleration_linX = abs(acceleration_linX)
    if -0.05 <= acceleration_linX <= 0.05:
        acceleration_linX = 0.0
        velocityX = integrations.getVelocityX(acceleration_linX, deltat)
    else:
        velocityX = integrations.getVelocityX(acceleration_linX, deltat)

    positionX, distanceX = integrations.getPositionX(
        velocityX, acceleration_linX, deltat)
    return positionX, distanceX  # , positionY, positionZ
