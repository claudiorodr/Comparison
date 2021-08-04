# Initializing the global variables
counter = 0
positX = 0  # Position in the X-Axis
coordinateX = 0
coordinateY = 0
coordinateZ = 0
velX = 0  # Velocity on the X-Axis
deltat = 0  # Delta T
accelX = 0  # Acceleration on the X-Axis
dispX = 0  # displacement on the X-Axis
ax = 0
positY = 0  # Position in the X-Axis
velY = 0  # Velocity on the X-Axis
accelY = 0  # Acceleration on the X-Axis
dispY = 0  # displacement on the X-Axis
ay = 0
positZ = 0  # Position in the X-Axis
velZ = 0  # Velocity on the X-Axis
accelZ = 0  # Acceleration on the X-Axis
dispZ = 0  # displacement on the X-Axis
az = 0


def updateCoordinates(coordX, coordY, coordZ):
    global coordinateX
    global coordinateY
    global coordinateZ
    coordinateX = coordX
    coordinateY = coordY
    coordinateZ = coordZ


def getCoordinate():
    global coordinateX
    global coordinateY
    global coordinateZ
    return coordinateX, coordinateY, coordinateZ


# calculating position on X-axis derived from velocity
def getPositionX(velX, accelX, deltat):
    global positX
    positX += (velX * deltat)
    distanceX = (velX * deltat)
    return positX, distanceX


# calculating velocity on X-axis derived from acceleration
def getVelocityX(accelX, deltat):
    global velX

    if accelX == 0.0:
        velX *= 0.55
    #  if (counter == 10):
    #    counter = 0
    #    velX = 0
    #
    #  elif (velX == velX + accelX*deltat):
    #    counter++;
    else:
        velX += accelX * deltat
    return velX


# calculating position on Y-axis derived from velocity
def getPositionY(velY, accelY, deltat):
    global positY
    positY += (velY * deltat) + (0.5 * accelY * (deltat ** 2))
    return positY


# calculating velocity on X-axis derived from acceleration
def getVelocityY(accelY, deltat):
    global velY

    if accelY == 0.0:
        velY *= 0.55
    #  if (counter == 10):
    #    counter = 0
    #    velX = 0
    #
    #  elif (velX == velX + accelX*deltat):
    #    counter++;
    else:
        velY += accelY * deltat
    return velY


# calculating position on Y-axis derived from velocity
def getPositionZ(velZ, accelZ, deltat):
    global positZ
    positZ += (velZ * deltat) + (0.5 * accelZ * (deltat ** 2))
    return positZ


# calculating velocity on Z-axis derived from acceleration
def getVelocityZ(accelZ, deltat):
    global velZ

    if accelZ == 0.0:
        velZ *= 0.8
    #  if (counter == 10):
    #    counter = 0
    #    velX = 0
    #
    #  elif (velX == velX + accelX*deltat):
    #    counter++;
    else:
        velZ += accelZ * deltat
    return velZ


# Calculating displacement on X-axis derived from the position
def getDisplacement(velX, positX):
    global dispX
    if velX != 0.00 and positX != 0.00:
        dispX += positX
        return dispX


def integrate():
    #  velX = 0.00
    accgToms2()

    #  remove noise, keep the sensor steady
    #  if (accelX <= 0.25 && accelX >= -0.25) accelX = 0.00;
    #  if (velX <= 0.01 && velX >= -0.01) velX = 0.00;
    #  if (velX == 0.00 && accelX == 0.00) positX = 0.00;

    getPositionX(positX, deltat)
    getVelocityX(velX, deltat)
    getDisplacementX(distX, deltat)

    prevTime = actTime

    print("deltat: " + deltat)
    print("accelX: " + accelX)
    print("ax: " + ax)
    print("velX: " + velX)
    print("positX: " + positX)
    print("distX: " + distX)


# Calculate the acceleration value into actual g's
def accToG(data):
    # get actual g value, this depends on scale being set
    ax = data[0] * aRes - accelBias[0]
    ay = data[1] * aRes - accelBias[1]
    az = data[2] * aRes - accelBias[2]


def accgToms2():
    # converting from g's to m/s squared
    # 9.78422 is gravity in Funchal at the height of Tecnopolo is 161m
    gravity = 9.81
    accelX = trunc(ax * gravity * 10) / 10
    accelY = ay * gravity
    accelZ = az * gravity


def gyroToG():
    # // Calculate the gyro value into actual degrees per second
    # // get actual gyro value, this depends on scale being set
    gx = MPU9250Data[4] * gRes
    gy = MPU9250Data[5] * gRes
    gz = MPU9250Data[6] * gRes
