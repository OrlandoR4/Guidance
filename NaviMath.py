import math
import DataUtility as dat
import OriMath as ori
from OriMath import radToDeg, degToRad, clamp


def schedule(startTime, endTime, Time):
    if startTime <= Time <= endTime:
        return True
    else:
        return False


class DOF6:
    Dataset = dat.DataRecord("")

    Mass = 0.0
    MMOI = ori.Vector3(0.0, 0.0, 0.0)
    Gravity = ori.Vector3(0.0, 0.0, 0.0)
    Floor = False

    Acceleration = ori.Vector3(0.0, 0.0, 0.0)
    Velocity = ori.Vector3(0.0, 0.0, 0.0)
    Position = ori.Vector3(0.0, 0.0, 0.0)
    Altitude = 0.0

    AngularAcceleration = ori.Vector3(0.0, 0.0, 0.0)
    AngularVelocity = ori.Vector3(0.0, 0.0, 0.0)
    Orientation = ori.Quaternion(1.0, 0.0, 0.0, 0.0)
    EulerAngles = ori.Vector3(0.0, 0.0, 0.0)

    GlobalAcceleration = ori.Vector3(0.0, 0.0, 0.0)

    def setFromEulerAngles(self, x, y, z, mode): # Sets orientation based on euler angles, degrees and radians input modes
        if mode == "deg":
            self.Orientation.eulerToQuaternion(degToRad(x), degToRad(y), degToRad(z))
        elif mode == "rad":
            self.Orientation.eulerToQuaternion(x, y, z)
        else:
            quit(str(self) + "setFromEulerAngles: invalid argument, use valid 'deg' or 'rad'")

    def addForce(self, x, y, z): # Adds forces to the rigidbody based on mass
        self.Acceleration += ori.Vector3(x, y, z)/self.Mass

    def addTorque(self, x, y, z): # Adds torque moments to the object based on mass moment of inertia
        self.AngularAcceleration += ori.Vector3(z, y, x)/ori.Vector3(self.MMOI.z, self.MMOI.y, self.MMOI.x)

    def update(self, dt): # Updates rigidbody and appends data
        self.AngularVelocity += self.AngularAcceleration * dt
        self.Orientation.IMU_ORI(self.AngularVelocity.x, self.AngularVelocity.y, self.AngularVelocity.z, dt)
        self.EulerAngles = self.Orientation.quaternionToEuler()

        self.GlobalAcceleration = self.Orientation.VectorRotate(self.Acceleration.x, self.Acceleration.y, self.Acceleration.z)
        self.GlobalAcceleration += self.Gravity

        self.Velocity += self.GlobalAcceleration * dt
        self.Position += self.Velocity * dt
        self.Altitude = -self.Position.z

        if self.Position.z >= 0.0 and self.Floor:
            self.Velocity.x = 0.0
            self.Velocity.y = 0.0
            self.Velocity.z = 0.0

            self.Position.z = 0.0

        # -------------------------- LOG-DATA --------------------------------
        self.addData("PosX", self.Position.x)
        self.addData("PosY", self.Position.y)
        self.addData("PosZ", self.Position.z)
        self.addData("Altitude", self.Altitude)
        self.addData("VelX", self.Velocity.x)
        self.addData("VelY", self.Velocity.y)
        self.addData("VelZ", self.Velocity.z)
        self.addData("AccX", self.Acceleration.x)
        self.addData("AccY", self.Acceleration.y)
        self.addData("AccZ", self.Acceleration.z)
        self.addData("AccGX", self.GlobalAcceleration.x)
        self.addData("AccGY", self.GlobalAcceleration.y)
        self.addData("AccGZ", self.GlobalAcceleration.z)
        self.addData("Roll", radToDeg(self.EulerAngles.x))
        self.addData("Pitch", radToDeg(self.EulerAngles.y))
        self.addData("Yaw", radToDeg(self.EulerAngles.z))
        self.addData("GyroX", radToDeg(self.AngularVelocity.x))
        self.addData("GyroY", radToDeg(self.AngularVelocity.y))
        self.addData("GyroZ", radToDeg(self.AngularVelocity.z))
        self.addData("AngularAccX", radToDeg(self.AngularAcceleration.x))
        self.addData("AngularAccY", radToDeg(self.AngularAcceleration.y))
        self.addData("AngularAccZ", radToDeg(self.AngularAcceleration.z))

        # ------------------ RESET ------------------
        self.Acceleration = ori.Vector3(0, 0, 0)
        self.AngularAcceleration = ori.Vector3(0, 0, 0)

    def createStandardDataSet(self, sName): # Creates a standard data set to append data to
        self.Dataset.Name = sName
        self.Dataset.createData("Time")
        self.Dataset.createData("PosX")
        self.Dataset.createData("PosY")
        self.Dataset.createData("PosZ")
        self.Dataset.createData("Altitude")
        self.Dataset.createData("VelX")
        self.Dataset.createData("VelY")
        self.Dataset.createData("VelZ")
        self.Dataset.createData("AccX")
        self.Dataset.createData("AccY")
        self.Dataset.createData("AccZ")
        self.Dataset.createData("AccGX")
        self.Dataset.createData("AccGY")
        self.Dataset.createData("AccGZ")
        self.Dataset.createData("Roll")
        self.Dataset.createData("Pitch")
        self.Dataset.createData("Yaw")
        self.Dataset.createData("GyroX")
        self.Dataset.createData("GyroY")
        self.Dataset.createData("GyroZ")
        self.Dataset.createData("AngularAccX")
        self.Dataset.createData("AngularAccY")
        self.Dataset.createData("AngularAccZ")

    def addData(self, NameToFind, Data): # Adds data to the dataset, function referenced from DataUtility
        return self.Dataset.addData(NameToFind, Data)

    def find(self, NameToFind): # Returns list of data from the requested dataset name, function referenced from DataUtility
        return self.Dataset.find(NameToFind)

    def processData(self): # Fills in zeroes for null data, function referenced from DataUtility
        return self.Dataset.processData()


class Simulation:
    Length = 0.0
    Gravity = 9.807
    Time = 0.0
    timeStep = 0.0
    iterations = 0

    def update(self): # Advances simulation in time
        self.iterations += 1
        self.Time += self.timeStep
        print(self.iterations, self.Time, self.timeStep)


class Keyboard:
    Brand = ""
    Model = ""
    Keys = []
    Color = ""
    RGB = False

    def __init__(self, aBrand, aModel, aKeys, aColor, aRGB):
        self.Brand = aBrand
        self.Model = aModel
        self.Keys = aKeys
        self.Color = aColor
        self.RGB = aRGB


class PID:
    kP = 0.0
    kI = 0.0
    kD = 0.0
    Setpoint = 0.0
    SetpointRate = 0.0

    Error = 0
    previousError = 0
    P = 0.0
    I = 0.0
    D = 0.0
    output = 0.0

    def __init__(self, skP, skI, skD):
        self.kP = skP
        self.kI = skI
        self.kD = skD

    def PID(self, Input, dt):
        self.Error = self.Setpoint - Input
        self.P = self.Error
        self.I += self.Error * dt
        self.D = (self.Error - self.previousError) / dt

        self.output = (self.P * self.kP) + (self.I * self.kI) + (self.D * self.kD)

        self.previousError = self.Error

        return self.output

    def changeSetpoint(self, target, dt):
        PIDError = target - self.Setpoint
        PIDSpeed = self.SetpointRate * dt
        PIDError = clamp(PIDError, -PIDSpeed, PIDSpeed)
        self.Setpoint += PIDError
        if timeStep <= 0.0 or self.SetpointRate <= 0:
            self.Setpoint = target

    def reset(self):
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.previousError = 0.0
        self.Error = 0.0


class TVC:
    Angle = 0.0
    Max = 0.0
    Offset = 0.0
    # Multiplier = 0.0 # Scales the output of the TVC, most useful for reversing the output
    Lever = 0.0
    AngleSpeed = 0.0
    SideForce = 0.0
    Torque = 0.0

    def __init__(self, sMax, sOffset, sLever, sAngleSpeed):
        self.Max = sMax
        self.Offset = sOffset
        self.Lever = sLever
        self.AngleSpeed = sAngleSpeed

    def actuate(self, target, dt):
        TVCError = target - self.Angle
        TVCSpeed = self.AngleSpeed * dt
        TVCError = clamp(TVCError, -TVCSpeed, TVCSpeed)
        self.Angle += TVCError
        self.Angle = clamp(self.Angle, -self.Max, self.Max)

    def getTorque(self, Force): # INPUT IN DEG, CONVERTED TO RAD INSIDE FUNCTION
        self.SideForce = Force * degToRad((self.Angle + self.Offset))
        self.Torque = self.SideForce * self.Lever

        return self.Torque


class LowPass:
    highGain = 0.0
    lowGain = 0.0
    filterOut = 0.0
    filterOld = 0.0

    def __init__(self, AhighGain):
        self.highGain = AhighGain

    def Pass(self, Input):
        self.lowGain = 1.0 - self.highGain
        self.filterOut = (Input * self.highGain) + (self.filterOld * self.lowGain)
        self.filterOld = self.filterOut

        return self.filterOut