import math

from DataUtility import Data, DataRecord

from OriMath import Vector3, Quaternion
from OriMath import radToDeg, degToRad, clamp


def schedule(startTime, endTime, Time):
    '''
        Returns true if the time is within the two given times
    '''

    if startTime <= Time <= endTime:
        return True
    else:
        return False


'''
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                                                    ALL UNITS ARE SI (METER, KILOGRAM, SECOND)
                                                    
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
'''


class DOF6:
    '''
                          DOF6: 6 Degrees of Freedom object, simulates position, orientation, etc. of a rigid body

        :Mass: Mass of the body, kg
        :MMOI: Vector3 containing the Mass Moment Of Inertia at the center of mass of the body along its xyz axes, kg*m^2
        :Gravity: Vector3 containing the gravitational acceleration of the body, m/s^2
        :Floor: When true, zeroes the velocity and the x-axis position (altitude) of the body when at less than 0 meters of altitude

        :Acceleration: Acceleration of the body in the body-frame, m/s^2
        :AngularAcceleration: Angular acceleration of the body in the body-frame, rad/s^2

        :AngularVelocity: Angular velocity of the body in the body frame, rad/s

        :Orientation: Global orientation quaternion of the body, derived from the angular velocity of the object
        :EulerAngles: Euler angle representation of the orientation quaternion (Z-Y-X order of rotations) X = Roll, Y = Pitch, Z = Yaw, rad

        :GlobalAcceleration: Global acceleration of the body, derived from the orientation quaternion and the body-frame acceleration
        :Velocity: Global velocity of the body
        :Position: Global position of the body

        :Dataset: Contains data of the 6DOF object, position, orientation, among others. It can accept new data headers as the user needs


        __init__: Assigns a name to the body, and creates a basic dataset to log basic data

        setFromEulerAngles: Sets orientation based on euler angles, degrees and radians input modes
        ^ !!! It appears it wont set the angles exactly but close enough for testing, needs checking !!!

        addForce: Applies a force to the body at the center of mass, N
        addTorque: Applis a torque to the body at the center of mass, N*m

        update: Updates rigidbody and appends data, dt: time step, delta time

        addData: Adds data to the dataset, function referenced from DataUtility
        find: Returns a list of data from the requested dataset name, function referenced from DataUtility
        processData: Fills in zeroes for null data, function referenced from DataUtility
    '''

    Name = ""
    Dataset = DataRecord("")

    Mass = 0.0
    MMOI = Vector3(0.0, 0.0, 0.0)
    Gravity = Vector3(0.0, 0.0, 0.0)
    Floor = False

    Acceleration = Vector3(0.0, 0.0, 0.0)
    AngularAcceleration = Vector3(0.0, 0.0, 0.0)
    AngularVelocity = Vector3(0.0, 0.0, 0.0)

    Orientation = Quaternion(1.0, 0.0, 0.0, 0.0)
    EulerAngles = Vector3(0.0, 0.0, 0.0)

    GlobalAcceleration = Vector3(0.0, 0.0, 0.0)
    Velocity = Vector3(0.0, 0.0, 0.0)
    Position = Vector3(0.0, 0.0, 0.0)

    def __init__(self, sName):
        self.Name = sName

        self.Dataset.Name = self.Name + "'s Data"
        self.Dataset.createData("Time")
        self.Dataset.createData("PosX")
        self.Dataset.createData("PosY")
        self.Dataset.createData("PosZ")
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

    def setFromEulerAngles(self, x, y, z, mode):
        if mode == "deg":
            self.Orientation.eulerToQuaternion(degToRad(x), degToRad(y), degToRad(z))
        elif mode == "rad":
            self.Orientation.eulerToQuaternion(x, y, z)
        else:
            quit(str(self) + "setFromEulerAngles: invalid argument, use valid 'deg' or 'rad'")

    def addForce(self, x, y, z):
        self.Acceleration += Vector3(x, y, z)/self.Mass

    def addTorque(self, x, y, z):
        self.AngularAcceleration += Vector3(x, y, z)/Vector3(self.MMOI.x, self.MMOI.y, self.MMOI.z)

    def update(self, dt):
        self.AngularVelocity += self.AngularAcceleration * dt

        self.Orientation.IMU_ORI(self.AngularVelocity.x, self.AngularVelocity.y, self.AngularVelocity.z, dt)
        self.EulerAngles = self.Orientation.quaternionToEuler()

        self.GlobalAcceleration = self.Orientation.VectorRotate(self.Acceleration.x, self.Acceleration.y, self.Acceleration.z)
        self.GlobalAcceleration += self.Gravity

        self.Velocity += self.GlobalAcceleration * dt
        self.Position += self.Velocity * dt

        if self.Position.x < 0.0 and self.Floor:
            self.Velocity.x = 0.0
            self.Velocity.y = 0.0
            self.Velocity.z = 0.0

            self.Position.x = 0.0

        '''
                                         !!!!!!!!!!IMPORTANT!!!!!!!!!!
                            Angular rates and everything orientation is logged as degrees
                               by default, feel free to remove the radtodeg conversion
        '''

        self.addData("PosX", self.Position.x)
        self.addData("PosY", self.Position.y)
        self.addData("PosZ", self.Position.z)
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
        self.Acceleration = Vector3(0, 0, 0)
        self.AngularAcceleration = Vector3(0, 0, 0)

    # You can directly call the Dataset property of DOF6, but these functions will make the code much cleaner, the same functions can be used for the Dataset object

    def addData(self, NameToFind, Data):
        return self.Dataset.addData(NameToFind, Data)

    def find(self, NameToFind):
        return self.Dataset.find(NameToFind)

    def processData(self):
        return self.Dataset.processData()


class Simulation:
    '''
                                 Simulation: Object to store simulation parameters

        :Length: Length the simulation will be run for, seconds
        :Gravity: Standard gravity value, m/s^2
        :Time: Current time of the simulation, seconds
        :timeStep: dt, time each 'snapshot' of the simulation lasts, seconds
        :iterations: Iterations the sim has executed, a 30 second sim with a 1 second dt would run for 30 iterations

        update: Advances simulation in time
    '''

    Length = 0.0
    Gravity = 9.807
    Time = 0.0
    timeStep = 0.0
    iterations = 0

    def update(self):
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
        self.RGB = aRGB # dont ask


class PID:
    '''
                                 PID: Proportional Integral Derivative controller, very easy to use and tune

        :kP, kI, kD: Respective gains for P, I, and D
        :Setpoint: Target for the PID
        :SetpointRate: Rate the setpoint can change, you can directly assign a value to self.Setpoint for an instant change

        :output: Output of the PID controller

        __init__: Assign gains to the controller

        PID: Compute PID output, input is the state you want to control, dt is change in time, delta time. Returns the output of the PID controller
        changeSetpoint: Function to change the setpoint gradually, target is the setpoint to change to, dt is delta time.
        reset: Resets PID values
    '''
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
    '''
                                       TVC: 1-Axis thrust vector control object, acts on the body frame of a body, radians

        :Max: Maximum angle for the TVC, a maximum angle means a maximum TVC deflection of +Max to -Max, so the range of actuation is 2 * Max
        :Offset: Offset for the TVC, simulating misalignment, TVC angle might be zero but the offset will be accounted for in the torque calculations
        :Lever: Lever arm the torque will be applied from, also known as the distance from the center of mass to the actuation point of the TVC
        :AngleSpeed: Maximum speed the TVC can actuate its angle by, if it is 1rad/s, means that the TVC will only be able to change its angle by 1 rad every second

        :Angle: Angle of the TVC, radians
        :SideForce: Side force the TVC will produce given the deflection angle, and a thrust
        :Torque: Torque the TVC is creating given the side force and lever arm, N*m

        __init__: Assigns basic parameters for the TVC described above

        actuate: Actuate the TVC given a target (where to point the TVC at), and dt, time step, change in time, delta time
        getTorque: Returns the torque created from the TVC given a thrust, N*m
    '''

    Max = 0.0
    Offset = 0.0
    Lever = 0.0
    AngleSpeed = 0.0

    Angle = 0.0
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

    def getTorque(self, Force):
        self.SideForce = Force * (self.Angle + self.Offset)
        self.Torque = self.SideForce * self.Lever

        return self.Torque


class LowPass: # Dont use, untested, low pass filter
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