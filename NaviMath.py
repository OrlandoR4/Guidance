import math

import DataUtility as dat

import OriMath as ori
from OriMath import radToDeg, degToRad, clamp


def schedule(startTime, endTime, Time): # Returns true if the time is within the two given times
    if startTime <= Time <= endTime:
        return True
    else:
        return False


'''
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                                                    ALL UNITS ARE SI (METER, KILOGRAM, SECOND)
                                                    
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
'''


class DOF6: # 6 Degrees of Freedom object, simulates position, orientation, etc.
    Dataset = dat.DataRecord("") # Dataset to store data in

    Mass = 0.0 # Mass of the body
    MMOI = ori.Vector3(0.0, 0.0, 0.0) # Mass Moment Of Inertia
    Gravity = ori.Vector3(0.0, 0.0, 0.0) # Applied gravitational acceleration
    Floor = False # Determines if body should not go below x = 0 (or the ground), also zeroes out xyz velocities. Prevents x body position from going below zero if True.

    Acceleration = ori.Vector3(0.0, 0.0, 0.0) # Body-frame acceleration
    AngularAcceleration = ori.Vector3(0.0, 0.0, 0.0) # Body-frame angular acceleration

    AngularVelocity = ori.Vector3(0.0, 0.0, 0.0) # Body-frame angular velocity
    Orientation = ori.Quaternion(1.0, 0.0, 0.0, 0.0) # Global orientation of the body
    EulerAngles = ori.Vector3(0.0, 0.0, 0.0) # Global Euler Angles, converted from the Orientation quaternion

    GlobalAcceleration = ori.Vector3(0.0, 0.0, 0.0) # Global acceleration, calculated from Orientation quaternion and body acceleration
    Velocity = ori.Vector3(0.0, 0.0, 0.0) # Global Velocity
    Position = ori.Vector3(0.0, 0.0, 0.0) # Global Position

    def setFromEulerAngles(self, x, y, z, mode): # Sets orientation based on euler angles, degrees and radians input modes (It appears it wont set the angles exactly but close enough, needs checking)
        if mode == "deg":
            self.Orientation.eulerToQuaternion(degToRad(x), degToRad(y), degToRad(z))
        elif mode == "rad":
            self.Orientation.eulerToQuaternion(x, y, z)
        else:
            quit(str(self) + "setFromEulerAngles: invalid argument, use valid 'deg' or 'rad'")

    def addForce(self, x, y, z): # Adds forces to the rigidbody based on mass (In Newtons)
        self.Acceleration += ori.Vector3(x, y, z)/self.Mass

    def addTorque(self, x, y, z): # Adds torque moments to the object based on mass moment of inertia. (In Newton-Meters)
        self.AngularAcceleration += ori.Vector3(x, y, z)/ori.Vector3(self.MMOI.x, self.MMOI.y, self.MMOI.z)

    def update(self, dt): # Updates rigidbody and appends data, dt: time step, delta time
        self.AngularVelocity += self.AngularAcceleration * dt # Integrating angular acceleration

        self.Orientation.IMU_ORI(self.AngularVelocity.x, self.AngularVelocity.y, self.AngularVelocity.z, dt) # Updating orientation based on angular rates
        self.EulerAngles = self.Orientation.quaternionToEuler() # Convert orientation quaternion to an Z-Y-X order Euler Angle representation

        self.GlobalAcceleration = self.Orientation.VectorRotate(self.Acceleration.x, self.Acceleration.y, self.Acceleration.z) # Rotate body frame acceleration into global values
        self.GlobalAcceleration += self.Gravity # Add gravity to the global acceleration

        self.Velocity += self.GlobalAcceleration * dt # Integrate Velocity
        self.Position += self.Velocity * dt # Integrate Position

        if self.Position.x <= 0.0 and self.Floor: # Floor function described above
            self.Velocity.x = 0.0
            self.Velocity.y = 0.0
            self.Velocity.z = 0.0

            self.Position.x = 0.0

        # ------------------------------------------- LOG DATA ---------------------------------------------

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
        self.Acceleration = ori.Vector3(0, 0, 0) # Zeroes acceleration
        self.AngularAcceleration = ori.Vector3(0, 0, 0) # Zeroes angular acceleration

    def createStandardDataSet(self, sName): # Creates a standard data set to append data to, sName: name for the dataset
        self.Dataset.Name = sName
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

    # You can directly call the Dataset property of DOF6, but these functions will make everything much easier, the same functions can be used for the Dataset object

    def addData(self, NameToFind, Data): # Adds data to the dataset, function referenced from DataUtility
        return self.Dataset.addData(NameToFind, Data)

    def find(self, NameToFind): # Returns a list of data from the requested dataset name, function referenced from DataUtility
        return self.Dataset.find(NameToFind)

    def processData(self): # Fills in zeroes for null data, function referenced from DataUtility
        return self.Dataset.processData()


class Simulation: # Simulation object to store simulation parameters
    Length = 0.0 # Length the simulation will be run for
    Gravity = 9.807 # Standard gravity value
    Time = 0.0 # Current time
    timeStep = 0.0 # Time to step through the simulation, dt
    iterations = 0 # Iterations the sim has executed, a 30 second sim with a 1 second dt would run for 30 iterations

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
        self.RGB = aRGB # dont ask


class PID: # Proportional Integral Derivative controller, very easy to use and tune
    kP = 0.0 # Proportional Gain
    kI = 0.0 # Integral Gain
    kD = 0.0 # Derivative Gain
    Setpoint = 0.0 # Setpoint, the target that the PID will try to aim for
    SetpointRate = 0.0 # Rate the setpoint can change, you can directly assign a value to the self.Setpoint for an instant change

    Error = 0
    previousError = 0
    P = 0.0
    I = 0.0
    D = 0.0
    output = 0.0 # Output of the PID algorithm

    def __init__(self, skP, skI, skD):
        self.kP = skP
        self.kI = skI
        self.kD = skD

    def PID(self, Input, dt): # Execute PID controller, input is the state you want to control, dt is change in time, delta time. Returns the output of the PID controller
        self.Error = self.Setpoint - Input
        self.P = self.Error
        self.I += self.Error * dt
        self.D = (self.Error - self.previousError) / dt

        self.output = (self.P * self.kP) + (self.I * self.kI) + (self.D * self.kD)

        self.previousError = self.Error

        return self.output

    def changeSetpoint(self, target, dt): # Function to change the setpoint gradually, target is the setpoint to change to, dt is delta time.
        PIDError = target - self.Setpoint
        PIDSpeed = self.SetpointRate * dt
        PIDError = clamp(PIDError, -PIDSpeed, PIDSpeed)
        self.Setpoint += PIDError
        if timeStep <= 0.0 or self.SetpointRate <= 0:
            self.Setpoint = target

    def reset(self): # Resets PID output
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.previousError = 0.0
        self.Error = 0.0


class TVC: # EVERYTHING IN RADIANS
    Max = 0.0 # Maximum angle for the TVC, a maximum angle means a maximum TVC deflection of +Max to -Max, so the range of actuation is 2 * Max
    Offset = 0.0 # Offset for the TVC, simulating misalignment, TVC angle might be zero but the offset will be accounted for in the torque calculations
    Lever = 0.0 # Lever arm the torque will be applied from, also known as the distance from the center of mass to the actuation point of the TVC
    AngleSpeed = 0.0 # Maximum speed the TVC can actuate its angle by, if it is 1rad/s, means that the TVC will only be able to change its angle by 1 rad every second

    Angle = 0.0  # Angle of the TVC (RADIANS)
    SideForce = 0.0 # Side force the TVC will produce given the deflection angle, and a thrust
    Torque = 0.0 # Torque the TVC is creating given the side force and lever arm

    def __init__(self, sMax, sOffset, sLever, sAngleSpeed):
        self.Max = sMax
        self.Offset = sOffset
        self.Lever = sLever
        self.AngleSpeed = sAngleSpeed

    def actuate(self, target, dt): # Actuate the TVC given a target (where to point the TVC at), and dt, time step, change in time, delta time
        TVCError = target - self.Angle
        TVCSpeed = self.AngleSpeed * dt
        TVCError = clamp(TVCError, -TVCSpeed, TVCSpeed)
        self.Angle += TVCError
        self.Angle = clamp(self.Angle, -self.Max, self.Max)

    def getTorque(self, Force): # Returns the torque created from the TVC given a thrust
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