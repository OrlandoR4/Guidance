import math

'''
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                        QUATERNION AND EULER FUNCTIONS' INPUT AND OUTPUT FORMAT HAVE BEEN ADJUSTED FOR THE VECTOR3 CONVENTION:
                                                        X = ROLL, Y = PITCH, Z = YAW
                                                        
                            X = POSITION ALONG ROLL AXIS, Y = POSITION ALONG PITCH AXIS, Z = POSITION ALONG YAW AXIS
                            
                                              INPUT DATA TO FUNCTIONS AS XYZ FOR XYZ OUTPUT
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
'''


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def radToDeg(num):
    num = num * 180.0 / 3.14159
    return num


def degToRad(num):
    num = num / 180.0 * 3.14159
    return num


def rotate(x, y, theta):
    cs = math.cos(theta)
    sn = math.sin(theta)

    rotated_x = x * sn + y * cs
    rotated_y = x * cs - y * sn

    return Vector3(rotated_x, rotated_y, 0)


class Vector3:
    x = 0.0
    y = 0.0
    z = 0.0

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        z = self.z + other.z

        return Vector3(x, y, z)

    def __sub__(self, other):
        x = self.x - other.x
        y = self.y - other.y
        z = self.z - other.z

        return Vector3(x, y, z)

    def __mul__(self, other):
        if isinstance(other, Vector3):
            x = self.x * other.x
            y = self.y * other.y
            z = self.z * other.z
        else:
            x = self.x * other
            y = self.y * other
            z = self.z * other

        return Vector3(x, y, z)

    def __truediv__(self, other):
        if isinstance(other, Vector3):
            x = self.x / other.x
            y = self.y / other.y
            z = self.z / other.z
        else:
            x = self.x / other
            y = self.y / other
            z = self.z / other

        return Vector3(x, y, z)

    def normalize(self):
        norm = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
        self.x = self.x / norm
        self.y = self.y / norm
        self.z = self.z / norm

        return self

    def getLength(self):
        norm = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

        return norm

    def dotProduct(self, other):
        x = self.x * other.x
        y = self.y * other.y
        z = self.z * other.z

        return x + y + z

    def crossProduct(self, other):
        x = self.y * other.z - self.z * other.y
        y = -(self.x * other.z - self.z * other.x)
        z = self.x * other.y - self.y * other.x

        return Vector3(x, y, z)

    def radToDeg(self):
        self.x = self.x * 180.0 / 3.14159
        self.y = self.y * 180.0 / 3.14159
        self.z = self.z * 180.0 / 3.14159

        return self

    def degToRad(self):
        self.x = self.x / 180.0 * 3.14159
        self.y = self.y / 180.0 * 3.14159
        self.z = self.z / 180.0 * 3.14159

        return self


class Quaternion:
    w = 1.0
    x = 0.0
    y = 0.0
    z = 0.0

    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __mul__(self, other):
        q = Quaternion(1.0, 0.0, 0.0, 0.0)

        q.w = (self.w * other.w) - (self.x * other.x) - (self.y * other.y) - (self.z * other.z)
        q.x = (self.w * other.x) + (self.x * other.w) + (self.y * other.z) - (self.z * other.y)
        q.y = (self.w * other.y) - (self.x * other.z) + (self.y * other.w) + (self.z * other.x)
        q.z = (self.w * other.z) + (self.x * other.y) - (self.y * other.x) + (self.z * other.w)

        return q

    def normalize(self):
        norm = math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        self.w = self.w / norm
        self.x = self.x / norm
        self.y = self.y / norm
        self.z = self.z / norm

        return self

    def conjugate(self):
        q = Quaternion(1.0, 0.0, 0.0, 0.0)
        q.w = self.w
        q.x = -self.x
        q.y = -self.y
        q.z = -self.z

        return q

    def eulerToQuaternion(self, roll, pitch, yaw): # INPUT IN RADS, INPUT AS XYZ VECTOR3
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        self.w = cr * cp * cy + sr * sp * sy
        self.x = sr * cp * cy - cr * sp * sy
        self.y = cr * sp * cy + sr * cp * sy
        self.z = cr * cp * sy - sr * sp * cy

        return self

    def VectorRotate(self, x, y, z):
        RotatedVector = Quaternion(0.0, x, y, z)
        RotatedVector = self * RotatedVector * self.conjugate()

        return Vector3(RotatedVector.x, RotatedVector.y, RotatedVector.z)

    def quaternionToEuler(self): # OUTPUT IN RADS
        # used to get out of function range, commenting untested, comment the following three lines, and comment out the lower one
        # asinClamp = ( 2.0 * (self.w * self.y - self.z * self.x) )
        # asinClamp = clamp(asinClamp, -1.0, 1.0)
        # pitch = math.asin(asinClamp)
        roll = math.atan2(2.0 * (self.w * self.x + self.y * self.z), 1.0 - 2.0 * (self.x ** 2 + self.y ** 2))
        pitch = 2.0 * (self.w * self.y - self.z * self.x)
        yaw = math.atan2(2.0 * (self.w * self.z + self.x * self.y), 1.0 - 2.0 * (self.y ** 2 + self.z ** 2))

        return Vector3(roll, pitch, yaw) # Z-Yaw Y-Pitch X-Roll

    def fromAxisAngle(self, theta, vx, vy, vz):
        sn = math.sin(theta / 2.0)

        self.w = math.cos(theta / 2.0)
        self.x = vx * sn
        self.y = vy * sn
        self.z = vz * sn

        return self

    def IMU_ORI(self, x, y, z, dT):  # INPUT IN RADS
        quatDelta = Quaternion(0, 0, 0, 0)

        angle = Vector3(x, y, z).getLength()

        if angle == 0:
            angle = 1e-5

        quatDelta.fromAxisAngle(angle * dT, x/angle, y/angle, z/angle)

        quatMult = self * quatDelta
        self.w = quatMult.w
        self.x = quatMult.x
        self.y = quatMult.y
        self.z = quatMult.z

        return self