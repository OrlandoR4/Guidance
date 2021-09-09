import math
import matplotlib.pyplot as plt

from OriMath import Vector3, Quaternion
from OriMath import radToDeg, degToRad, clamp, rotate

from NaviMath import DOF6, Simulation, PID, TVC
from NaviMath import schedule

from DataUtility import Data, DataRecord

from ThrustCurve import ThrustCurve

# ------------------------- SIMULATION -------------------------
Sim = Simulation()
Sim.Length = 3.5
Sim.timeStep = 0.05
Sim.Gravity = -9.807

# ------------------------- ROCKET BODY -------------------------
Rocket = DOF6("Rocket")
Rocket.Mass = 0.730
Rocket.DryMass = Rocket.Mass
Rocket.MMOI = Vector3(0.005, 0.042, 0.042)
Rocket.Gravity = Vector3(Sim.Gravity, 0, 0)
Rocket.Floor = True
Rocket.setFromEulerAngles(0, 5, -10, "deg")

# DATA TESTING
RocketApogee = 1.0 # Apogee for graph
posLim = 1.0 # Limit for position graph

# ------------------------- ORIENTATION PID -------------------------
YPID = PID(0.30, 0.2, 0.12)
YPID.Setpoint = 0.0

ZPID = PID(YPID.kP, YPID.kI, YPID.kD)
ZPID.Setpoint = 0.0

# ------------------------- POSITION PID -------------------------
Pos_YPID = PID(10, 0.0, 10)
Pos_YPID.Setpoint = 0.0

Pos_ZPID = PID(Pos_YPID.kP, Pos_YPID.kI, Pos_YPID.kD)
Pos_ZPID.Setpoint = 10.0

# ------------------------- TVC OBJECTS -------------------------
YTVC = TVC(degToRad(5.0), degToRad(0.0), 0.3, degToRad(45.0))
ZTVC = TVC(YTVC.Max, degToRad(0.0), YTVC.Lever, YTVC.AngleSpeed)

Rocket.Dataset.createData("YTVC")
Rocket.Dataset.createData("ZTVC")
Rocket.Dataset.createData("PitchSetpoint")
Rocket.Dataset.createData("YawSetpoint")
Rocket.Dataset.createData("Apogee")

# ---------------------------- MOTOR ----------------------------
Motor = ThrustCurve("motor_files/Estes_F15.rse")
MotorThrust = 0.0

while Sim.iterations <= Sim.Length/Sim.timeStep:
    '''
                    THE PID IN THIS EXAMPLE IS SET UP WITH DEG INPUT, DEG OUTPUT, DEG SETPOINT
                                  FEEL FREE TO CHANGE IT TO RAD-EVERYTHING
                                 
                        THE TVC WORKS IN RADIANS, BUT IS LOGGED AS DEGREES IN THIS EXAMPLE
               EULER ANGLES ARE LOGGED AS DEGREES BY DEFAULT, CHECK DOF6 IN NAVIMATH FOR MORE INFO
    '''
    # ------------- POSITION CONTROL --------------
    # YPID.Setpoint = -clamp(Pos_ZPID.PID(Rocket.Position.z, Sim.timeStep), -25, 25)
    # ZPID.Setpoint = clamp(Pos_YPID.PID(Rocket.Position.y, Sim.timeStep), -25, 25)

    # ------------- CONTROL --------------
    YPID.PID(radToDeg(Rocket.EulerAngles.y), Sim.timeStep)
    ZPID.PID(radToDeg(Rocket.EulerAngles.z), Sim.timeStep)

    RotatedTVC = rotate(YPID.output, ZPID.output, -Rocket.EulerAngles.x) # Rotate TVC to compensate for body roll

    YTVC.actuate(degToRad(RotatedTVC.x), Sim.timeStep) # PID output is represented to be degrees, TVC takes in radians
    ZTVC.actuate(degToRad(RotatedTVC.y), Sim.timeStep)

    # ------------- PHYSICS --------------
    # if schedule(2, Sim.Length, Sim.Time):
    #     MotorThrust = Motor.getThrust(Sim.Time-2)

    MotorThrust = Motor.getThrust(Sim.Time)

    Rocket.Mass = Rocket.DryMass + Motor.getMass(Sim.Time)/1000.0
    Rocket.addTorque(0, YTVC.getTorque(MotorThrust), ZTVC.getTorque(MotorThrust))
    Rocket.addForce(MotorThrust, 0, 0)

    # ------------ UPDATE BODIES ----------
    Rocket.update(Sim.timeStep)
    # ------------- LOGGING --------------
    Rocket.addData("Time", Sim.Time)
    Rocket.addData("YTVC", radToDeg(-YTVC.Angle)) # Reversed to be compatible with real data (CHANGE)
    Rocket.addData("ZTVC", radToDeg(-ZTVC.Angle)) # Reversed to be compatible with real data (CHANGE)
    Rocket.addData("PitchSetpoint", YPID.Setpoint)
    Rocket.addData("YawSetpoint", ZPID.Setpoint)

    # APOGEE DATA TESTING
    if Rocket.Position.x > RocketApogee:
        RocketApogee = round(Rocket.Position.x, 2)
    if math.sqrt(Rocket.Position.y ** 2 + Rocket.Position.z ** 2) > posLim:
        posLim = round(math.sqrt(Rocket.Position.y ** 2 + Rocket.Position.z ** 2), 2)

    Rocket.addData("Apogee", RocketApogee)

    # ------------- TIMESTEP -------------
    Sim.update()


# ---------------------------- DATA PROCESSING ----------------------------
Rocket.processData()
Rocket.Dataset.createFile("data_directory/FLIGHTLOGTEST_1.CSV")

# --------------- FIGURE ONE --------------------------------- MATPLOTLIB PLOTTING ------------------------------
time = Rocket.find("Time") # Find standard time

figure_1, ((axOri, axPos), (axCon, axVel)) = plt.subplots(2, 2)
figure_1.set_size_inches(12, 8)
figure_1.suptitle(Rocket.Dataset.Name)

axOri.plot(time, Rocket.find("Roll"), label="Roll", color = 'red')
axOri.plot(time, Rocket.find("Pitch"), label="Pitch", color = 'green')
axOri.plot(time, Rocket.find("Yaw"), label="Yaw", color = 'blue')

axOri.set_title("Attitude")
axOri.legend(loc="upper right")
axOri.grid(True)

axCon.plot(time, Rocket.find("YTVC"), label="YTVC", color = 'green')
axCon.plot(time, Rocket.find("ZTVC"), label="ZTVC", color = 'blue')
axCon.plot(time, Rocket.find("PitchSetpoint"), label="PitchSetpoint", color = 'teal')
axCon.plot(time, Rocket.find("YawSetpoint"), label="YawSetpoint", color = 'purple')

axCon.set_title("Control")
axCon.set_xlabel("Time")
axCon.legend(loc="upper right")
axCon.grid(True)

axPos.plot(time, Rocket.find("PosX"), label="PosX", color = 'red')
axPos2 = axPos.twinx()
axPos2.plot(time, Rocket.find("PosY"), label="PosY", color = 'green')
axPos2.plot(time, Rocket.find("PosZ"), label="PosZ", color = 'blue')

axPos.set_title("Position")
axPos.legend(loc="lower left")
axPos2.legend(loc="lower right")
axPos.grid(True)

axVel.plot(time, Rocket.find("VelX"), label="VelX", color = 'red')
axVel.plot(time, Rocket.find("VelY"), label="VelY", color = 'green')
axVel.plot(time, Rocket.find("VelZ"), label="VelZ", color = 'blue')
axVel.plot(time, Rocket.find("AccGX"), label="AccGX", color = 'orange')
axVel.plot(time, Rocket.find("AccGY"), label="AccGY", color = 'teal')
axVel.plot(time, Rocket.find("AccGZ"), label="AccGZ", color = 'indigo')

axVel.set_title("Velocity & Acceleration")
axVel.set_xlabel("Time")
axVel.legend(loc="upper right")
axVel.grid(True)
# -------------------------- FIGURE TWO -------------------------------
figure_2 = plt.figure()
figure_2.set_size_inches(6, 5)
figure_2.suptitle(Rocket.Dataset.Name)

axPos3D = figure_2.add_subplot(1, 1, 1, projection='3d')

axPos3D.plot(Rocket.find("PosY"), Rocket.find("PosZ"), Rocket.find("PosX"), label = 'Rocket Position', color = 'darkorange')

# --------------------------- APOGEE PLOT ---------------------------
apogeeIndex = Rocket.find("Apogee").index(RocketApogee)
axPos3D.text(Rocket.find("PosY")[apogeeIndex], Rocket.find("PosZ")[apogeeIndex], Rocket.find("PosX")[apogeeIndex], "   <- Apogee", (0, 0, 0))
axPos3D.scatter(Rocket.find("PosY")[apogeeIndex], Rocket.find("PosZ")[apogeeIndex], Rocket.find("PosX")[apogeeIndex], color="darkorange", edgecolors="black")
# --------------------------- LIFTOFF PLOT ---------------------------
axPos3D.text(Rocket.find("PosY")[0], Rocket.find("PosZ")[0], Rocket.find("PosX")[0], "   <- Liftoff!", (0, 0, 0))
axPos3D.scatter(Rocket.find("PosY")[0], Rocket.find("PosZ")[0], Rocket.find("PosX")[0], color="darkorange", edgecolors="black")

axPos3D.set(xlim=(-posLim, posLim), ylim=(-posLim, posLim), zlim=( 0, RocketApogee+(RocketApogee*0.1) ))
axPos3D.set_title("Apogee: " + str(RocketApogee))
axPos3D.set_xlabel('Y Position')
axPos3D.set_ylabel('Z Position')
axPos3D.set_zlabel('X Position')
# axPos3D.legend(loc="upper left")
axPos3D.grid(True)

# ------------------------------------------- FIGURE THREE -------------------------------------------
figure_3, (axAcc) = plt.subplots(1, 1)
figure_3.set_size_inches(6, 4)
figure_3.suptitle(Rocket.Dataset.Name)

axAcc.plot(time, Rocket.find("AccX"), label="AccX", color = 'red')
axAcc.plot(time, Rocket.find("AccY"), label="AccY", color = 'green')
axAcc.plot(time, Rocket.find("AccZ"), label="AccZ", color = 'blue')

axAcc.set_title("Body Acceleration")
axAcc.legend(loc="upper right")
axAcc.grid(True)

plt.show()