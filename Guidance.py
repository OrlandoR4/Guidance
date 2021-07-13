import math
import matplotlib.pyplot as plt

import OriMath as ori
from OriMath import radToDeg, degToRad, clamp, rotate

import NaviMath as nav
from NaviMath import schedule

import DataUtility as dat

# ------------------------- SIMULATION -------------------------
Sim = nav.Simulation()
Sim.Length = 30.0
Sim.timeStep = 0.01
Sim.Gravity = -9.807

# ------------------------- ROCKET BODY -------------------------
Rocket = nav.DOF6()
Rocket.Mass = 0.6
Rocket.MMOI = ori.Vector3(0.005, 0.0348, 0.0348)
Rocket.Gravity = ori.Vector3(Sim.Gravity, 0, 0)
Rocket.Floor = True
Rocket.setFromEulerAngles(0, -5.0, 10, "deg")
Rocket.createStandardDataSet("Rocket Data")

# DATA TESTING
RocketApogee = 1.0 # Apogee for graph
posLim = 1.0 # Limit for position graph

# ------------------------- ORIENTATION PID -------------------------
YPID = nav.PID(0.25, 0.0, 0.1)
YPID.Setpoint = 0.0

ZPID = nav.PID(YPID.kP, YPID.kI, YPID.kD)
ZPID.Setpoint = 0.0

# ------------------------- POSITION PID -------------------------
Pos_YPID = nav.PID(10, 0.0, 10)
Pos_YPID.Setpoint = 0.0

Pos_ZPID = nav.PID(Pos_YPID.kP, Pos_YPID.kI, Pos_YPID.kD)
Pos_ZPID.Setpoint = 0.0

# ------------------------- TVC OBJECTS -------------------------
YTVC = nav.TVC(5.0, 0.0, 0.3, 45.0)
ZTVC = nav.TVC(YTVC.Max, 0.0, YTVC.Lever, YTVC.AngleSpeed)

MotorThrust = 10.0

Rocket.Dataset.createData("YTVC")
Rocket.Dataset.createData("ZTVC")
Rocket.Dataset.createData("PitchSetpoint")
Rocket.Dataset.createData("YawSetpoint")
Rocket.Dataset.createData("Apogee")

while Sim.iterations <= Sim.Length/Sim.timeStep:

    # ------------- POSITION CONTROL --------------
    # YPID.Setpoint = -clamp(Pos_ZPID.PID(Rocket.Position.z, Sim.timeStep), -25, 25)
    # ZPID.Setpoint = clamp(Pos_YPID.PID(Rocket.Position.y, Sim.timeStep), -25, 25)

    # ------------- CONTROL --------------
    YPID.PID(radToDeg(Rocket.EulerAngles.y), Sim.timeStep)
    ZPID.PID(radToDeg(Rocket.EulerAngles.z), Sim.timeStep)

    RotatedTVC = rotate(YPID.output, ZPID.output, -Rocket.EulerAngles.x)

    YTVC.actuate(RotatedTVC.x, Sim.timeStep)
    ZTVC.actuate(RotatedTVC.y, Sim.timeStep)

    # ------------- PHYSICS --------------
    if not schedule(0, 8, Sim.Time):
        MotorThrust = 0.0

    Rocket.addTorque(0, YTVC.getTorque(MotorThrust), ZTVC.getTorque(MotorThrust))
    Rocket.addForce(MotorThrust, 0, 0)

    # ------------ UPDATE BODIES ----------
    Rocket.update(Sim.timeStep)

    # ------------- LOGGING --------------
    Rocket.addData("Time", Sim.Time)
    Rocket.addData("YTVC", -YTVC.Angle) # Reversed to be compatible with real data (CHANGE)
    Rocket.addData("ZTVC", -ZTVC.Angle) # Reversed to be compatible with real data (CHANGE)
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
Rocket.Dataset.createFile("Data Directory/FLIGHTLOGTEST_1.CSV")

# --------------- FIGURE ONE --------------------------------- MATPLOTLIB PLOTTING ------------------------------
time = Rocket.find("Time") # Find standard time

figure_1, ((axOri, axPos), (axCon, axVel)) = plt.subplots(2, 2)
figure_1.set_size_inches(15, 9)
figure_1.suptitle(Rocket.Dataset.Name)

axOri.plot(time, Rocket.find("Roll"), label="Roll", color = 'red')
axOri.plot(time, Rocket.find("Pitch"), label="Pitch", color = 'green')
axOri.plot(time, Rocket.find("Yaw"), label="Yaw", color = 'blue')

axOri.set_title("Attitude")
axOri.legend(loc="upper right")
axOri.grid(True)

axCon.plot(time, Rocket.find("YTVC"), label="YTVC", color = 'green')
axCon.plot(time, Rocket.find("ZTVC"), label="ZTVC", color = 'blue')
axCon.plot(time, Rocket.find("PitchSetpoint"), label="PitchSetpoint", color = 'darkgreen')
axCon.plot(time, Rocket.find("YawSetpoint"), label="YawSetpoint", color = 'darkblue')

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
figure_1.set_size_inches(9, 9)
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
axPos3D.set_title("Rocket's Position" + "\nRocket's Apogee: " + str(RocketApogee))
axPos3D.set_xlabel('Y Position')
axPos3D.set_ylabel('Z Position')
axPos3D.set_zlabel('X Position')
# axPos3D.legend(loc="upper left")
axPos3D.grid(True)

plt.show()
