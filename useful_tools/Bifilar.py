R_mass = 0.730
R_StrCom = 0.20
R_StrLength = 0.65
R_avgRot = 1.92
R_tvcCOM = 0.295 # Unused

InertiaUp = R_mass * 9.8 * pow(R_avgRot, 2) * pow(R_StrCom, 2)
InertiaDown = 4 * pow(3.141519, 2) * R_StrLength

Inertia = InertiaUp/InertiaDown

print(Inertia)