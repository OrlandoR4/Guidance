R_mass = 0.83
R_StrCom = 0.42
R_StrLength = 0.5
R_avgRot = 1.05

InertiaUp = R_mass * 9.8 * pow(R_avgRot, 2) * pow(R_StrCom, 2)
InertiaDown = 4 * pow(3.141519, 2) * R_StrLength

Inertia = InertiaUp/InertiaDown

print(Inertia)