from xml.dom import minidom;

from scipy.interpolate import interp1d

class ThrustCurve:
  def __init__(self, fileName):
    try:
      file = minidom.parse(fileName)
    except FileNotFoundError:
      quit(str(self) + "__init__: Engine thrust curve file not found")

    try:
      # Structure of .RSE file implies multiple motors could be added to a singular file,
      # this ensures we only take data from the first one
      data = file.getElementsByTagName("data")[0].getElementsByTagName("eng-data")
    except IndexError:
      quit(str(self) + "__init__: Engine thrust curve file invalid, please provide a .rse file")

    if len(data) == 0:
      quit(str(self) + "__init__: Engine thrust curve empty")

    measurementTimes = []
    measurementValues = []

    for dataPoint in data:
      try:
        measurementTimes.append(float(dataPoint.getAttribute("t")))
        measurementValues.append(float(dataPoint.getAttribute("f")))
      except ValueError:
        quit(str(self) + "__init__: Engine thrust data point invalid")

    self.thrustInterpolator = interp1d(measurementTimes, measurementValues, bounds_error=False, fill_value=0)

  def getThrust(self, time):
    return self.thrustInterpolator([time])[0]
