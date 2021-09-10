

class Data:
    def __init__(self, sName, sIndex):
        self.Name = sName
        self.Index = sIndex
        self.Datas = []


class DataRecord: # Stores multiple data objects, a dataset
    def __init__(self, sName):
        self.Name = sName  # Name of the dataset
        self.DataObjects = []  # List of data objects
        self.Indexes = 0  # Number of data objects that the datarecord stores

    def createData(self, Name): # Creates a data object to store data to
        self.DataObjects.append(Data(Name, self.Indexes))
        self.Indexes += 1

    def addData(self, NameToFind, data): # Adds data to a data object within the datarecord
        for n in range(len(self.DataObjects)):
            if self.DataObjects[n].Name == NameToFind:
                return self.DataObjects[n].Datas.append(data)

        quit(str(self) + "Data not found: " + str(NameToFind))

    def find(self, NameToFind): # Finds a data object within the data record and returns the list of contents
        for n in range(len(self.DataObjects)):
            if self.DataObjects[n].Name == NameToFind:
                return self.DataObjects[n].Datas

        quit(str(self) + "Data not found: " + str(NameToFind))

    def processData(self): # Fills in zeroes for null data, fills in data up to the length of data that the first data index of the datarecord stores
        for i in range(len(self.DataObjects[0].Datas)):
            for n in range(len(self.DataObjects)):
                if len(self.DataObjects[n].Datas) < i+1:  # Check if the data is empty
                    self.DataObjects[n].Datas.append(0.0)

    def parseFile(self, directory): # Creates datarecord based on imported data, CSV formatted data
        file = open(directory, "r") # Read file

        # Parsing header names out and append data storate ------------------------------------------------------------------------------------------------------------------------------
        dataString = file.readline()  # Read first line of the file, so header names first

        k = 0
        for i in range(len(dataString)):  # Repeat for every character in the read datastring
            if dataString[i] == "," or dataString[i] == "\n":  # Cut string to up to the read section before the comma or return
                self.createData(str(dataString[k: i])) # Append dataname to the datanames list
                k = i + 1  # Set starting point of string to the next character that is not a comma or return

        # Parsing Data ------------------------------------------------------------------------------------------------------------------------------------------------------------------
        dataString = file.read()  # Read the next line, this time the first data string

        k = 0
        d = 0
        for i in range(len(dataString)):  # Repeat for the length of the read datastring
            if dataString[i] == "," or dataString[i] == "\n":  # Cut string to up to the read section before the comma or return
                self.DataObjects[d].Datas.append(float(dataString[k: i]))  # Append data to the 'd' data object to its Datas list
                k = i + 1  # Set starting point of string to the next character that is not a comma or return
                d += 1  # Move to the next data object
                if d > len(self.DataObjects) - 1:  # Set data object to find index to zero for the next line
                    d = 0

        file.close() # Close the file

    def createFile(self, directory): # Creates a file with the datarecord's data, CSV formatted data
        file = open(directory, "w") # Create new file

        for i in range(len(self.DataObjects)): # Write headers
            if i == len(self.DataObjects) - 1: # Check if the data header to write is the last one so to write a return rather than a comma separation
                file.write(self.DataObjects[i].Name + "\n")
            else:
                file.write(self.DataObjects[i].Name + ",")

        self.processData()

        for i in range(len(self.DataObjects[0].Datas)):
            for n in range(len(self.DataObjects)):
                if n == len(self.DataObjects) - 1:  # Check if the data to write is the last one so to write a return rather than a comma separation
                    file.write(str(round(self.DataObjects[n].Datas[i], 5)) + "\n")
                else:
                    file.write(str(round(self.DataObjects[n].Datas[i], 5)) + ",")

        file.close()


# --------------------------------------------------------------------------------- TESTING ---------------------------------------------------------------------------------
'''
flightData = DataRecord("flightData#1", [])
flightData.parsefile("Parsing/FL575.CSV")

print(" ")
print("DataNames: ")
for i in range(len(flightData.DataObjects)):
    print(flightData.DataObjects[i].Name)

print(" ")
print("Data: ")
for i in range(len(flightData.DataObjects)):
    print(flightData.DataObjects[i].Datas)

flightData.createfile("Data Directory/FLTEST.CSV")
'''