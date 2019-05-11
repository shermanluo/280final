import subprocess
import ast
import cv2
import numpy as np
import time

from picamera import PiCamera

camera = PiCamera()
TEMP_PICTURE_PATH = '/home/pi/Desktop/image.jpg'

def cleanDuplicates(Xs, Zs):
	realZs = []
	realXs = []
	for X, Z in zip(Xs, Zs):
		unique = True
		for X2, Z2 in zip(realXs, realZs):
			dist = np.linalg.norm(np.array([Z, X]), np.array([Z2, X2]))
			if dist <= 0.1: #<- TODO
				unique = False
				break
		if unique:
			realZs.append(Z)
			realXs.append(X)
	return realXs, realZs


def take_picture():
    camera.start_preview()
    sleep(5)
    camera.capture(TEMP_PICTURE_PATH)
    camera.stop_preview()
    return cv2.imread(TEMP_PICTURE_PATH)


def findFires(rotations):
	#i is how many rotations we do.
	#i = 4 means we do four 90 degree turns
	Zs = []
	Xs = []

	for i in range(rotations):
		theta = (360 / rotations) * i

		IMG1 = take_picture()
		time.sleep(1)

		ArduinoUnoSerial.write('4') #move camera forward
		time.sleep(5)

		IMG2 = take_picture()
		time.sleep(1)

		ArduinoUnoSerial.write('5') #move camera back
		time.sleep(5)

		ArduinoUnoSerial.write("1" + str(360/rotations))
		time.sleep(3)


		#TODO: TAKE TWO PICTURES AND SAVE TO IMG1, IMG2
                result = subprocess.run(['sshpass', '-p', '1', 'scp', IMG1, 'shermanluo@192.168.?/Desktop/280proj'], stdout=subprocess.PIPE)
                result = subprocess.run(['sshpass', '-p', '1', 'scp', IMG2, 'shermanluo@192.168.?/Desktop/280proj'], stdout=subprocess.PIPE)
                result = subprocess.run(['python', '280.py', IMG1, IMG2], stdout=subprocess.PIPE)
                temp = ast.literal_eval(result.stdout)
		Zs += temp[1]
		Xs += temp[0]
	if not Zs:
		return None
	#At this point, there will surely be duplicates that are pretty close. Let's get rid of the duplicates
	Xs, Zs = cleanDuplicates(Xs, Zs)

	locations = zip(Xs, Zs)
	minLoc = min(range(len(locations)), key = lambda x: np.linalg.norm(np.array(locations[x])))

	return minLoc #X and Z of closest fire

def findAndTargetFire():
		res = findFires(numRotations)
		if not res:
				return 0
		X, Z = res
		dist = np.linalg.norm(np.array([X, Z]))
		if Z >= 0 and X >= 0: #Q1
				theta = 90 - arctan(Z / X) #theta with respect to X axis
		if Z < 0 and X >= 0:
				theta = 90 - arctan(Z / X) #Q4
		if Z < 0 and X < 0:
				theta = 90 + arctan(Z / X) #Q3
				theta = -theta
		else:
				theta = 90 + arctan(Z / X) #Q2
				theta = -theta
		ArduinoUnoSerial.write('1' + str(theta)) #rotate by theta
		time.sleep(3)
		ArduinoUnoSerial.write('2' + str(dist)) #drive by distance
		time.sleep(5)
		ArduinoUnoSerial.write('3') #kill fire
		time.sleep(3)
		return 1

def killFires():
		while(findAndTargetFire()):
				pass



def main():
		ArduinoUnoSerial = serial.Serial('com15',9600)       #Create Serial port object called ArduinoUnoSerialData time.sleep(2)                                                             #wait for 2 secounds for the communication to get established
		killFires()
