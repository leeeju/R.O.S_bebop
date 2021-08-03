#!/usr/bin/env python
from microbit import*
import music

north = 0
east = 90
south = 180
west = 270

northPitch = 659 #E fifth
eastPitch = 440 #A forth
westPitch = 294 #D forth
southPitch = 196 #G 3rd

#start compass calibration
compass.calibrate()


def compassOutput(compassDir, directionDeg, pitch, displayChars):
	global compassTotal
	global pitchDuration
	#North has to be treated differently due to being greater than 0 and less than 360
	if directionDeg == 0:
		if compassDir > 315:
			pitchChange = pitch - (360 - compassDir)
		else:
			pitchChange = pitch - compassDir
	else:
		if compassDir > (directionDeg - 45) and compassDir <= directionDeg:
			pitchChange = pitch - (directionDeg - compassDir)
		else:
			pitchChange = pitch - (compassDir - directionDeg)

	display.show(displayChars)
	music.pitch(pitchChange, 50)


#MAIN APP LOOP
while True:
	# sleep(100)
	compassDir = compass.heading()
	if (compassDir > 315 or compassDir <= 45):
		compassOutput(compassDir, north, northPitch, 'N')
	elif compassDir > 45 and compassDir <= 135:
		compassOutput(compassDir, east, eastPitch, 'E')
	elif compassDir > 135 and compassDir <= 225:
		compassOutput(compassDir, south, southPitch, 'S')
	else:
		compassOutput(compassDir, west, westPitch, 'W')