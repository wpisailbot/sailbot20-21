import math
import json

class P2P:
	def __init__(self, curpos, dest):
		self.curpos = curpos #coordinates of starting point
		self.dest = dest #coordinates of end point 
		self.state = 1
		self.temphead = 0 #45 when on port tack, 135 when on starboard
	

	def getAction(self, wind, cmpas):
		windAng = self.getWindToNorth(wind,cmpas) #direction of wind relative to north
		boatAng = self.getHeading() #direction relative to north to get from current position to end position
		if boatAng < 0:
			boatAng += 360
		pointofsail = boatAng - windAng #angle between wind and direction to get to destination
		if pointofsail < 0:
			pointofsail += 360
		if self.state == 1:
			self.state1(pointofsail, windAng, boatAng)
		elif self.state == 2:
			self.state2()
		elif self.state == 3:
			self.state3()
		
	def state1(self, pointofsail, windAng, boatAng):
		#########
		#state 1#
		#########
		if pointofsail >= 0 and pointofsail < 45:
			self.temphead = 45
			x = {"tack":"port","trimtab":"lift"}
			#json.dumps({"tack":"port","trimtab":"lift"})
			self.state == 2
		elif pointofsail >= 45 and pointofsail <= 135:
			heading = boatAng
			x = {"tack":"port","trimtab":"lift"}
			#json.dumps({"tack":"port","trimtab":"lift"})
			self.state == 3
		elif pointofsail > 135 and pointofsail < 180:
			heading = boatAng
			x = {"tack":"port","trimtab":"drag"}
			#json.dumps({"tack":"port","trimtab":"drag"})
			self.state == 3
		elif pointofsail >= 180 and pointofsail < 225:
			heading = boatAng
			x = {"tack":"starboard","trimtab":"drag"}
			#json.dumps({"tack":"starboard","trimtab":"drag"})
			self.state == 3
		elif pointofsail >= 225 and pointofsail <= 315:
			heading = boatAng
			x = {"tack":"starboard","trimtab":"lift"}
			#json.dumps({"tack":"starboard","trimtab":"lift"})
			self.state == 3
		elif pointofsail >= 315 and pointofsail < 360:
			self.temphead = 315
			x = {"tack":"starboard","trimtab":"lift"}
			#json.dumps({"tack":"starboard","trimtab":"lift"})
			self.state == 2
		print(x)
		return(x)
	
	def state2(self, boatAng, windAng, track):
		tempheading = windAng + self.temphead
		if tempheading >= 360:
			while tempheading >= 360:
				tempheading -= 360
		# get and keep the boat on course #
		variance = tempheading - track
		if variance < 4 and vaiance > -4 : # heading=track
			rudders = {"channel" : "8", "angle" : "70"}
		elif variance < -4 and pointofsail < 180: #heading<track & pTack
			rudders = {"channel" : "8", "angle" : "90"}
		elif variance < -4 and pointofsail >= 180: #heading<track & sTack
			rudders = {"channel" : "8", "angle" : "50"}
		elif variance > 4 and pointofsail < 180: #heading>track & pTack
			rudders = {"channel" : "8", "angle" : "50"}
		elif variance > 4 and pointofsail >= 180: #heading>track & sTack
			rudders = {"channel" : "8", "angle" : "90"}
		if self.temphead == 45:
			if (boatAng < 315) and (boatAng > 180):
				self.state = 1
		elif self.temphead == 315:
			if (boatAng > 45) and (boatAng < 180):
				self.state = 1
		print("hi")

	def state3(self,heading,pointofsail,windAng):
		
	
	def getHeading(self):
		X = math.cos(math.radians(self.dest[0]))*math.sin(math.radians(self.difLon()))
		Y = math.cos(math.radians(self.curpos[0]))*math.sin(math.radians(self.dest[0]))-math.sin(math.radians(self.curpos[0]))*math.cos(math.radians(self.dest[0]))*math.cos(math.radians(self.difLon()))
		angle = math.degrees(math.atan2(X,Y))
		if angle < 0:
            angle += 360
		print(X,Y,angle)
		return(angle)

	def getWindToNorth(self, wind, cmpas):
		angle = wind + cmpas
		if angle >= 360:
			while angle >= 360:
				angle -= 360
			return angle
		else:
			return angle
			
	def difLon(self):
		return self.dest[1]-self.curpos[1]

	def difLat(self):
		return self.dest[0]-self.curpos[0]
		

x = P2P((0,0),(1,1))
x.getAction()
