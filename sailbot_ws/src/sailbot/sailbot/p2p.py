import math
import json

class P2P:
	def __init__(self, curpos, dest):
		self.curpos = curpos #coordinates of starting point
		self.dest = dest #coordinates of end point 
		self.state = 1
		self.tempheadWP = 45 #windward + port
		self.tempheadWS = 315 #windward + starboard
		
	

	def getAction(self, ):
		self.state1()
		
	def state1(self):
        #########
		#state 1#
		#########
		self.setTT(wind,cmpas)
	
	def setTT(self, wind, cmpas):
		windAng = self.getWindToNorth(wind,cmpas) #direction of wind relative to north
		boatAng = self.getHeading() #direction relative to north to get from current position to end position
		if boatAng < 0:
		    boatAng += 360
		pointofsail = boatAng - windAng #angle between wind and direction to get to destination
		print(windAng)
		heading = 0 #desired direction of travel taking wind into account, relative to north
		if pointofsail < 0:
			pointofsail += 360
		if pointofsail >= 0 and pointofsail < 45:
			heading = windAng + self.tempheadWP
			if heading >= 360:
				heading -= 360
			x = {"tack":"port","trimtab":"lift"}
			#json.dumps({"tack":"port","trimtab":"lift"})
			#self.state2(heading,pointofsail)
		elif pointofsail >= 45 and pointofsail <= 135:
			heading = boatAng
			x = {"tack":"port","trimtab":"lift"}
			#json.dumps({"tack":"port","trimtab":"lift"})
			#self.state3(heading,pointofsail)
		elif pointofsail > 135 and pointofsail < 180:
			heading = boatAng
			x = {"tack":"port","trimtab":"drag"}
			#json.dumps({"tack":"port","trimtab":"drag"})
			#self.state3(heading,pointofsail)
		elif pointofsail >= 180 and pointofsail < 225:
			heading = boatAng
			x = {"tack":"starboard","trimtab":"drag"}
			#json.dumps({"tack":"starboard","trimtab":"drag"})
			#self.state2(heading,pointofsail)
		elif pointofsail >= 225 and pointofsail <= 315:
			heading = boatAng
			x = {"tack":"starboard","trimtab":"lift"}
			#json.dumps({"tack":"starboard","trimtab":"lift"})
			#self.state3(heading,pointofsail)
		elif pointofsail >= 315 and pointofsail < 360:
			heading = windAng + self.tempheadWS
			if heading >= 360:
				heading -= 360
			x = {"tack":"starboard","trimtab":"lift"}
			#json.dumps({"tack":"starboard","trimtab":"lift"})
			#self.state2(heading,pointofsail)
		print(x)
		return(x)
	
	def state2(self,heading,pointofsail,windAng):
	    # get and keep the boat on course #
		variance = heading - self.track
		if variance == 0: # heading=track
		    self.evenout()
		elif variance < 0 and pointofsail < 180: #heading<track & pTack
			rudders = {"channel" : "8", "angle" : "50"}
		elif variance < 0 and pointofsail >= 180: #heading<track & sTack
			self.falloff(pointofsail)
		elif variance > 0 and pointofsail < 180: #heading>track & pTack
			self.falloff(pointofsail)
		elif variance > 0 and pointofsail >= 180: #heading>track & sTack
			self.headup(pointofsail)
		
		newBoatAng = self.getHeading()
		
	def headup(self,pointofsail): #turn toward windward
		if pointofsail < 180: #port tack
			rudders = {"channel" : "8", "angle" : "50"}
		elif pointofsail >= 180: #starboard tack
			rudders = {"channel" : "8", "angle" : "90"}
	
	def evenout(self): #straighten boat
		rudders = {"channel" : "8", "angle" : "70"}
	
	def falloff(self,pointofsail): #turn toward leeward
		if pointofsail < 180: #port tack
			rudders = {"channel" : "8", "angle" : "90"}
		elif pointofsail >= 180: #starboard tack
			rudders = {"channel" : "8", "angle" : "50"}
	

		
			
		

			
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
		

x = P2P(315,45,0,(0,0),(1,1))
x.getAction()

