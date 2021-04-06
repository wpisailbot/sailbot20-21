import math
import json

class P2P:
	def __init__(self, curpos, dest):
		self.curpos = curpos #coordinates of starting point
		self.dest = dest #coordinates of end point 
		self.state = 1
		
		
	

	def getAction(self, ):
		self.state2()
		
	def state1(self):
        #########
		#state 1#
		#########
		self.setTT(wind,cmpas)
	
	def state2(self):
	    
	    
	    print("hi")
	    
	def setTT(self, wind, cmpas):
		tempheadP = 45 #port
		tempheadS = 315 #starboard
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
			heading = windAng + tempheadP
			if heading >= 360:
				heading -= 360
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
			heading = windAng + tempheadS
			if heading >= 360:
				heading -= 360
			x = {"tack":"starboard","trimtab":"lift"}
			#json.dumps({"tack":"starboard","trimtab":"lift"})
			self.state == 2
		print(x)
		return(x)
	
	def state3(self,heading,pointofsail,windAng):
	    # get and keep the boat on course #
		variance = heading - self.track
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
		

x = P2P((0,0),(1,1))
x.getAction()
