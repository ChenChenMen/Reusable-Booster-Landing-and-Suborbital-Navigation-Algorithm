import krpc
import math
import numpy as np

from math import sin
from math import cos
from math import pi

import matplotlib.pyplot as plt

conn = krpc.connect()
vessel = conn.space_center.active_vessel
flight_info = vessel.flight()

# Celestical body where the flight occurs
cBody = vessel.orbit.body
gSur = cBody.surface_gravity
gPara = cBody.gravitational_parameter
cBodyRotSpeed = cBody.rotational_speed
squareCBRS = cBodyRotSpeed ** 2
equatorialR = cBody.equatorial_radius

# Reference frame creation
create_relative = conn.space_center.ReferenceFrame.create_relative
create_hybrid = conn.space_center.ReferenceFrame.create_hybrid

surRF = vessel.surface_reference_frame
bodyRF = cBody.reference_frame
# Body surface spherical tracking reference frame
bodySurRF = create_hybrid(
	position = bodyRF,
	rotation = surRF)

# Landing site reference frame - VAB building rooftop
# Define the landing site as the top of the VAB
landing_latitude = -(0+(5.0/60)+(48.38/60/60))
landing_longitude = -(74+(37.0/60)+(12.2/60/60))
landing_altitude = 111
# (orientation: x=zenith, y=north, z=east)
landing_position = cBody.surface_position(
    landing_latitude, landing_longitude, cBody.reference_frame)
q_long = (
    0,
    sin(-landing_longitude * 0.5 * pi / 180),
    0,
    cos(-landing_longitude * 0.5 * pi / 180)
)
q_lat = (
    0,
    0,
    sin(landing_latitude * 0.5 * pi / 180),
    cos(landing_latitude * 0.5 * pi / 180)
)
landing_reference_frame = \
    create_relative(
        create_relative(
            create_relative(
                cBody.reference_frame,
                landing_position,
                q_long),
            (0, 0, 0),
            q_lat),
        (landing_altitude, 0, 0))
  
#position info of the launch site
osurA = flight_info.surface_altitude
oLa = flight_info.latitude
oLon = flight_info.longitude

# Initialization data for C+ (standard) model
designatedFuelComsumptionRatio = 0.15
returnBurnThrottle = 0.3
landingBurnThrottle = 0.4
rollAlti = 1000
startPitchAlti = 0
pitchAlti = 40000
apoAlti = 700000
fullThrustAlti = 12000
brakeOnAlti = 25000
stopTimeTuple = (1.7, 0.5, 0.5)
decelerationTimeTuple = (1.3, 4, 3)

dataBModel = (0.4, 1, (0.7, 0.5, 0.7), (2, 4, 2), 1, 0.96, 5000)
dataCModel = (designatedFuelComsumptionRatio, fullThrustAlti, stopTimeTuple,\
 decelerationTimeTuple, returnBurnThrottle, landingBurnThrottle, brakeOnAlti)

'''0 --> B+ model
   1 --> C+ model'''
dataSelector = (dataBModel, dataCModel)
selectorIndex = 1

def launchSequence():
	#found total amount of liquid fuel been carried
	oFuelAmount = vessel.resources.amount("LiquidFuel")

	if vessel.situation == vessel.situation.pre_launch:
		print("Prelaunch surface altitude is ", osurA)
		print("Launch site latitude is ", oLa)
		print("Launch site longitude is ", oLon)
		print("\nclear for launch")

		vessel.control.brakes = False
		vessel.control.throttle = 1
		vessel.control.activate_next_stage()
		print("Launch sequence initiated\n")

		if vessel.situation == vessel.situation.flying:
			print("Lift off......")
			vessel.control.gear = False
		else:
			print("Launch fail")
			return 0

		vessel.auto_pilot.engage()
		print("Auto pilot engaged")

	# Either use up all desinated fuel or apoapsis reaches 100k 
	while(vessel.orbit.apoapsis <= apoAlti and \
		vessel.resources.amount("LiquidFuel") > oFuelAmount * dataSelector[selectorIndex][0]):
		alti = flight_info.surface_altitude

		#vessel.auto_pilot.target_roll = min(90, alti/rollAlti * 90)
		vessel.auto_pilot.target_heading = 90
		vessel.auto_pilot.target_pitch = max(35, 90 - max(alti - startPitchAlti, 0)/pitchAlti * 55)
		vessel.control.throttle = 0.5 + min(alti/dataSelector[selectorIndex][1], 0.5)
		# - min(max(alti/dataSelector[selectorIndex][1] - 1, 0), 0.5)

	vessel.control.throttle = 0
	print("Suborbit Entry Completed")
	return 1

def returnBurn():
	vessel.control.rcs = True
	vessel.auto_pilot.target_pitch = 180
	vessel.auto_pilot.stopping_time = dataSelector[selectorIndex][2]
	vessel.auto_pilot.deceleration_time = dataSelector[selectorIndex][3]

	vessel.auto_pilot.wait()

	vthetaPrediction = (0, 0)
	while (vthetaPrediction[0] + vessel.velocity(bodySurRF)[2] >= 0):

		alti = flight_info.surface_altitude + equatorialR
		antilaAngle = pi/2 - flight_info.latitude*pi/180
		lonAngle = flight_info.longitude*pi/180
		vr, vphi, vtheta = vessel.velocity(bodySurRF)

		minError = 40
		for v in range(10, 210, 20):
			for vphi in range(-6, 10, 4):
				(errorla, errorlon) = targetCalculator(alti, antilaAngle, lonAngle, vr, vphi, -v)
				if (minError > abs(errorlon) + abs(errorla) * 10):
					vthetaPrediction = (v, vphi)
					minError = abs(errorlon) + abs(errorla) * 10

		print(vthetaPrediction)

		for v in range(vthetaPrediction[0] - 10, vthetaPrediction[0] + 10, 2):
			for vphi in range(vthetaPrediction[1] - 2, vthetaPrediction[1] + 2):
				(errorla, errorlon) = targetCalculator(alti, antilaAngle, lonAngle, vr, vphi, -v)
				if (minError > abs(errorlon) + abs(errorla) * 10):
					vthetaPrediction = (v, vphi)
					minError = abs(errorlon) + abs(errorla) * 10

		print(vthetaPrediction)
		print()

		dvtheta = vthetaPrediction[0] - vessel.velocity(bodySurRF)[2]
		dvphi = vthetaPrediction[1] - vessel.velocity(bodySurRF)[1]
		dheading = 180/pi*math.atan(dvphi/max(100, dvtheta - 100))

		if (abs(dheading) >= 4):
			dheading = 0

		vessel.auto_pilot.target_heading = 90 - dheading
		if (vessel.auto_pilot.error >= 0.5):
			vessel.control.throttle = 0
			vessel.auto_pilot.wait()

		vessel.control.throttle = dataSelector[selectorIndex][4]

	
	vessel.control.throttle = 0;

def targetCalculator(r, phi, theta, vr, vphi, vtheta):
	dt = 0.2

	while (r >= equatorialR + landing_altitude):
		ar = -1*gPara/r**2 - 2*cBodyRotSpeed*vtheta + r*squareCBRS*sin(phi)**2
		aphi = 2*cBodyRotSpeed*vtheta - squareCBRS*r*sin(phi)*cos(phi)
		atheta = -2*cBodyRotSpeed*(vphi*sin(phi) + vr*cos(phi))

		r = r + vr*dt + 1/2*ar*dt**2
		phi = phi + vphi*dt/r + 1/2*aphi*dt**2/r
		theta = theta + vtheta*dt/r/sin(phi) + 1/2*atheta*dt**2/r/sin(phi)

		vr = vr + ar*dt
		vphi = vphi + aphi*dt 
		vtheta = vtheta + atheta*dt

	return (pi/2 - phi - landing_latitude*pi/180, theta - landing_longitude*pi/180)

def predictionAirDragVersion(m, v):
	aForce = flight_info.aerodynamic_force
	aPho = flight_info.atmosphere_density

	k = aForce[0] / v**2
	force = vessel.available_thrust*dataSelector[selectorIndex][5] - m*gSur

	h = m/k*math.log(math.sqrt((force+aForce[0])/force))

	''' Suicide burn calculation with air drag
	Based on the aerodynamic force and current velocity
	air drag coefficient can be predicted'''
	return h

def landingBurn(h, v):
	vessel.control.throttle = dataSelector[selectorIndex][5]
	if (h <= 1000):
		vessel.control.gear = True

def decendControl():
	vessel.auto_pilot.target_pitch = 90
	
	while (vessel.recoverable is False):
		h = flight_info.surface_altitude
		m = vessel.mass
		velVessel = vessel.velocity(bodySurRF)
		velRTarget = vessel.velocity(landing_reference_frame)

		if (velVessel[0] < 0):
			vessel.auto_pilot.disengage()
			vessel.auto_pilot.sas = True
			vessel.auto_pilot.sas_mode = vessel.auto_pilot.sas_mode.retrograde

			if (h <= dataSelector[selectorIndex][6]):
				vessel.control.brakes = True

			if (h <= 10000):

				h = flight_info.surface_altitude
				t1 = conn.space_center.ut
				tarH = predictionAirDragVersion(m, velVessel[0])
				t2 = conn.space_center.ut

				if (h <= tarH + (t2-t1)*velVessel[0]):
					landingBurn(h, velVessel[0])
				else:
					vessel.control.throttle = 0
			else:
				vessel.control.throttle = 0

	vessel.control.throttle = 0
vessel.auto_pilot.rcs = False

launchSequence()
returnBurn()
decendControl()