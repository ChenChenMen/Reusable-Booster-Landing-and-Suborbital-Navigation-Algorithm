# Vertical launch and Auto-landing sequence test
import krpc
import numpy as np
import math

from math import sin
from math import cos
from math import pi
from numpy.linalg import inv

conn = krpc.connect();
vessel = conn.space_center.active_vessel;
create_relative = conn.space_center.ReferenceFrame.create_relative;
create_hybrid = conn.space_center.ReferenceFrame.create_hybrid;
flight_info = vessel.flight();

cBody = vessel.orbit.body;
gSur = cBody.surface_gravity;

surRF = vessel.surface_reference_frame;
bodyRF = cBody.reference_frame;
#body surface spherical tracking reference frame
bodySurRF = create_hybrid(
	position = bodyRF,
	rotation = surRF);

# Define the landing site as the top of the VAB
landing_latitude = -(0+(5.0/60)+(48.38/60/60))
landing_longitude = -(74+(37.0/60)+(12.2/60/60))
landing_altitude = 111

# Determine landing site reference frame
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

om = vessel.mass;
#found total amount of liquid fuel been carried
oFuelAmount = vessel.resources.amount("LiquidFuel");

#position info before launch
osufA = flight_info.surface_altitude;
oLa = flight_info.latitude;
oLon = flight_info.longitude;

#Launch Sequence
def launch():
	if vessel.situation == vessel.situation.pre_launch:
		print("Prelaunch surface altitude is ",osufA);
		print("Launch site latitude is ",oLa);
		print("Launch site longitude is ",oLon);
		print("clear for launch");
		print();

#Initiate launch by pressing the space bar
		vessel.control.brakes = False;
		vessel.control.throttle = 1;
		vessel.control.activate_next_stage();
		print("Launch sequence initiated");

		if vessel.situation == vessel.situation.flying:
			print("Lift off!");
			vessel.control.gear = False;
		else:
			print("Launch fail");
			return;

#In loop real-time assending control
def assendControl():
	vessel.auto_pilot.engage();
	print("Auto pilot engaged");

	vessel.control.rcs = True;
	vessel.auto_pilot.target_pitch = 90;
	while(vessel.resources.amount("LiquidFuel") > oFuelAmount * 0.6):
		vessel.control.throttle = 1;

	vessel.control.throttle = 0;

#alpha - rotate about z
#beta - rotate about y
#phi - rotate about x
def rotationMatrix (alpha, beta, phi):
	return (
		[cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(phi)-sin(alpha)*cos(phi), cos(alpha)*sin(beta)*cos(phi)+sin(alpha)*sin(phi)],
		[sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(phi)+cos(alpha)*cos(phi), sin(alpha)*sin(beta)*cos(phi)-cos(alpha)*sin(phi)],
		[-sin(beta), cos(beta)*sin(phi), cos(beta)*cos(phi)]);

# Method that Ignore air resistance during decending
# Simple newton second law

def prediction(m, v):
	a = vessel.available_thrust*0.96 / m - gSur;
	t = v/a;
	h = v*t - 0.5*a*t*t;
	return h;

def predictionAirDragVersion(m, v):
	aForce = flight_info.aerodynamic_force;
	aPho = flight_info.atmosphere_density;

	k = aForce[0] / v**2;
	force = vessel.available_thrust*0.96 - m*gSur;

	h = m/k*math.log(math.sqrt((force+aForce[0])/force));

	''' Suicide burn calculation with air drag
	Based on the aerodynamic force and current velocity
	air drag coefficient can be predicted'''
	return h;

def landingBurn(h, v):
	vessel.control.throttle = 1;
	if (h <= 1000):
		vessel.control.gear = True;

def decendControl():
	while (vessel.recoverable is False):
		h = flight_info.surface_altitude;
		m = vessel.mass;
		velVessel = vessel.velocity(bodySurRF);
		velRTarget = vessel.velocity(landing_reference_frame);

		targetAdjust();

		if (velVessel[0] < 0 and h <= 20000):
			vessel.control.brakes = True;

		if (velVessel[0] < 0 and h <= 10000):

			h = flight_info.surface_altitude;
			t1 = conn.space_center.ut;
			tarH = predictionAirDragVersion(m, velVessel[0]);
			t2 = conn.space_center.ut;

			if (h <= tarH + osufA + (t2-t1)*velVessel[0]):
				landingBurn(h, velVessel[0]);
			else:
				vessel.control.throttle = 0;
		else:
			vessel.control.throttle = 0;

	vessel.control.throttle = 0;

''' 
navigate the origin of the vessel's coordinate within certain range around landing site
keep the horizontal velocity minor
'''
def targetAdjust ():
	ob = vessel.orbit;
	pos = vessel.position(landing_reference_frame);
	vel = vessel.velocity(landing_reference_frame)

	print(pos, vel)
	print()

	

launch();
assendControl();
decendControl();