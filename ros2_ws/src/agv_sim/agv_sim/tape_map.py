# import math
# import numpy as np

# def stem (x0=-1.0, x1=3.0 ,n=40):
# 	"""straight tape along the x axis. """
# 	return [(x0+(x1-x0)*i /n ,0.0) for i in range(n+1)]

# def branch (x0,y0,length ,dy,n=60):
# 	"""smooth S-curve from (x0,y0) , sideways by over 'length', same heading at both ends."""
# 	pts= []
# 	for i in range(n+1):
# 		t=i/n
# 		pts.append((x0+length*t,y0+dy*(1-math.cos(math.pi*t))/2))
# 	return pts

# def straight(x0,y0,x1,n=20):
# 	return [(x0+(x1-x0)*i/n ,y0) for i in range(n+1)]

# def y_fork():
# 	"""return a list of polylines (each a list of (x,y))."""
# 	left=branch(3.0,0.0,4.0,+1.5)+ straight(7.0,1.5,11.0)
# 	right=branch(3.0,0.0,4.0,-1.5)+straight(7.0,-1.5,11.0)
# 	return [stem(),left,right]

# def to_segments(lines):
# 	"""all plylines -> arrays A,B of the segment start and end points ,shape (N,2)"""
# 	A,B=[],[]

# 	for line in lines:
# 		for p,q in zip(line[:-1],line[1:]):
# 			A.append(p)
# 			B.append(q)

# 	return np.array(A),np.array(B)

# def dist_to_tape(points,A,B):
# 	"""Distance from each point (M,2) to the nearest segment. Retuens shape (m,)."""

# 	P=points[:,None,:] #(M,1,2)
# 	AB=B-A #(N,2)
# 	t=np.sum((P-A)*AB,axis=2)/np.maximum(np.sum(AB*AB,axis=1),1e-12) #(M,N)
# 	t=np.clip(t,0.0,1.0) #stay on the segment
# 	closest=A+t[...,None]*AB #(M,N,2)
# 	return np.min(np.linalg.norm(P-closest,axis=2),axis=1)

import math 
import numpy as np

TAPE_WIDTH = 0.025     # magnetic tape width (m): Naviq MTRK25 (use 0.05 for MTRK50)
MARKER_SIZE = 0.05     # side of a square Navicode marker (m)
MARKER_GAP = 0.025     # gap between tape edge and marker edge (m); the manual recommends 20-30 mm
MARKER_OFFSET = TAPE_WIDTH / 2 + MARKER_GAP + MARKER_SIZE / 2   # marker centre from tape centre (m)


class Path:
    """Turtle that draws a polyline: start pose, then straight() and arc() moves"""
    def __init__(self,x,y,heading_deg):
        self.x,self.y,self.th= x,y,math.radians(heading_deg)
        self.pts=[(x,y)]

    def straight(self,length,step=0.05):

        n=max(1,int(length/step))
        for i in range(1,n+1):
            d=length*i/n
            self.pts.append((self.x+d*math.cos(self.th),self.y+d*math.sin(self.th)))
        self.x,self.y=self.pts[-1]
        return self

    def arc(self,radius,angle_deg,step=0.05):
        """positive angle =turn left negative angle turn right """
        a=math.radians(angle_deg)

        side=1 if a>0 else -1

        cx=self.x -side* radius*math.sin(self.th) #circle center
        cy=self.y + side*radius*math.cos(self.th)
        n=max(1,int(abs(a)*radius/step))
        th0 = self.th

        for i in range(1,n+1):
            t=th0+a*i/n
            self.pts.append((cx+side*radius*math.sin(t),cy-side*radius*math.cos(t)))

        self.th=th0+a
        self.x,self.y=self.pts[-1]
        return self


R_SPUR = 0.8     # radius of the junction arcs (m)
STEM = 1.5       # length of a station stem (m)
TAG_AT = 1.0     # Navicode tag distance along the stem from its base (m)
START_POSE = (-3.25, 2.5, 0.0)   # x, y, heading (rad): on the START stem, facing the loop
SPURS = [('START', 4, -1, 2.5, -90),
         ('S3', 3, 4, 0, 0),
         ('S2', 2, 6, 5, 180),
         ('S1', 1, 2.5, 5, 180)]

def spur(px, py, d_deg):
	"""Y-junction on the loop at (px, py); the loop runs in direction d_deg there.
	Returns the 3 polylines (entry arc, stem, exit arc) and the tag position."""
	d = math.radians(d_deg)
	dx, dy = math.cos(d), math.sin(d)
	entry = Path(px - dx * R_SPUR, py - dy * R_SPUR, d_deg).arc(R_SPUR, -90)
	bx, by = entry.x, entry.y                       # stem base B
	stem = Path(bx, by, d_deg - 90).straight(STEM)
	exit_ = Path(bx, by, d_deg + 90).arc(R_SPUR, -90)
	s = math.radians(d_deg - 90)                    # direction from B along the stem
	tag = (bx + TAG_AT * math.cos(s), by + TAG_AT * math.sin(s))
	r = s - math.pi / 2                             # right of the stem direction
	marker = (tag[0] + MARKER_OFFSET * math.cos(r), tag[1] + MARKER_OFFSET * math.sin(r))
	return [entry.pts, stem.pts, exit_.pts], tag, marker

def loop_path():
	"""The main loop, counter-clockwise, as a Path."""
	loop = Path(0, 0, 0)
	loop.straight(8).arc(1, 90).straight(3).arc(1, 90).straight(8).arc(1, 90).straight(3).arc(1, 90)
	return loop

def loop_positions():
	"""Distance (m) along the loop, counter-clockwise from (0, 0), of each spur's junction.
	Returns (positions, loop_length)."""
	pts = loop_path().pts
	cum = [0.0]
	for p, q in zip(pts[:-1], pts[1:]):
		cum.append(cum[-1] + math.dist(p, q))       # running length of the loop
	pos = {}
	for name, code, px, py, d in SPURS:
		nearest = min(range(len(pts)), key=lambda k: math.dist(pts[k], (px, py)))
		pos[name] = cum[nearest]
	return pos, cum[-1]



def station_loop():
	"""Counter-clockwise loop with START, S1, S2, S3 spurs.
	Returns (lines, stations); stations[name] = {'tag': (x, y), 'code': n}."""
	loop = loop_path()
	loop.straight(8).arc(1, 90).straight(3).arc(1, 90).straight(8).arc(1, 90).straight(3).arc(1, 90)
	lines = [loop.pts]
	stations = {}

	for name, code, px, py, d in SPURS:
		spur_lines, tag, marker = spur(px, py, d)
		lines += spur_lines
		stations[name] = {'tag': tag, 'marker': marker, 'code': code}

	return lines, stations


def to_segments(lines):
	"""All polylines -> arrays A, B of segment start and end points, shape (N, 2)."""
	A, B = [], []
	for line in lines:
		for p, q in zip(line[:-1], line[1:]):
			A.append(p)
			B.append(q)
	return np.array(A), np.array(B)


# def dist_to_tape(points, A, B):
# 	"""Distance from each point (M, 2) to the nearest segment. Returns shape (M,).
# 	Used by the old 16-cell sensor; removed in Step 3."""
# 	P = points[:, None, :]
# 	AB = B - A
# 	t = np.sum((P - A) * AB, axis=2) / np.maximum(np.sum(AB * AB, axis=1), 1e-12)
# 	t = np.clip(t, 0.0, 1.0)
# 	closest = A + t[..., None] * AB
# 	return np.min(np.linalg.norm(P - closest, axis=2), axis=1)


def read_mts160(x, y, th, A, B, sensor_x=0.25, half_width=0.08, merge_dist=TAPE_WIDTH):
	"""Virtual MTS160 at sensor_x ahead of the robot pose (x, y, th).
	Returns None if no tape, else (left_mm, left_deg, right_mm, right_deg)
	in MTS convention: position +right, angle + when the tape heads right."""

	c, s = math.cos(th), math.sin(th)
	S = np.array([x + sensor_x * c, y + sensor_x * s])   # sensor centre
	L = np.array([-s, c])                                # unit vector to the robot's left
	AB = B - A
	D = A - S
	# solve  S + u*L = A + t*AB  for every segment (u: along the bar, t: along the segment)
	det = AB[:, 0] * L[1] - AB[:, 1] * L[0]
	ok = np.abs(det) > 1e-9                              # skip segments parallel to the bar
	det = np.where(ok, det, 1.0)
	u = (AB[:, 0] * D[:, 1] - AB[:, 1] * D[:, 0]) / det
	t = (L[0] * D[:, 1] - L[1] * D[:, 0]) / det
	hit = ok & (t >= 0) & (t <= 1) & (np.abs(u) <= half_width)

	phi = np.arctan2(AB[hit, 1], AB[hit, 0])             # tape heading at each crossing
	ang = (phi - th + math.pi / 2) % math.pi - math.pi / 2   # relative, folded to ±90°

	tracks = []                                          # (u, angle), leftmost first
	for uu, aa in sorted(zip(u[hit], ang), key=lambda p: -p[0]):
		if tracks and abs(tracks[-1][0] - uu) < merge_dist:
			continue                                     # same tape as the previous crossing
		tracks.append((uu, aa))
	if not tracks:
		return None
	lu, la = tracks[0]
	ru, ra = tracks[-1]
	return (round(-lu * 1000), round(-math.degrees(la)),     # the minus signs convert
	        round(-ru * 1000), round(-math.degrees(ra)))     # ROS +left to MTS +right


def marker_under_sensor(x, y, th, mx, my, sensor_x=0.25, size=MARKER_SIZE, depth=0.035, half_width=0.08):
	"""Is the marker centred at (mx, my) under the sensor footprint?
	Returns 'left', 'right' or None (side relative to the robot)."""
	c, s = math.cos(th), math.sin(th)
	dx = mx - (x + sensor_x * c)
	dy = my - (y + sensor_x * s)
	fwd = dx * c + dy * s          # marker position in the robot frame: ahead (+) / behind (-)
	lat = -dx * s + dy * c         # left (+) / right (-)
	if abs(fwd) <= (size + depth) / 2 and abs(lat) <= half_width:
		return 'left' if lat > 0 else 'right'
	return None

