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

