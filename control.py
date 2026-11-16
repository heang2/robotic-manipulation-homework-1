"""Franka hand tool kinematics, damped IK, raster and force adaptation (SI)."""
import numpy as np
from scipy.spatial.transform import Rotation
DT=1/120
FPS=24
TOP=.125
ROWS=np.linspace(-.148,.148,5)
X0,X1=.345,.635
TCP=.157
MARKS=np.array([(x,y) for y in ROWS for x in np.linspace(X0,X1,12)])
HOME=np.array([0,-.44,0,-2.12,0,1.68,.78])
DOWN=Rotation.from_euler("x",np.pi).as_matrix()
ORIGINS=[(0,0,.333,0),(0,0,0,-np.pi/2),(0,-.316,0,np.pi/2),(.0825,0,0,np.pi/2),
         (-.0825,.384,0,-np.pi/2),(0,0,0,np.pi/2),(.088,0,0,np.pi/2)]
def kinematics(q):
    p=np.zeros(3); R=np.eye(3); origins=[]; axes=[]
    for (x,y,z,rx),theta in zip(ORIGINS,q):
        p=p+R@np.array([x,y,z])
        R=R@Rotation.from_euler("x",rx).as_matrix()
        axes.append(R[:,2]);origins.append(p)
        R=R@Rotation.from_euler("z",theta).as_matrix()
    p=p+R@np.array([0,0,.107+TCP])
    R=R@Rotation.from_euler("z",-np.pi/4).as_matrix()
    J=np.vstack((np.cross(axes,p-origins).T,np.array(axes).T))
    return p,R,J

def solve(q,point,iterations=2):
    for _ in range(iterations):
        p,r,j=kinematics(q)
        err=np.r_[point-p,Rotation.from_matrix(DOWN@r.T).as_rotvec()]
        inverse=j.T@np.linalg.solve(j@j.T+.018**2*np.eye(6),np.eye(6))
        q=q+np.clip(inverse@err+(np.eye(7)-inverse@j)@(.02*(HOME-q)),-.09,.09)
    return q

def ref(t):
    seq=[(0,[.42,-.07,.34]),(1.8,[X0,ROWS[0],TOP+.04]),(2.7,[X0,ROWS[0],TOP]),(3.6,[X0,ROWS[0],TOP])]
    end=3.6
    for i,y in enumerate(ROWS):
        x=X1 if i%2==0 else X0
        end+=1.95; seq.append((end,[x,y,TOP]))
        if i<4:
            end+=.62;seq.append((end,[x,ROWS[i+1],TOP]))
    seq.extend([(16.4,[X1,ROWS[-1],TOP]),(18,[.47,.10,.33]),(19,[.47,.10,.33])])
    for (a,p),(b,v) in zip(seq,seq[1:]):
        if t<=b:
            u=np.clip((t-a)/(b-a),0,1)
            return np.array(p)+(np.array(v)-p)*(u**3*(10-15*u+6*u*u))
    return np.array(seq[-1][1])
def force_goal(t):
    if 2.7<=t<16.4:return 10. if 7.3<=t<12.4 else 6.
    return 0.

