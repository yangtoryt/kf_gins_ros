import math
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
def llh_to_ecef(lat_rad, lon_rad, h_m):
    a=WGS84_A; e2=WGS84_E2
    sl=math.sin(lat_rad); cl=math.cos(lat_rad)
    slo=math.sin(lon_rad); clo=math.cos(lon_rad)
    N=a/(1.0 - e2*sl*sl)**0.5
    x=(N+h_m)*cl*clo; y=(N+h_m)*cl*slo; z=(N*(1.0-e2)+h_m)*sl
    return x,y,z
def ecef_to_enu(x,y,z,lat0_rad,lon0_rad,x0,y0,z0):
    sl=math.sin(lat0_rad); cl=math.cos(lat0_rad)
    slo=math.sin(lon0_rad); clo=math.cos(lon0_rad)
    dx=x-x0; dy=y-y0; dz=z-z0
    e=-slo*dx+clo*dy
    n=-clo*sl*dx - slo*sl*dy + cl*dz
    u= clo*cl*dx + slo*cl*dy + sl*dz
    return e,n,u
def rpy_to_quat(roll,pitch,yaw):
    cr=math.cos(roll*0.5); sr=math.sin(roll*0.5)
    cp=math.cos(pitch*0.5); sp=math.sin(pitch*0.5)
    cy=math.cos(yaw*0.5); sy=math.sin(yaw*0.5)
    w=cr*cp*cy + sr*sp*sy
    x=sr*cp*cy - cr*sp*sy
    y=cr*sp*cy + sr*cp*sy
    z=cr*cp*sy - sr*sp*cy
    return x,y,z,w
