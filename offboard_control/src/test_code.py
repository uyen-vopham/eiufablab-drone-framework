import pymap3d as pm


e = 0.0
n = 0.0
u = 0.0
# The local coordinate origin (Zermatt, Switzerland)
lat0 = 11.052945 # deg
lon0 = 106.6661470  # deg
h0 = 0     # meters

# The point of interest
lat = 11.0529021000  # deg
lon = 106.6662267000   # deg
h = 7      # meters

e, n, u = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
print(f"e is: {e}, n is {n}, u is {u}")