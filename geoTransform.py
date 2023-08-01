from math import radians, sqrt, sin, cos, atan2

# Conversion from WGS84 to cartesian coordinates
def cart(geo_coord): 
    lat_rad = radians(geo_coord.lat)
    lon_rad = radians(geo_coord.lon)
    flat_factor = 1/298.257223563
    e_squared = 2 * flat_factor - flat_factor ** 2
    a = 6378137.0
    N = a/(sqrt(1 - e_squared * sin(lat_rad) ** 2))
    x = (N + geo_coord.alt) * cos(lat_rad) * cos(lon_rad)
    y = (N + geo_coord.alt) * cos(lat_rad) * sin(lon_rad)
    z = ((1 - e_squared) * N + geo_coord.alt) * sin(lat_rad)
    return x, y, z

def az(c1, c2):
    R = 6371*1000
    dlat = radians(c2.lat - c1.lat)
    dlon = radians(c2.lon - c1.lon)
    a = pow(sin(dlat/2), 2) + cos(radians(c1.lat))*cos(radians(c2.lat))*pow(sin(dlon/2),2)
    d = 2*R*atan2(sqrt(a), sqrt(1 - a))
    phi = atan2(sin(dlon)*cos(radians(c2.lat)), cos(radians(c1.lat))*sin(radians(c2.lat))-sin(radians(c1.lat))*cos(radians(c2.lat))*cos(dlon))
    return d, phi