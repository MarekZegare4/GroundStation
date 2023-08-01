from math import radians, sqrt, sin, cos, atan2, degrees

# Conversion from WGS84 to cartesian coordinates
class geo_coord():
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class cart_coord():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

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

def DistAziElev(c1, c2):
    R = 6371000
    dlat = radians(c2.lat - c1.lat)
    dlon = radians(c2.lon - c1.lon)
    a = pow(sin(dlat/2), 2) + cos(radians(c1.lat))*cos(radians(c2.lat))*pow(sin(dlon/2),2)
    d = 2*R*atan2(sqrt(a), sqrt(1 - a))
    azimuth = degrees(atan2(sin(dlon)*cos(radians(c2.lat)), cos(radians(c1.lat))*sin(radians(c2.lat))-sin(radians(c1.lat))*cos(radians(c2.lat))*cos(dlon)))
    if azimuth < 0:
        azimuth += 360

    # https://stackoverflow.com/questions/29858543/elevation-angle-between-positions
    phi = d/R
    d1 = (R + c1.alt)*cos(phi)
    d3 = (R + c1.alt)*sin(phi)
    d2 = c2.alt - c1.alt*cos(phi) + R*(1 - cos(phi))
    elevation = degrees(atan2(c2.alt - c1.alt*cos(phi) + R*(1 - cos(phi)), (R + c1.alt)*sin(phi)))

    return d, azimuth, elevation