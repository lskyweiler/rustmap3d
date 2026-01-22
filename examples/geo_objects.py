import rustmap3d
import math


# Construct a GeoPosition from either global or local coordinates
reference = rustmap3d.GeoPosition.from_lla((0.0, 0.0, 0.0))
rustmap3d.GeoPosition.from_enu(rustmap3d.DVec3(100.0, 0.0, 0.0), reference)
rustmap3d.GeoPosition.from_ned(rustmap3d.DVec3(100.0, 0.0, 0.0), reference)
pos = rustmap3d.GeoPosition.from_aer(
    rustmap3d.DVec3(90.0, 0.0, 100.0), (0.0, 0.0, 0.0)
)  # all reference locations accept LLA tuples or GeoPositions for reference locations

# Conversions
reference.aer_to(pos)
reference.ned_to(pos)
reference.enu_to(pos)

# Operations
vec = pos - reference  #> GeoVector
new_pos = reference + vec  #> GeoPosition

vel = rustmap3d.GeoVelocity.from_dir_speed(rustmap3d.DVec3(1.0, 0.0, 0.0), 100.0)
dt = 1.0
pos = reference + vel * dt  #> GeoPosition

rotation = rustmap3d.GeoOrientation.from_ecef_euler(rustmap3d.DVec3(0.0, math.pi, 0.0))
vec = rustmap3d.GeoVector.from_ecef(rustmap3d.DVec3(100.0, 0, 0.0), reference)
vec = rotation * vec

# Orientations
rot = rustmap3d.GeoOrientation.from_axis_angle(rustmap3d.DVec3(0., 0., 1), math.pi)
rot.forward()  #> ecef x axis
rot.left() #> ecef y axis
rot.up() #> ecef y axis