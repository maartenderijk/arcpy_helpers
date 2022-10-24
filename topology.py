"""
Helper functions
For different line topology problems

Maarten de Rijk
"""
import arcpy
import math
from datetime import datetime


def objectids_touch_feature(geometry, compare_feature):
    # Returns a list of objectids of the compare_feature that touch the a geometry
    oids = []
    for row in arcpy.da.SearchCursor(compare_feature, ["SHAPE@", "OID@"]):
        if row[0].touches(geometry):
            oids.append(row[1])
    return oids


def get_geometry_by_objectid(feature, oid):
    # Returns geometry object from a featureclass based on objectid
    base_geom = None
    for base in arcpy.da.SearchCursor(feature, ["OID@", "SHAPE@"], "OBJECTID = {}".format(str(oid))):
        base_geom = base[1]
    return base_geom


def get_attribute_by_objectid(feature, oid, field):
    # Returns attribute from a featureclass based on objectid
    attribute = None
    for attr in arcpy.da.SearchCursor(feature, ["OID@", field], "OBJECTID = {}".format(str(oid))):
        attribute = attr[1]
    return attribute


def touching_features(in_features, geometry):
    # Returns a list of object id's from in_features that touch the geometry
    oids = []
    with arcpy.da.SearchCursor(in_features, ["OID@", "SHAPE@"]) as cursor:
        for row in cursor:
            if geometry.touches(row[1]):
                oids.append(row[0])
    return oids


def overlapping_features(in_features, geometry):
    # Returns a list of object id's from in_features that touch the geometry
    oids = []
    with arcpy.da.SearchCursor(in_features, ["OID@", "SHAPE@"]) as cursor:
        for row in cursor:
            if geometry.overlaps(row[1]):
                oids.append(row[0])
    return oids


def count_intersects(geometry, feature):
    # Count the number of intersects of a geometry with a feature class
    no_intersects = 0
    with arcpy.da.SearchCursor(feature, ["OID@", "SHAPE@"]) as cursor:
        for row in cursor:
            if row[1].touches(geometry):
                no_intersects += 1

    return no_intersects


def point_from_line(line_geometry, endpoint="FIRST"):
    # Returns a point geometry from a line geometry
    if endpoint.upper() == "FIRST":
        return arcpy.PointGeometry(line_geometry.firstPoint, line_geometry.spatialReference)
    elif endpoint.upper() == "LAST":
        return arcpy.PointGeometry(line_geometry.lastPoint, line_geometry.spatialReference)
    else:
        raise ValueError("Function input should be FIRST or LAST")


def angle_line(line_geometry):
    # Returns the angle of a line geometry in degrees
    last_point = point_from_line(line_geometry, "LAST")
    first_point = point_from_line(line_geometry, "FIRST")
    try:
        radian = math.atan((last_point.X - first_point.X) /
                           (last_point.Y - first_point.Y))
    except ZeroDivisionError:
        radian = 360
    degrees = radian * 180 / math.pi
    return degrees


def flip_line(shape):
    line_part = shape.getPart(0)
    reverse_points = arcpy.Array()

    for i in range(len(line_part)):
        reverse_points.append(line_part[len(line_part) - i - 1])

    reverse_polyline = arcpy.Polyline(reverse_points)
    return reverse_polyline


def create_parallel(line_geometry, shift_distance):
    part = line_geometry.getPart(0)
    parallel_array = arcpy.Array()
    # rArray=arcpy.Array()

    for line_vertex in part:
        distance_on_line = line_geometry.measureOnLine(line_vertex)
        point_1 = line_geometry.positionAlongLine(
            distance_on_line - 0.01).firstPoint
        point_2 = line_geometry.positionAlongLine(
            distance_on_line + 0.01).firstPoint

        delta_x = float(point_2.X) - float(point_1.X)
        delta_y = float(point_2.Y) - float(point_1.Y)
        distance_x_y = math.hypot(delta_x, delta_y)

        shifted_x = -delta_y * shift_distance / distance_x_y
        shifted_y = delta_x * shift_distance / distance_x_y

        shifted_point = arcpy.Point(
            line_vertex.X + shifted_x, line_vertex.Y + shifted_y)
        parallel_array.add(shifted_point)

        # rightP=arcpy.Point(line_vertex.X-shifted_x, line_vertex.Y-shifted_y)
        # rArray.add(rightP)
    # array = arcpy.Array([parallel_array, rArray])

    parallel_geometry = arcpy.Polyline(parallel_array)
    return parallel_geometry


def collapse_roads(shape_a, shape_b):
    """
    Returns one collapsed line feature replacing to sides with perpendicular direction
    :param shape_a:
    :param shape_b:
    :return: one polyline feature
    """

    # tests
    if shape_a.partCount != 1 or shape_b.partCount != 1:
        raise Exception('Multipart line features are not supported')

    # if shape_b.lastPoint.X != shape_a.firstPoint.X or shape_b.lastPoint.Y != shape_a.firstPoint.Y:
    #     raise Exception('Start/ends of both lines do not coincide')

    # Assumption: both polylines have one parts and are in opposite direction
    points_a, points_b = list(shape_a.getPart(0)), list(shape_b.getPart(0))
    points_b.reverse()

    # Different point counts; alternate approach
    if shape_a.pointCount != shape_b.pointCount:
        if len(points_a) < len(points_b):
            # Use b as a reference
            dists = [shape_b.measureOnLine(p) for p in points_b][1:-1]
            points_a = [points_a[0]] + \
                       [shape_a.positionAlongLine(abs(shape_a.getLength() - dist)).firstPoint for dist in dists] + \
                       [points_a[-1]]
        else:
            # Use a as a reference
            dists = [shape_a.measureOnLine(p) for p in points_a][1:-1]
            points_b = [points_b[0]] + \
                       [shape_b.positionAlongLine(abs(shape_b.getLength() - dist)).firstPoint for dist in dists] + \
                       [points_b[-1]]

    # This will only work if a and b have equal length
    pnt_list = [(points_a[i], points_b[i]) for i in range(len(points_a))]
    array_c = arcpy.Array([midpoint(*points) for points in pnt_list])

    # Return collapsed geometry
    return arcpy.Polyline(array_c)


def midpoint(p1, p2):
    """
    Returns the center between to cartesian points
    :param p1: arcpy.Point
    :param p2: arcpy.Point
    :return: arcpy.Point
    """
    return arcpy.Point(p1.X + (p2.X - p1.X) / 2., p1.Y + (p2.Y - p1.Y) / 2.)


def log(string):
    log_message = datetime.now().strftime('%Y%m%d %H:%M:%S') + ": " + str(string)
    arcpy.AddMessage(log_message)
    # print(log_message)


def quadrant(shape):
    north_azimuth = azimuth(shape)
    if (north_azimuth >= 45) & (north_azimuth < 135):
        quad = "O"
    elif (north_azimuth >= 135) & (north_azimuth < 225):
        quad = "Z"
    elif (north_azimuth >= 225) & (north_azimuth < 315):
        quad = "W"
    else:
        quad = "N"
    return quad


def azimuth(shape):
    deg = 90 - math.degrees(
        math.atan2((shape.lastPoint.Y - shape.firstPoint.Y), (shape.lastPoint.X - shape.firstPoint.X)))
    if deg < 0:
        return deg + 360
    else:
        return deg

def get_length_intersect_geometries(geometry_a, geometry_b):
    intersect = geometry_a.intersect(geometry_b, 2)
    return intersect.getLength("GEODESIC", "METERS")


def get_oid_of_longest_border(fc, original_oid):
    geometry = get_geometry_by_objectid(fc, original_oid)
    touching_polygon_ids = objectids_touch_feature(geometry, fc)
    intersect_lengths = []
    for oid in touching_polygon_ids:
        compare_geometry = get_geometry_by_objectid(fc, oid)
        intersect_lengths.append((get_length_intersect_geometries(geometry, compare_geometry), oid))
    if len(intersect_lengths) > 0:
        return sorted(intersect_lengths, key=lambda s: s[0], reverse=True)[0][1]
    else:
        return None