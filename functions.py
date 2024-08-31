def compute_polygon_centroid(pts):
    n = len(pts)

    x = 0
    y = 0

    for pt in pts:
        x += pt[0]
        y += pt[1]

    return [x/n, y/n]