




def whatPiece(corners):
    xy = zip(*corners)
    xs = xy[0]
    ys = xy[1]
    xmin = min(xs)
    xmax = max(xs)
    ymin = min(ys)
    ymax = max(ys)
    width = xmax - xmin
    height = ymax - ymin

    # Scale the pieces to combinations of four unit squares. (Blocks must be pretty much orthogonal)
    scalefactor = 15
    slop = 5
    corners = [( (x - xmin+slop)/scalefactor, (y - ymin + 5)/scalefactor) for x, y in corners]
    corners.sort()

    			# Do so many unit tests on these oh my gawd
			# Make this output orientation (from basicTetris)
    if len(corners) == 4:	#I or O
        if corners == [(0,0),(0,1),(4,0),(4,1)] or [(0,0),(0,4),(1,0),(1,4)]:
            piece = 'i'
        if corners == [(0,0),(0,2),(2,0),(2,2)]:
            piece = 'o'

    if len(corners) == 6:	#L or J
        if corners == [(0,0),(0,3),(1,0),(1,2),(2,2),(2,3)] or [(0,1),(0,2),(2,0),(2,1),(3,0),(3,2)] or [(0,0),(0,1),(1,1),(1,3),(2,0),(2,3)] or [(0,0),(0,2),(1,1),(1,2),(3,0),(3,1)]:
            piece = 'l'
        if corners == [(0,2),(0,3),(1,0),(1,2),(2,0),(2,3)] or [(0,0),(0,1),(2,1),(2,2),(3,0),(3,2)] or [(0,0),(0,3),(1,1),(1,3),(2,0),(2,1)] or [(0,0),(0,2),(1,0),(1,1),(3,1),(3,2)]:
            piece = 'j'

    if len(corners) >= 8:	#T, S or Z
        if corners == [(0,0),(0,1),(1,1),(1,2),(2,0),(2,1),(3,1),(3,2)] or [(0,1),(0,3),(1,0),(1,1),(1,2),(1,3),(2,0),(2,2)]:
            piece = 'z'
        if corners == [(0,1),(0,2),(1,0),(1,1),(2,1),(2,2),(3,0),(3,1)] or [(0,0),(0,2),(1,0),(1,1),(1,2),(1,3),(2,1),(2,3)]:
            piece = 's'

    return piece, 0



def pix_to_pos(CoM):
    # Remember that 
    pix_minx = 0	# Define the limits of the picture
    pix_maxx = 1000
    pix_miny = 0
    pix_maxy = 1000

    pos_minx = 0	# Corresponding arm positions
    pos_maxx = 10
    pos_miny = 2700
    pos_maxy = 7500
