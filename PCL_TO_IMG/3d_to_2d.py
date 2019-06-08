#azimuth, elevation, in-plane rotation, distance, the field of view
#https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
#https://en.wikipedia.org/wiki/3D_projection (see perspective projection)

#the point clouds are written to output file rotate.pcd, so simply view this to see that the point
#cloud is indeed being rotated 
 
from mpl_toolkits.mplot3d import Axes3D

import math
import matplotlib.pyplot as plt
import numpy as np



camera_pos_file = open('metadata', 'r')

for i in range(1):
	phi, theta, zero, r, field = [x for x in camera_pos_file.readline().split(' ')] # read first line

r = float(r)
theta = float(theta)
phi = float(phi)
# # inclidation is theta azimuth is phi
print("phi->", phi, " theta->", theta, " r->", r)

 
source_pcd = open('model_normalized.pcd', 'r')
rotated_pcd = open('rotated.pcd', 'w')
image_snapshot = open('multiview_image.pcd', 'w')



F_MM = 35.  # Focal length
SENSOR_SIZE_MM = 32.

RESOLUTION_PCT = 100
CAM_MAX_DIST = 1.75

IMG_W = 127 + 10  # Rendering image size. Network input size + cropping margin.
IMG_H = 127 + 10


CAM_ROT = np.matrix(((1.910685676922942e-15, 4.371138828673793e-08, 1.0),
                     (1.0, -4.371138828673793e-08, -0.0),
                     (4.371138828673793e-08, 1.0, -4.371138828673793e-08)))



#the original version of this function from occupancy was quite poorly written, though the logic was sound
# see this paper  http://www.cs.uns.edu.ar/cg/clasespdf/p465carlbom.pdf
#we had to change the variable names using the more intuitive and accurate explanation from this video
#https://www.youtube.com/watch?v=iMZdNIWygHw&list=LLXUyxYccmOdGdbyxiGTtOSQ&index=7
def getkrt(az, el, distance_ratio, img_w=IMG_W, img_h=IMG_H):
    """Calculate 4x3 3D to 2D projection matrix given viewpoint parameters."""

    # Calculate intrinsic matrix.
    scale = RESOLUTION_PCT / 100
    f_u = F_MM * img_w / SENSOR_SIZE_MM
    f_v = F_MM * img_h / SENSOR_SIZE_MM
    u_0 = img_w * scale / 2
    v_0 = img_h * scale / 2
    K = np.matrix(((f_u, 0, u_0), (0, f_v, v_0), (0, 0, 1)))

    # Calculate rotation and translation matrices.
    # Step 1: World coordinate to object coordinate.
    sa = np.sin(np.radians(-az))
    ca = np.cos(np.radians(-az))
    se = np.sin(np.radians(-el))
    ce = np.cos(np.radians(-el))
    R_world2cam = np.transpose(np.matrix(((ca * ce, -sa, ca * se),
                             	              (sa * ce, ca, sa * se),
                                          (-se, 0, ce))))

    # Step 2: Object coordinate to camera coordinate.
    R_cam2screen = np.transpose(CAM_ROT)
    R_world2screen = R_cam2screen * R_world2cam
    cam_location = np.transpose(np.matrix((distance_ratio * CAM_MAX_DIST,
                                           0,
                                           0)))
    T_cam2screen = -1 * R_cam2screen * cam_location

    RT = np.hstack((R_cam2screen, T_cam2screen))

    return K, RT

K, RT = getkrt(phi, theta, r)


#declare some arrays that can be used to visualise the point cloud from pyplot 
#also make use of lidar view to visualise the final point cloud 


# x_input = []
# y_input = []
# z_input = []
# x_rotated = []
# y_rotated = []
# z_rotated = []

x_img = []
y_img = []


check = 0
#loop runs through each of the points of point cloud A and translates and rotates it to its new position in point cloud B
while ( 1 ):

	#a little loop to skip the medata in the first 11 lines and directly translate them over
	if(check == 0):
		for i in range(11):
			ln = source_pcd.readline() 
			rotated_pcd.write(ln)
		check = 1

	#now actually take each line, which is an individual point, and calculate its new location in the new point cloud
	ln = source_pcd.readline().split(' ')
	if (len(ln) < 3):
		break
	ax, ay, az = [x for x in ln] # read first line
	ax, ay, az = float(ax), float(ay), float(az)
	# x_input.append(ax)
	# y_input.append(ay)
	# z_input.append(az)


	A = np.asarray([[ax], [ay], [az], [1]])

	#calulate point B from point A by multiplyin by K and RT
	B = np.dot(  K, (np.dot(RT, A )) )

	#the rotated and traslated version of each given point A
	bx = float(B[0])
	by = float(B[1])
	bz = float(B[2])

	#add the coordinates to the lists, to be later added to point clouds
	# x_rotated.append(bx)
	# y_rotated.append(by)
	# z_rotated.append(bz)

	#write the line of points to the final pointcloud
	#note that mutliplying by k and rt actually gives a 
	# 3d projection and you just need to use the formula jia found to get 
	#the perspective projection on the image plane
	#see https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/building-basic-perspective-projection-matrix
	#and  https://en.wikipedia.org/wiki/3D_projection#Weak_perspective_projection
	rotated_pcd.write( str(bx) + ' ' + str(by) + ' ' + str(bz) + '\n' ) 

	#having written to the rotated point cloud, we can now write to the perspective projection on image plane
	#from here on is still experimental- it appears that the generated 
	#plot is an upside down (in z , not y) version of the corresponding image
	x_img.append(-bx)
	y_img.append(-by)

#convert the lists to arrays and visualise with pyplot
x_img = np.asarray(x_img)
y_img = np.asarray(y_img)
plt.scatter(x_img, y_img)
plt.show()


# x_input = np.asarray(x_input)
# y_input = np.asarray(y_input)
# z_input = np.asarray(z_input)
# x_rotated = np.asarray(x_rotated)
# y_rotated = np.asarray(y_rotated)
# z_rotated = np.asarray(z_rotated)


# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(x_input, y_input, z_input, zdir='z', c='red')
# ax.scatter(x_rotated, y_rotated, z_rotated, zdir='z', c='blue')

# plt.savefig("demo.png")

