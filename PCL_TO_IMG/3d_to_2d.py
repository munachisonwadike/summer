#azimuth, elevation, in-plane rotation, distance, the field of view
#https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
#https://en.wikipedia.org/wiki/3D_projection (see perspective projection)

#the point clouds are written to output file rotate.pcd, so simply view this to see that the point
#cloud is indeed being rotated 


from mpl_toolkits.mplot3d import Axes3D

import math
import matplotlib.pyplot as plt
import numpy as np
import sys

 
num = int(sys.argv[1])
camera_pos_file = open('metadata', 'r')
print(num)
count = -1



for i in range(num+1): #this will open up the parameters for the p'th image
	phi, theta, zero, r, field = [x for x in camera_pos_file.readline().split(' ')] # read first line
	count+=1

r = float(r)
theta = float(theta)
phi = float(phi)
# # inclidation is theta azimuth is phi
print("count->", count, "phi->", phi, " theta->", theta, " r->", r)

 
source_pcd = open('model_normalized.pcd', 'r')
rotated_pcd = open("rotated" + str(count) + ".pcd", 'w')
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


# CAM_ROT = np.matrix(((0, 0, -10.0),
#                      (2.0, 0, 0),
#                      (0, 2.0, 0)))



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

    RT = np.hstack((R_world2screen, T_cam2screen))

    # RT = np.hstack((R_cam2screen, np.asarray ([ [0],[0],[0] ]) ))

    return K, RT

K, RT = getkrt(phi, theta, r)

RT = np.array(

 
[[-9.93613886e-01,  3.81394245e-08, -1.12833707e-01, -2.34964172e-15],
 [ 5.51264006e-02, -8.72528327e-01, -4.85443508e-01,  5.37535308e-08],
 [-9.84506239e-02, -4.88563526e-01,  8.66956259e-01,  1.22973744e+00]]
 
# [[ 8.39559493e-01,  3.88904269e-08, -5.43267759e-01, -2.85119142e-15],
#  [ 2.48016746e-01, -8.89709260e-01,  3.83282047e-01,  6.52276494e-08],
#  [-4.83350341e-01, -4.56527582e-01, -7.46963864e-01,  1.49223468e+00]]
 
# [[ 8.51908344e-01,  3.79141233e-08, -5.23690915e-01, -3.17360311e-15],
#  [ 2.60618368e-01, -8.67374036e-01,  4.23957956e-01,  7.26035681e-08],
#  [-4.54235886e-01, -4.97656792e-01, -7.38923188e-01,  1.66097603e+00]]

# [[-8.09921933e-01,  3.85905179e-08, -5.86537691e-01, -3.16626592e-15],
#  [ 2.75472417e-01, -8.82848142e-01, -3.80386785e-01,  7.24357128e-08],
#  [-5.17823725e-01, -4.69658555e-01,  7.15038063e-01,  1.65713595e+00]]

# [[-2.41193397e-02,  3.79263633e-08, -9.99709086e-01, -2.70185936e-15],
#  [ 4.97023790e-01, -8.67654055e-01, -1.19914070e-02,  6.18113304e-08],
#  [-8.67401643e-01, -4.97168423e-01,  2.09272241e-02,  1.41407841e+00]]


# [[ 8.13509761e-01,  3.89267472e-08,  5.81551261e-01, -2.53452566e-15],
#  [-2.64550320e-01, -8.90540172e-01,  3.70069359e-01,  5.79831820e-08],
#  [ 5.17894774e-01, -4.54904608e-01, -7.24463112e-01,  1.32650058e+00]]


# [[ 7.36795294e-01,  3.88963746e-08, -6.76115888e-01, -2.32083817e-15],
#  [ 3.08486221e-01, -8.89845327e-01,  3.36171898e-01,  5.30945826e-08],
#  [-6.01638551e-01, -4.56262308e-01, -6.55633862e-01,  1.21466246e+00]]


# [[ 9.39473662e-01,  3.89448411e-08,  3.42621130e-01, -2.68282397e-15],
#  [-1.55581945e-01, -8.90954111e-01,  4.26608756e-01,  6.13758514e-08],
#  [ 3.05259721e-01, -4.54093352e-01, -8.37027915e-01,  1.40411581e+00]]
 

# [[ 5.21990453e-01,  3.82122172e-08,  8.52951328e-01, -2.26304040e-15],
#  [-4.14173944e-01, -8.74193631e-01,  2.53466840e-01,  5.17723238e-08],
#  [ 7.45644628e-01, -4.85577486e-01, -4.56320713e-01,  1.18441271e+00]]
 

# [[ 2.47736357e-01,  3.85283650e-08,  9.68827486e-01, -3.14214474e-15]
#  [-4.57598218e-01, -8.81426248e-01,  1.17011285e-01,  7.18838847e-08]
#  [ 8.53949980e-01, -4.72321680e-01, -2.18361310e-01,  1.64451159e+00]]


# [[ 9.53744441e-01,  3.94419971e-08,  3.00618596e-01, -3.02645366e-15]
#  [-1.29581818e-01, -9.02327715e-01,  4.11112207e-01,  6.92371816e-08]
#  [ 2.71256507e-01, -4.31050687e-01, -8.60590037e-01,  1.58396208e+00]]
 )


print(K, '\n', RT)

#declare some arrays that can be used to visualise the point cloud from pyplot 
#also make use of lidar view to visualise the final point cloud 


x_input = []
y_input = []
z_input = []
x_rotated = []
y_rotated = []
z_rotated = []

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
	x_input.append(ax)
	y_input.append(ay)
	z_input.append(az)


	A = np.asarray([[ax], [ay], [az], [1]])

	#calulate point B from point A by multiplyin by K and RT
	B = np.dot(  K, (np.dot(RT, A )) )

	#the rotated and traslated version of each given point A
	bx = float(B[0])
	by = float(B[1])
	bz = float(B[2])

	#add the coordinates to the lists, to be later added to point clouds
	x_rotated.append(bx)
	y_rotated.append(by)
	z_rotated.append(bz)

	#write the line of points to the final pointcloud
	rotated_pcd.write( str(bx) + ' ' + str(by) + ' ' + str(bz) + '\n' ) 

	#note that mutliplying by k and rt actually gives a 
	#3d projection and you just need to use the formula for 
	#perspective projection on the image plane
	#see https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/building-basic-perspective-projection-matrix
	#and  https://en.wikipedia.org/wiki/3D_projection#Weak_perspective_projection 
	x_img.append(bx/bz)
	y_img.append(by/bz)
	# print("bz =", bz)

# convert the lists to arrays and visualise with pyplot
x_img = np.asarray(x_img)
y_img = np.asarray(y_img)
img = plt.imread("/home/munachiso/summer/02691156/123ac29b0aac8c8e5d07633dda45110/rendering/00.png")
axScatter = plt.subplot(111)
axScatter.imshow(img)
axScatter.scatter(x_img, y_img, s=.2)
# set axes range
# plt.xlim(0, 140)
# plt.ylim(0, 140)
plt.show()



x_input = np.asarray(x_input)
y_input = np.asarray(y_input)
z_input = np.asarray(z_input)
x_rotated = np.asarray(x_rotated)
y_rotated = np.asarray(y_rotated)
z_rotated = np.asarray(z_rotated)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_input, y_input, z_input, c='red')
ax.scatter(x_rotated, y_rotated, z_rotated, c='blue')

plt.savefig("demo"+str(count)+".png")

