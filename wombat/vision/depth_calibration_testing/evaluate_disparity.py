import numpy as np
import matplotlib.pyplot as plt

# load the saved disparity map
disparity = np.load("disparity_guided.npy")
disparity_clipped = np.clip(disparity, a_min=0.5, a_max=None)

# crop the disparity to remove the bars
offset_x = 0
offset_y = 0

cropped_disparity = disparity_clipped[offset_y:,offset_x:]
print(f'cropped_disparity: {cropped_disparity}')

plt.figure()
plt.imshow(cropped_disparity, cmap='jet')
plt.colorbar()

# compute the depth map
f = 445.57261062314484
u0 = 438.61173157687153
v0 = 261.0113152340737
baseline = 5.976137772117957

depth = np.clip(f*baseline / cropped_disparity, a_min=0, a_max=None)
print(f'depth: {depth}')

# plots with min and max depths considered
min_depth = 5 # cm
max_depth = 200 # cm

plt.figure()
plt.imshow(np.clip(depth, a_min=min_depth, a_max=max_depth), cmap='jet')
plt.colorbar()

# compute the points

height, width = disparity_clipped.shape

u_vals = np.tile(np.arange(width)+1,(height,1))
v_vals = np.tile((np.arange(height)+1).reshape(height,1),(1,width))

z = depth.flatten()
x = (depth / f * (u_vals[offset_y:,offset_x:] - u0)).flatten()
y = (depth / f * (v_vals[offset_y:,offset_x:] - v0)).flatten()

# perform randomized downsampling (cutoff using a uniformly random distribution)
downsample_fraction = 0.05
downsample_mask = np.random.uniform(size=(len(z))) <= downsample_fraction

# removes depths that are not within range


disparity_cutoff_mask = np.logical_and(z <= max_depth, z >= min_depth)

idxs = np.logical_and(downsample_mask, disparity_cutoff_mask)

x_downsampled = x[idxs]
y_downsampled = y[idxs]
z_downsampled = z[idxs]

print(f'Original num points: {len(z)}')
print(f'Downsampled num points: {len(z_downsampled)}')

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter3D(x_downsampled,z_downsampled,-y_downsampled, c=z_downsampled, cmap='jet')
ax.set_xlabel('x')
ax.set_ylabel('-z')
ax.set_zlabel('y')

plt.show()
