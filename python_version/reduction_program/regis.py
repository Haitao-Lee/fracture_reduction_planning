#!/usr/bin/env python
# coding: utf-8

# In[10]:

import numpy as np
import cycpd
import open3d as o3d

# In[11]:

x_src = np.load("source.npy")
x_tgt = np.load("target.npy")

# In[27]:

pcd1 = o3d.geometry.PointCloud()
pcd2 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(x_tgt)
pcd2.points = o3d.utility.Vector3dVector(x_src)
pcd1.paint_uniform_color([1., 0., 0.])
pcd2.paint_uniform_color([0., 0., 1.])
o3d.visualization.draw_geometries([pcd1, pcd2],
                                  window_name="",
                                  width=800,
                                  height=600,
                                  left=50,
                                  top=50,
                                  point_show_normal=False)

# In[12]:

print(x_src.shape)
print(x_tgt.shape)

# In[34]:

reg = cycpd.rigid_registration(
    **{
        'X': x_tgt,
        'Y': x_src,
        'max_iterations': 100,
        'tolerance': 1.0,
        'w': 0.5,
        'verbose': True,
        'print_reg_params': True,
        'scale': False
    })
tx_src, (s, r, t) = reg.register()

# In[36]:

pcd1 = o3d.geometry.PointCloud()
pcd2 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(x_tgt)
pcd2.points = o3d.utility.Vector3dVector(tx_src)
pcd1.paint_uniform_color([1., 0., 0.])
pcd2.paint_uniform_color([0., 0., 1.])
o3d.visualization.draw_geometries([pcd1, pcd2],
                                  window_name="",
                                  width=800,
                                  height=600,
                                  left=50,
                                  top=50,
                                  point_show_normal=False)

# In[33]:

print(s, r, t)

# In[ ]:
