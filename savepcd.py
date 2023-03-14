import open3d as o3d
import numpy as np

xyzi = np.random.rand(100, 4)

xyz = xyzi[:,0:3]
i = [[i] for i in xyzi[:,3]]

pcd = o3d.t.geometry.PointCloud()

pcd.point["positions"] = o3d.core.Tensor(xyz)
pcd.point["intensities"] = o3d.core.Tensor(i)

o3d.t.io.write_point_cloud("pointcloud.pcd", pcd)
