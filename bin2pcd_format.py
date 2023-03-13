import numpy as np
import os 
import struct
import open3d as o3d
 


path = "/home/parvez/Desktop/Autera/20230228_121348_RecFile_1@20230228_121348746329-20230307T092815Z-001/20230228_121348_RecFile_1@20230228_121348746329/hdl_64e_s2_convert_to_XYZ_1_XYZ_points/RecFile_1_20230228_121348@20230228_121348746329_hdl_64e_s2_convert_to_XYZ_1_XYZ_points.idx"


bin_file_path = "/home/parvez/Desktop/Autera/20230228_121348_RecFile_1@20230228_121348746329-20230307T092815Z-001/20230228_121348_RecFile_1@20230228_121348746329/hdl_64e_s2_convert_to_XYZ_1_XYZ_points/RecFile_1_20230228_121348@20230228_121348746329_hdl_64e_s2_convert_to_XYZ_1_XYZ_points.bin"



for i in range(10):
   list = []
   with open(path, 'rb') as f:
      f.seek(i*8, 0)
      start_pos = struct.unpack('<Q', f.read(8))[0]
      f.seek((i+1) * 8, 0)
      end_pos = struct.unpack('<Q', f.read(8))[0]
      size = int(( end_pos - start_pos ) / 8) 

      

      with open(bin_file_path, "rb") as fb:
         fb.seek(start_pos, 0)
   
         for k in range(size):
              list.append(struct.unpack("<d", fb.read(8))[0])
   N = int(len(list) / 3) 
   points = np.array(list).reshape((N, 3))
      
   pcd = o3d.t.geometry.PointCloud()
      
   pcd.point["positions"] = o3d.core.Tensor(points)
   
   path_pcd = "/home/parvez/Desktop/Autera/"+str(i)+"pointcloud.pcd"  
   o3d.t.io.write_point_cloud(path_pcd, pcd)
      
   
      
      
      






#pcd = o3d.geometry.PointCloud()
#pcd.points = o3d.utility.Vector3dVector(points)

#print(pcd)
#print(np.asarray(pcd.points))
#o3d.visualization.draw_geometries([pcd])



#geom=o3d.geometry.PointCloud()
#geom.points=o3d.utility.Vector3dVector(points)

# Set background color and point size and visualize the point cloud
#o3d.visualization.draw_geometries([geom], point_show_normal=False, background=[0, 0, 0], point_size=2)
  
   



   
   
   
   
   
   
