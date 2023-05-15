import numpy as np
import os 
import struct
import open3d as o3d
 

class NonBlockVisualizer:
    def __init__(self, point_size=2, background_color=[0, 0, 0]):
        self.__visualizer = o3d.visualization.Visualizer()
        self.__visualizer.create_window()
        opt = self.__visualizer.get_render_option()
        opt.background_color = np.asarray(background_color)
        opt = self.__visualizer.get_render_option()
        opt.point_size = point_size

        self.__pcd_vis = o3d.geometry.PointCloud()
        self.__initialized = False

    def update_renderer(self, pcd, wait_time=0):
        self.__pcd_vis.points = pcd.points
        #self.__pcd_vis.colors = pcd.colors

        if not self.__initialized:
            self.__initialized = True
            self.__visualizer.add_geometry(self.__pcd_vis)
        else:
            self.__visualizer.update_geometry(self.__pcd_vis)
        self.__visualizer.poll_events()
        self.__visualizer.update_renderer()

        if wait_time > 0:
            time.sleep(wait_time)
            
            
obj = NonBlockVisualizer()


















path = "/media/parvez/C00A-4B52/20230515_162516_RecFile_1@20230515_162516956711/hdl_64e_s2_convert_to_XYZ_1_XYZ_points/RecFile_1_20230515_162516@20230515_162516956711_hdl_64e_s2_convert_to_XYZ_1_XYZ_points.idx"
bin_file_path = "/media/parvez/C00A-4B52/20230515_162516_RecFile_1@20230515_162516956711/hdl_64e_s2_convert_to_XYZ_1_XYZ_points/RecFile_1_20230515_162516@20230515_162516956711_hdl_64e_s2_convert_to_XYZ_1_XYZ_points.bin"



for i in range(360):
   list = []
   with open(path, 'rb') as f:
      f.seek(i*8, 0)
      start_pos = struct.unpack('<Q', f.read(8))[0]
      f.seek((i+1) * 8, 0)
      end_pos = struct.unpack('<Q', f.read(8))[0]
      size = int(( end_pos - start_pos ) / 8) 

      with open(bin_file_path, "rb") as fb:
         fb.seek(start_pos, 0)
   
         for i in range(size):
            list.append(struct.unpack("<d", fb.read(8))[0])
      
      
   N = int(len(list) / 3) 
   points = np.array(list).reshape((N, 3))
   pcd = o3d.geometry.PointCloud()
   pcd.points = o3d.utility.Vector3dVector(points)
   
   obj.update_renderer(pcd)
   #print(pcd)
   #print(np.asarray(pcd.points))
   #o3d.visualization.draw_geometries([pcd])
   
   



#geom=o3d.geometry.PointCloud()
#geom.points=o3d.utility.Vector3dVector(points)

# Set background color and point size and visualize the point cloud
#o3d.visualization.draw_geometries([geom], point_show_normal=False, background=[0, 0, 0], point_size=2)
  
   
