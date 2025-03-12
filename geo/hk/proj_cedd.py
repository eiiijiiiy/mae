import numpy as np
import open3d as o3d
from PIL import Image


def proj_cedd(bdlg_ply, veg_ply, ground_ply, png_path):
    # Load the point clouds
    bdlg_pcd = o3d.io.read_point_cloud(bdlg_ply)
    center = bdlg_pcd.get_center()
    bdlg_pcd.translate(-center)
    veg_pcd = o3d.io.read_point_cloud(veg_ply)
    veg_pcd.translate(-center)
    ground_pcd = o3d.io.read_point_cloud(ground_ply)
    ground_pcd.translate(-center)

    # Get the bounding box of the point clouds
    min_x = min(bdlg_pcd.get_min_bound()[0], veg_pcd.get_min_bound()[0], ground_pcd.get_min_bound()[0])
    min_y = min(bdlg_pcd.get_min_bound()[1], veg_pcd.get_min_bound()[1], ground_pcd.get_min_bound()[1])
    min_z = min(bdlg_pcd.get_min_bound()[2], veg_pcd.get_min_bound()[2], ground_pcd.get_min_bound()[2])
    max_x = max(bdlg_pcd.get_max_bound()[0], veg_pcd.get_max_bound()[0], ground_pcd.get_max_bound()[0])
    max_y = max(bdlg_pcd.get_max_bound()[1], veg_pcd.get_max_bound()[1], ground_pcd.get_max_bound()[1])
    max_z = max(bdlg_pcd.get_max_bound()[2], veg_pcd.get_max_bound()[2], ground_pcd.get_max_bound()[2])
    delta_x = max_x - min_x
    delta_y = max_y - min_y
    delta_z = max_z - min_z

    # Create a meshgrid
    bldg_channel = np.zeros((224, 224))
    veg_channel = np.zeros((224, 224))
    ground_channel = np.zeros((224, 224))

    # Project the point clouds to the meshgrid
    # bdlg_pcd = np.asarray(bdlg_pcd.points)
    # veg_pcd = np.asarray(veg_pcd.points)
    # ground_pcd = np.asarray(ground_pcd.points)

    # bdlg_pcd[:, 0] = (bdlg_pcd[:, 0] - min_x) / delta_x * 224
    # bdlg_pcd[:, 1] = (bdlg_pcd[:, 1] - min_y) / delta_y * 224
    # bdlg_pcd[:, 2] = (bdlg_pcd[:, 2] - min_z) / delta_z 
    
    for i in range(len(bdlg_pcd.points)):
        x = int((bdlg_pcd.points[i][0] - min_x) / delta_x * 224)
        y = int((bdlg_pcd.points[i][1] - min_y) / delta_y * 224)
        x = np.clip(x, 0, 223)
        y = np.clip(y, 0, 223)
        z = float(bdlg_pcd.points[i][2] - min_z) / delta_z * 225
        bldg_channel[x][y] = z
    
    for i in range(len(veg_pcd.points)):
        x = int((veg_pcd.points[i][0] - min_x) / delta_x * 224)
        y = int((veg_pcd.points[i][1] - min_y) / delta_y * 224)
        x = np.clip(x, 0, 223)
        y = np.clip(y, 0, 223)
        z = float(veg_pcd.points[i][2] - min_z) / delta_z * 225
        veg_channel[x][y] = z
    
    for i in range(len(ground_pcd.points)):
        x = int((ground_pcd.points[i][0] - min_x) / delta_x * 224)
        y = int((ground_pcd.points[i][1] - min_y) / delta_y * 224)
        x = np.clip(x, 0, 223)
        y = np.clip(y, 0, 223)
        z = float(ground_pcd.points[i][2] - min_z) / delta_z * 225
        ground_channel[x][y] = z
    
    img = np.stack((bldg_channel, veg_channel, ground_channel), axis=2)
    img = img.astype(np.uint8)
    # save the image
    img = Image.fromarray(img)
    img.save(png_path)
    
    return bdlg_ply, veg_ply, ground_ply

# folder = '/Users/yijie/Documents/download-f574b62a-57f1-40bb-aed3-df231748394e/ply/'
# folder = '/Users/yijie/Documents/download-a0ace6ed-3668-4701-92bd-29bc09e7e16b/ply/'
# folder = '/Users/yijie/Documents/download-a65f96aa-b085-415c-9b73-1688b58c3771/ply/'
# folder = '/Users/yijie/Documents/download-9603b8eb-e4f8-4312-b909-1f9ff331556a/ply/'
# folder = '/Users/yijie/Documents/download-aafc70d7-1560-4586-8c3d-95c17d5635df/ply/'
# folder = '/Users/yijie/Documents/download-718dadd3-6d4f-4506-a2a1-560918387be5/ply/'
# folder = '/Users/yijie/Documents/download-6f0c76fa-5b5f-4fe4-a8ff-364d3e5c4afb/ply/'
# folder = '/Users/yijie/Documents/download-6f0c76fa-5b5f-4fe4-a8ff-364d3e5c4afb/ply/'
folder = '/Users/yijie/Documents/download-1f65056d-2aac-44dd-91cb-5c286ae993e5/ply/'
bldg_path = folder + 'building.ply'
veg_path = folder + 'vegetation.ply'
ground_path = folder + 'ground.ply'
png_path = folder + 'proj_cedd.png'
proj_cedd(bldg_path, veg_path, ground_path, png_path)