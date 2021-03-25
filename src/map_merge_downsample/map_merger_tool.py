'''
this program provide 3 functions
1. pcd map convert frame utm to enu 
2. merge each small map to global map
3. downsample map uniformly

Need Library:
    numpy
    open3d
'''

import os
import numpy as np
import open3d
from tqdm import tqdm

# prefixes is list of map to combine
prefixes=[
        "Track_A_20201223_231551_Profiler_zfs_0_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_1_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_2_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_3_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_4_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_5_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_6_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_7_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_8_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_9_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_10_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_11_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_12_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_13_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_14_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_15_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_16_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_17_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_18_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_19_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_20_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_21_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_22_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_23_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_24_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_25_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_26_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_27_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_28_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_29_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_30_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_31_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_32_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_33_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_34_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_35_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_36_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_37_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_38_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_39_m_colorized",
        "Track_A_20201223_231551_Profiler_zfs_40_m_colorized"
    ]

def UTMtoLL(UTMNorthing, UTMEasting, UTMZone):
    m_RAD2DEG = 180/np.pi
    m_DEG2RAD = np.pi/180
    k0 = 0.9996
    a = 6378137.0
    WGS84_E = 0.0818191908
    eccSquared = (WGS84_E*WGS84_E)
    e1 =  (1 - np.sqrt(1 - eccSquared)) / (1 + np.sqrt(1 - eccSquared))

    x = UTMEasting - 500000.0
    y = UTMNorthing

    ZoneNumber = 52
    ZoneLetter = 'N'

    # if((ZoneLetter - 'N') < 0):
    # y = y - 10000000.0

    LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3
    eccPrimeSquared = (eccSquared) / (1 - eccSquared)

    M = y / k0
    mu = M / (a*(1 - eccSquared / 4 - 3 * eccSquared*eccSquared / 64 - 5 * eccSquared*eccSquared*eccSquared / 256))
    
    phi1Rad = mu + ((3 * e1 / 2 - 27 * e1*e1*e1 / 32)*np.sin(2 * mu) + (21 * e1*e1 / 16 - 55 * e1*e1*e1*e1 / 32)*np.sin(4 * mu) + (151 * e1*e1*e1 / 96)*np.sin(6 * mu))

    N1 = a / np.sqrt(1 - eccSquared*np.sin(phi1Rad)*np.sin(phi1Rad))
    T1 = np.tan(phi1Rad)*np.tan(phi1Rad)
    C1 = eccPrimeSquared*np.cos(phi1Rad)*np.cos(phi1Rad)
    R1 = a*(1 - eccSquared) / ((1 - eccSquared*np.sin(phi1Rad)*np.sin(phi1Rad))**1.5)
    D = x / (N1*k0)

    Lat = phi1Rad - ((N1*np.tan(phi1Rad) / R1) *(D*D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1*C1 - 9 * eccPrimeSquared)*D*D*D*D / 24 + (61 + 90 * T1 + 298 * C1 + 45 * T1*T1 - 252 * eccPrimeSquared - 3 * C1*C1)*D*D*D*D*D*D / 720))
    Lat = Lat * m_RAD2DEG

    Long = ((D - (1 + 2 * T1 + C1)*D*D*D / 6 + (5 - 2 * C1 + 28 * T1 - 3 * C1*C1 + 8 * eccPrimeSquared + 24 * T1*T1)*D*D*D*D*D / 120) / np.cos(phi1Rad))
    Long = LongOrigin + Long * m_RAD2DEG

    return ([Lat, Long])

def FnKappaLat(ref_lat, height):
    Geod_a = 6378137.0
    Geod_e2 = 0.00669437999014
    m_RAD2DEG = 180/np.pi
    m_DEG2RAD = np.pi/180

    # dKappaLat=0.
    # Denominator = 0.
    # dM = 0.

    # estimate the meridional radius
    Denominator = np.sqrt(1 - Geod_e2 * (np.sin(ref_lat * m_DEG2RAD))**2)
    dM = Geod_a * (1 - Geod_e2) / (Denominator**3)

    # Curvature for the meridian
    dKappaLat = 1 / (dM + height) * m_RAD2DEG;

    return dKappaLat

def FnKappaLon(ref_lat, height):
    Geod_a = 6378137.0
    Geod_e2 = 0.00669437999014
    m_RAD2DEG = 180/np.pi
    m_DEG2RAD = np.pi/180
    dKappaLon = 0
    Denominator = 0
    dN = 0

    # estimate the normal radius
    Denominator = np.sqrt(1 - Geod_e2 * (np.sin(ref_lat * m_DEG2RAD))**2)
    dN = Geod_a / Denominator

    # Curvature for the meridian
    dKappaLon = 1 / ((dN + height) * np.cos(ref_lat * m_DEG2RAD)) * m_RAD2DEG

    return dKappaLon

def llh2enu(lat, lon, height, ref_lat, ref_lon, ref_height):
    Geod_a = 6378137.0
    Geod_e2 = 0.00669437999014
    m_RAD2DEG = 180/np.pi
    m_DEG2RAD = np.pi/180
    dKLat = 0.
    dKLon = 0.

    m_dRefLatitude_deg = m_dRefLatitude_deg
    m_dRefLongitude_deg = ref_lon

    height = height - ref_height

    # estimate the meridional radius
    Denominator = np.sqrt(1 - Geod_e2 * (np.sin(m_dRefLatitude_deg * m_DEG2RAD))**2)
    dM = Geod_a * (1 - Geod_e2) / (Denominator**3)

    # Curvature for the meridian
    dKappaLat = 1 / (dM + height) * m_RAD2DEG;

    dKLon = 0
    Denominator = 0
    dN = 0

    # estimate the normal radius
    Denominator = np.sqrt(1 - Geod_e2 * (np.sin(m_dRefLatitude_deg * m_DEG2RAD))**2)
    dN = Geod_a / Denominator

    # Curvature for the meridian
    dKLon = 1 / ((dN + height) * np.cos(m_dRefLatitude_deg * m_DEG2RAD)) * m_RAD2DEG

    east_m = (lon-m_dRefLongitude_deg)/dKLon
    north_m = (lat-m_dRefLatitude_deg)/dKat


    return [east_m, north_m, height]



color_map = np.array(       ### bgr
    [[0, 0, 0],             ### unlabeled
    [0, 10, 255],           ### outlier
    [245, 150, 100],        ### car
    [245, 230, 100],        ### bicycle
    [250, 80, 100],         ### bus
    [150, 60, 30],          ### motorcycle
    [255, 0, 0],            ### on rails
    [180, 30, 80],          ### truck
    [200, 40, 255],         ### bicyclist
    [90, 30, 150],          ### motorcyclist
    [255, 0, 255],          ### road
    [255, 150, 255],        ### parking
    [75, 0, 75],            ### sidewalk
    [75, 0, 175],           ### other-ground
    [0, 200, 255],          ### building
    [50, 120, 255],         ### fence
    [0, 150, 255],          ### other-structure
    [170, 255, 150],        ### lane-marking
    [0, 175, 0],            ### vegetation
    [0, 60, 135],           ### trunk
    [80, 240, 150],         ### terrain
    [150, 240, 255],        ### pole
    [0, 0, 255],            ### traffic-sign
    [255, 255, 50],         ### other-object
    [184, 249, 7],          ### other-building
    [142, 135, 31],         ### kerb
    [255, 0, 0],            ### traffic-light
    [80, 100, 0],           ### tunnel-fan
    [137, 95, 174],         ### tunnel-emergency-light
    [255, 0, 171]]          ### tunnel-hydrant
)


data_path = "/home/jiwon/pcd_colorized_sy"
is_first_step = True
pcd_npy_edit = np.array([0]);
cnt = 0
for prefix in prefixes:
    print("colorizing", prefix)
    prefix = os.path.join(data_path, prefix)
    pcd_file = prefix + ".pcd" 
    # label_file = prefix + ".labels" 
    saved_file = prefix + "_global2.pcd"
    
    pcd = open3d.io.read_point_cloud(pcd_file)   #### load .pcd file       

    pcd_down = pcd.uniform_down_sample(every_k_points=20)
    
    pcd_npy = np.asarray(pcd_down.points)
    pcd_cor = np.asarray(pcd_down.colors)
    
    # print(pcd_npy)

    for idx in tqdm(range(pcd_npy.shape[0])):
        [lat, lon] = UTMtoLL(pcd_npy[idx,1],pcd_npy[idx,0],"52N")
        
        # Set reference position using first position of first file
        if is_first_step == True:
            # reference_lat = reference point latitude of map
            reference_lat = 37.3962732790
            # reference_lon = reference point longitude of map
            reference_lon = 127.1066872418
            # reference_height = reference point height of map
            reference_height = 60.37
            is_first_step = False
        pcd_npy[idx,0:3] = llh2enu(lat, lon, pcd_npy[idx,2], reference_lat, reference_lon, reference_height)

    if cnt ==0:
        pcd_sum = np.concatenate((pcd_npy,pcd_cor),axis=1)
    else:
        tmp_pcd_sum = np.concatenate((pcd_npy,pcd_cor),axis=1)
        pcd_sum = np.concatenate((pcd_sum,tmp_pcd_sum),axis=0)
    
    cnt = cnt + 1
        



pcd_final = open3d.geometry.PointCloud()
pcd_final.points = open3d.utility.Vector3dVector(pcd_sum[:, 0:3])    
pcd_final.colors = open3d.utility.Vector3dVector(pcd_sum[:, 3:6])    

open3d.io.write_point_cloud(saved_file, pcd_final)
print("\nreference_lat : ")
print(reference_lat)
print("\nreference_lon : ")
print(reference_lon)
print("\nreference_Height : ")
print(reference_height)

f = open(data_path+"/configuration.ini",'w')
f.write("[configuration]\n")
f.write("RefLatitude_deg = %f \n"%reference_lat)
f.write("RefLongitude_deg = %f \n"%reference_lon)
f.write("RefHeight_m = %f"%reference_height)