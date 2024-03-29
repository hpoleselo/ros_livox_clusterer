#!/usr/bin/env python3
"""
HDBSCAN Clusterizer takes in a Point Cloud from ROS, clusterizes it by using HBDSCAN
algorithm and creates Bounding Boxes in RViz from the clustered objects.

Author: Henrique Poleselo
November 19th 2022.
"""

import numpy as np
from typing import List
from collections import deque
import random
import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from utils.helper_functions import get_number_of_labels, get_point_cloud_size, get_points_from_point_cloud
from utils.point_cloud_utils import downsample, get_pass_through_filter_boundaries, pass_through_filter
from utils.point_cloud_utils import cluster_hdbscan, ros_msg_to_numpy, numpy_to_o3d, numpy_to_ros_msg, clustering_dbscan, visualize_pcd

# Queue size should be <= to the frequency of publication of the topic in order to it to be async.
bounding_box_publisher = rospy.Publisher('cluster_bounding_boxes', BoundingBoxArray, queue_size=8)

rospy.init_node('livox_clusterer', anonymous=True)

# TODO: Use rosparam to get and set parameters
BOUNDING_BOX_FRAME_ID = 'cluster'
IS_TESTING_ONLINE = False
LIVOX_FRAME_ID = 'livox'

# TODO: improve performance for dealing with class
# TODO: https://stackoverflow.com/questions/44046920/changing-class-attributes-by-reference

class PointCloudCluster:
    """
    Class that describes data structure to represent a cluster to be colored in Open3D
    and to generate a bounding box for the cluster. Such bounding box will be published
    to RViz.
    HDBSCAN will output N clusters, therefore N classes should be created.
    """
    def __init__(self, 
                label=0,
                color_open3d = [],
                cluster_data_points = [],
                bounding_box = []):
        
        self.label = label   # Int that is represented by the label from a cluster outputted from HDBSCAN
        self.color_open3d = color_open3d
        self.cluster_data_points = cluster_data_points

        # Calls the default configuration for bounding boxes.
        self.bounding_box = self.configure_default_bounding_box_msg()

    def configure_default_bounding_box_msg(self):
        """
        Configures the ROS msg with default values, some of them
        are to be populated a posteriori.
        """
        default_bounding_box_msg = BoundingBox()

        # No rotation for the bounding box.
        default_bounding_box_msg.pose.orientation.w = 1
        default_bounding_box_msg.pose.orientation.z = 0.0
        default_bounding_box_msg.pose.orientation.y = 0.0
        default_bounding_box_msg.pose.orientation.x = 0.0

        # TODO: This will change to cluster_N dynamically afterwards so that each cluster has its frame_id
        default_bounding_box_msg.header.frame_id = BOUNDING_BOX_FRAME_ID

        default_bounding_box_msg.pose.position.x = 0.0
        default_bounding_box_msg.pose.position.y = 0.0
        default_bounding_box_msg.pose.position.z = 0.0

        # * Will be dynamically populated
        default_bounding_box_msg.dimensions.x = 0.0
        default_bounding_box_msg.dimensions.y = 0.0
        default_bounding_box_msg.dimensions.z = 0.0

        # * Will be dynamically populated
        default_bounding_box_msg.label = 0
        return default_bounding_box_msg

def generate_clusters(unique_labels: list) -> deque:
    """
    Based on the N unique labels identified from HDBSCAN clustering algorithm, N classes
    will be generated (along with colors) to be populated with the separeted points.
    In other words N different point clouds will be generated and then merged together.
    """

    clusters = deque()
    for label in unique_labels:
        # Generates Red, Green and Blue color channels for the Cluster color
        pcc = PointCloudCluster()
        pcc.label = label
        pcc.color_open3d = [
                            round(random.uniform(0.0, 1.0), 3),    # Red channel
                            round(random.uniform(0.0, 1.0), 3),    # Green channel
                            round(random.uniform(0.0, 1.0), 3)
                            ]
        pcc.cluster_data_points = []
        clusters.append(pcc)
    return clusters

def assign_data_points_to_each_cluster(
                                  point_cloud_clusters: List[PointCloudCluster],
                                  point_cloud,
                                  size_of_pc: int,
                                  unique_labels: List[int], 
                                  labels_from_hdbscan: List[List[float]]
                                  ):
    """
    Compares each data point XYZ from the original Point Cloud and its respective label from the point
    cloud and HDBSCAN output and assigns each data point to the correct cluster (to the correct Class).
    Each class (cluster) will be populated with their respective datapoints, other values are going to be 0. 
    """
    for i in range(size_of_pc):
        for label in unique_labels:
            if labels_from_hdbscan[i] == label:
                # Since we're expecting the labels to be ordered, we can reference the correct class/cluster id by the label itself
                # Stores the datapoint to the class
                point_cloud_clusters[label].cluster_data_points.append(point_cloud[i])

def calculate_bounding_box_dimensions_and_centroid(bounding_box_points):
    """
    Given the identified cluster max/min data points (i.e: vertices for the
    to-be drawn bounding box), calculates the X, Y and Z dimensions of the box
    in order to fit the clustered object.

    Such operation is needed because BoundingBox.msg from jsk_recognition_msgs.msg
    takes in such parameters in order to draw the box in RViz.
    """

    # z varies from z_max to z_min
    bbox_z_dimension = bounding_box_points[2] - bounding_box_points[1]
    # Only y varies from y_max to y_min
    bbox_y_dimension = bounding_box_points[3] - bounding_box_points[1]
    # Only x varies from x_max to x_min
    bbox_x_dimension = bounding_box_points[4] - bounding_box_points[1]

    # Calculates the centroid of the bounding box
    bbox_center_x = (bounding_box_points[2] + bounding_box_points[1])/2
    bbox_center_y = (bounding_box_points[3] + bounding_box_points[1])/2
    bbox_center_z = (bounding_box_points[4] + bounding_box_points[1])/2

    # Extracting values from the x, y and z component for each calculated compoenent
    bbox_origin = [bbox_center_x[0], bbox_center_y[1], bbox_center_z[2]]

    box_dimensions_xyz = [bbox_x_dimension[0], bbox_y_dimension[1], bbox_z_dimension[2]]
    return np.array(box_dimensions_xyz), np.array(bbox_origin)

def get_cluster_bounding_box_vertices_and_dimensions(cluster_point_cloud):
    """
    Gets each cluster min/max in order to draw each cluster a bounding box.
    This function will be used by both Open3D and RVIZ.

    Returns:
    - bounding_box_origin_points: XYZ point to represent where the initial point will be drawn.
    - box_dimensions_xyz: X, Y and Z dimensions of the bounding box.
    - bounding_box_points: vertices in order to validate if the bounding box is correct in Open3D.
    """
    #pc_points = np.asarray(cluster_point_cloud.points)

    cluster_point_cloud = np.asarray(cluster_point_cloud)

    x_max, y_max, z_max = cluster_point_cloud.max(axis=0)
    x_min, y_min, z_min = cluster_point_cloud.min(axis=0)

    # * Userd for Open3D in order to validate the bounding box vertices
    bounding_box_points = [
        [x_max, y_max, z_max],
        [x_min, y_min, z_min],
        [x_min, y_min, z_max],  # Varies only Z from the previous element, meaning we get Z
        [x_min, y_max, z_min],  # Varies only Y from the element 1, meaning we get Y
        [x_max, y_min, z_min]
    ]

    bounding_box_points = np.array(bounding_box_points)

    box_dimensions_xyz, bbox_center_point = calculate_bounding_box_dimensions_and_centroid(bounding_box_points)
    # Prior testing
    #bounding_box = cluster_point_cloud.get_axis_aligned_bounding_box()
    #print(f"\nBounding Box From Open3D: {np.asarray(bounding_box)}")

    return bbox_center_point, box_dimensions_xyz, bounding_box_points

def get_cluster_bounding_box_and_publish(point_cloud_clusters):
    """
    For each cluster get its bounding box parameters, updates the Point Cloud Cluster
    class with the bounding box information for the ROS message and publishes it.
    """
    bounding_box_array = BoundingBoxArray()
    bounding_box_array.header.frame_id = BOUNDING_BOX_FRAME_ID
    for point_cloud_cluster in point_cloud_clusters:
        # Retrieving min and max data points to draw bounding boxes in RViz
        bounding_box_origin_position, box_dimensions_xyz, _ = get_cluster_bounding_box_vertices_and_dimensions(point_cloud_cluster.cluster_data_points)

        # * Conditional to check if bounding box is zero, basically when we take the noise label -1
        # * in account...
        if (box_dimensions_xyz[0] == 0.0) or (box_dimensions_xyz[1] == 0.0) or (box_dimensions_xyz[2] == 0.0):
            rospy.logwarn(f"Bounding box for label{point_cloud_cluster.label} is empty. Ignoring data to be published to the topic.")
            continue

        # * Updates message with the bounding box information
        # * Updating each bounding box with a specific frame_id isn't needed.
        #point_cloud_cluster.bounding_box.header.frame_id = f'cluster_{point_cloud_cluster.label}'
        point_cloud_cluster.bounding_box.pose.position.x = bounding_box_origin_position[0]
        point_cloud_cluster.bounding_box.pose.position.y = bounding_box_origin_position[1]
        point_cloud_cluster.bounding_box.pose.position.z = bounding_box_origin_position[2]
        point_cloud_cluster.bounding_box.dimensions.x = box_dimensions_xyz[0]
        point_cloud_cluster.bounding_box.dimensions.y = box_dimensions_xyz[1]
        point_cloud_cluster.bounding_box.dimensions.z = box_dimensions_xyz[2]

        # * Since BoundingBox.label doesn't allow the label int to be unsigned, we must
        # * Convert it to some positive number
        if point_cloud_cluster.label == -1:
            point_cloud_cluster.label = 1000
        point_cloud_cluster.bounding_box.label = point_cloud_cluster.label

        # Appending a bounding box message to the Bounding Box Array
        bounding_box_array.boxes.append(point_cloud_cluster.bounding_box)

        bounding_box_publisher.publish(bounding_box_array)

def generate_cluster_bboxes_and_publish(filtered_pc: o3d.geometry.PointCloud,
                                        cluster_labels_for_each_data_point: list,
                                        remove_noise_cluster=True
                                        ):
    """
    Function to generate clusters/classes based on the HDBSCAN algorithm,
    get bounding boxes from clusters and publish them to ROS.
    """
    pcd_points = get_points_from_point_cloud(filtered_pc)
    unique_labels = get_number_of_labels(cluster_labels_for_each_data_point)

    # Removes label -1, which represents classified noise
    if remove_noise_cluster:
        if -1 in unique_labels:
            unique_labels.remove(-1)
    
    point_cloud_size = get_point_cloud_size(pcd_points)
    colored_clusters = generate_clusters(unique_labels)

    # Updates each class/cluster with given their respective data points
    assign_data_points_to_each_cluster(colored_clusters, pcd_points, point_cloud_size, unique_labels, cluster_labels_for_each_data_point)
    get_cluster_bounding_box_and_publish(colored_clusters)
    return colored_clusters

def clusterize_point_cloud(data):
    """
    Gets point cloud data from /scan topic, converts to numpy, converts to Open3D format,
    downsamples it, filters it by removing the ground plane, clusterizes it using HDBSCAN
    and then publishes bounding boxes from clusterized objects to RViz.
    """
    xyz = ros_msg_to_numpy(data)
    pc = numpy_to_o3d(xyz)
    downsampled_pc = downsample(pc)
    pc_points = np.asarray(downsampled_pc.points)
    filter_boundaries = get_pass_through_filter_boundaries(pc_points)
    filtered_pc = pass_through_filter(filter_boundaries, downsampled_pc)
    cluster_labels_for_each_data_point = cluster_hdbscan(filtered_pc)
    generate_cluster_bboxes_and_publish(filtered_pc, cluster_labels_for_each_data_point)

def convert_to_open3d_and_paint_clusters(colored_point_cloud_clusters) -> deque:
    """
    Function for visualizing clustered point clouds in Open3D.
    """
    colored_clusters_with_data_points = []
    for colored_point_cloud_cluster in colored_point_cloud_clusters:
        colored_pc = o3d.geometry.PointCloud()
        colored_pc.points = o3d.utility.Vector3dVector(colored_point_cloud_cluster.cluster_data_points)
        colored_pc.paint_uniform_color(colored_point_cloud_cluster.color_open3d)
        # Appends bounding boxes too
        colored_clusters_with_data_points.append(colored_pc.get_axis_aligned_bounding_box())
        colored_clusters_with_data_points.append(colored_pc)
    return colored_clusters_with_data_points

# ! May be deprecated, check later.
def publish_points_to_ros(o3d_pc: o3d.geometry.PointCloud):
    from time import sleep
    pc_points = np.asarray(o3d_pc.points)
    pcl_msg = numpy_to_ros_msg(pc_points)
    rospy.loginfo(pcl_msg)
    point_cloud_publisher.publish(pcl_msg)

if __name__ == '__main__':

    # * Online package usage
    if IS_TESTING_ONLINE:
        # Sets up subscriber for Point Cloud retrieval
        rospy.Subscriber("scan", PointCloud, clusterize_point_cloud)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    
    # * Offline testing
    else:
        point_cloud_publisher = rospy.Publisher('pc_publisher', PointCloud, queue_size=5)

        files = ['wall_table_chair.pcd', 'box_wall_test.pcd', 'multiple_objects.pcd']

        visualize_in_open3d = False

        def run(visualize_in_open3d, dbscan=True):
            """ Function to ease the execution when checking in Open3D or not"""
            import time
            pcd = o3d.io.read_point_cloud(files[2])
            raw_pc = np.asarray(pcd.points)
            visualize_pcd([pcd])
            print(f"Raw point cloud size: {raw_pc.shape[0]}")
            downsampled_pc = downsample(pcd)
            #visualize_pcd([downsampled_pc])
            pc_points = np.asarray(downsampled_pc.points)
            print(f"Downsampled point cloud {pc_points.shape[0]}")
            #visualize_pcd([downsampled_pc])
            
            filter_boundaries = get_pass_through_filter_boundaries(pc_points)
            filtered_pc = pass_through_filter(filter_boundaries, downsampled_pc)
            #visualize_pcd([filtered_pc])
            filtered_pc_points = np.asarray(filtered_pc.points)
            print(f"Filtered cloud size: {filtered_pc_points.shape[0]}")


            # Estimates dbscan epsilon and minpoints
            if dbscan:
                import matplotlib.pyplot as plt
                start_counter_ns = time.perf_counter_ns()
                from sklearn.neighbors import NearestNeighbors # importing the library
                neighb = NearestNeighbors(n_neighbors=2) # creating an object of the NearestNeighbors class
                nbrs = neighb.fit(pc_points) # fitting the data to the object
                distances, indices = nbrs.kneighbors(pc_points) # finding the nearest neighbours
                # Sort and plot the distances results
                distances = np.sort(distances, axis = 0) # sorting the distances
                distances = distances[:, 1] # taking the second column of the sorted distances
                end_counter_ns = time.perf_counter_ns()
                exec_time_in_ms = (end_counter_ns - start_counter_ns)/(10**6)
                print(f"Time took to execute KNN: {exec_time_in_ms}ms") 
                plt.rcParams['figure.figsize'] = (5,3) # setting the figure size
                plt.plot(distances) # plotting the distances
                plt.xlabel("Número de Pontos")
                plt.ylabel("Épsilon (m)")
                plt.show() # showing the plot

                dbscan_min_points = 2*pc_points.shape[1]

            # K-means
            #clf = k_means_clustering.KMeans(k=6)
            #pc = np.asarray(filtered_pc.points)
            #labels = clf.predict(pc)
            #k_means_clustering.draw_labels_on_model(filtered_pc,labels)
            start_counter_ns = time.perf_counter_ns()
            
            cluster_labels_for_each_data_point = cluster_hdbscan(downsampled_pc)
            #clustered = clustering_dbscan(filtered_pc, 0.09, dbscan_min_points)

            end_counter_ns = time.perf_counter_ns()
            exec_time_in_ms = (end_counter_ns - start_counter_ns)/(10**6)
            print(f"Time took to execute clusterization: {exec_time_in_ms}ms") 

            colored_paint_cloud_clusters = generate_cluster_bboxes_and_publish(filtered_pc, cluster_labels_for_each_data_point)
            clustered = convert_to_open3d_and_paint_clusters(colored_paint_cloud_clusters)

            if visualize_in_open3d:
                print(f"Downsampled PointCloud with {pc_points.shape[0]} points.")

            return clustered, pcd

        if visualize_in_open3d:
            clustered, raw_pcd = run(visualize_in_open3d)
            #draw_bounding_box_from_cluster()
            visualize_pcd([clustered])
        else:
            from time import sleep
            for i in range(0,5):
               _, _ = run(visualize_in_open3d)

    

