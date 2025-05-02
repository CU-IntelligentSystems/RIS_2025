#!/usr/bin/env python3
import rospy
import numpy as np
import cv2, yaml, os

from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import OccupancyGrid

class MapSaverNode:
    def __init__(self):
        rospy.init_node('map_saver_node')
        
        #read Duckie name from the ENV 
        vehicle = os.environ.get('VEHICLE_NAME', 'duckie')

        #users can override via ROS params, but default into /data/<veh>/maps
        default_dir  = f"/data/{vehicle}/maps"
        default_name = f"{vehicle}_map"

        #parameters
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.output_dir = rospy.get_param('~output_dir', 'default_dir')
        self.map_name   = rospy.get_param('~map_name', 'default_name')

        #store latest map
        self.latest_map = None

        #subs & service
        rospy.Subscriber(self.map_topic, OccupancyGrid,
                         self.map_callback, queue_size=1)
        self.srv = rospy.Service('~save_map', Empty,
                                 self.handle_save_map)
        rospy.loginfo(f"[map_saver] Listening on {self.map_topic}")
        rospy.on_shutdown(self._on_shutdown_save)
        rospy.spin()

    def _on_shutdown_save(self):
        if self.latest_map is not None:
            rospy.loginfo("[map_saver] Auto-saving map on shutdown…")
            self._dump_to_disk(self.latest_map)

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg
        self._dump_to_disk(msg)

    def handle_save_map(self, req):
        if self.latest_map is None:
            rospy.logwarn("[map_saver] No map received yet!")
            return EmptyResponse()
        self._dump_to_disk(self.latest_map)
        return EmptyResponse()

    def _dump_to_disk(self, grid: OccupancyGrid):
        #ensure the folder exists
        os.makedirs(self.output_dir, exist_ok=True)

        #converting data to 2D array
        w, h = grid.info.width, grid.info.height
        arr = np.array(grid.data, dtype=np.int8).reshape((h, w))

        #PGM image: free=254, occ=0, unk=205
        img = np.zeros((h, w), dtype=np.uint8)
        img[arr == 0] = 254
        img[arr == 100] =   0
        img[arr == -1] = 205

        #timestamp to avoid overwriting previous data
        ts = int(rospy.Time.now().to_sec())
        base = f"{self.map_name}_{ts}"
        pgm_fp = os.path.join(self.output_dir, base + '.pgm')
        yaml_fp = os.path.join(self.output_dir, base + '.yaml')

        #attempt write files
        try:
            cv2.imwrite(pgm_fp, img)
            meta = {
                'image': os.path.basename(pgm_fp),
                'resolution': grid.info.resolution,
                'origin': [
                    grid.info.origin.position.x,
                    grid.info.origin.position.y,
                    0.0
                ],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.20
            }
            with open(yaml_fp, 'w') as f:
                yaml.dump(meta, f, default_flow_style=False)
            rospy.loginfo(f"[map_saver] Map saved to {pgm_fp} + {yaml_fp}")
        except Exception as e:
            rospy.logerr(f"[map_saver] Failed to save map: {e}")

if __name__ == '__main__':
    try:
        MapSaverNode()
    except rospy.ROSInterruptException:
        pass
