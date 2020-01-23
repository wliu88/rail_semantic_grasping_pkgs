#!/usr/bin/env python

import rospy
from rail_semantic_grasping.srv import SegmentSemanticObjects, GetSemanticGrasps

class ModularDataCollection:
    """
    This class is used to segment object and sample grasps for semantic grasping through python.
    """

    SEMANTIC_OBJECTS_TOPIC = "/semantic_grasp_suggestion/semantic_objects"
    SEMANTIC_OBJECTS_WITH_GRASPS_TOPIC = "/semantic_grasp_suggestion/semantic_objects_with_grasps"

    SEGMENT_SEMANTIC_OBJECTS_SERVICE = "/object_semantic_segmentation/segment_objects"
    GET_SEMANTIC_GRASPS_SERVICE = "/semantic_grasp_suggestion/get_grasps"

    def __init__(self):
        
        # Set up services
        self.segment_semantic_objects_srv = rospy.ServiceProxy(ModularDataCollection.SEGMENT_SEMANTIC_OBJECTS_SERVICE, SegmentSemanticObjects)
        self.get_semantic_grasps_srv = rospy.ServiceProxy(ModularDataCollection.GET_SEMANTIC_GRASPS_SERVICE, GetSemanticGrasps)
        
        print("Connecting to semantic segmentation and grasp sampling services ...")
        self.segment_semantic_objects_srv.wait_for_service()
        self.get_semantic_grasps_srv.wait_for_service()
        print("... semantic segmentation and grasp sampling services connected")
    
    def get_semantic_objects(self):
        """
        This function returns the segmented object in the form of rail_semantic_grasping.msg.SemanticObjectList
        """

        try:
            resp = self.segment_semantic_objects_srv()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp.semantic_objects

    def get_semantic_grasps(self, semantic_objects):
        """
        This function returns the previously segmented object and in the form of rail_semantic_grasping.msg.SemanticObjectList
        """
        try:
            resp = self.get_semantic_grasps_srv(semantic_objects)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp.semantic_objects_with_grasps

if __name__ == "__main__":
    MDC = ModularDataCollection()
    semantic_objects = MDC.get_semantic_objects()
    semantic_objects_with_grasps = MDC.get_semantic_grasps(semantic_objects)
    print(semantic_objects_with_grasps)
