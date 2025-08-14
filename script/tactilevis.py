#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
from scipy.spatial import Delaunay
from scipy.interpolate import griddata
from geometry_msgs.msg import WrenchStamped


class TactileDisplay:
    def __init__(self):
        self.roscvbridge = CvBridge()
        rospy.Subscriber('tt_distactile', Float32MultiArray, self.tactile_callback)
        rospy.Subscriber('tt_photodigit', Float32MultiArray, self.photoimage_visualization)
        rospy.Subscriber('tt_contact', WrenchStamped, self.contact_callback)
        self.publish_period = rospy.Duration(1.0/rospy.get_param("~vis_pub_hz", default=30))
        rospy.Timer(self.publish_period, self.refresh_vis)

        self.pub_itp_surface = rospy.Publisher('tt_interpolate_points', Float32MultiArray, queue_size=3)
        self.pub_forcearray = rospy.Publisher('vis_force3d', MarkerArray, queue_size=10)
        self.pub_deformation = rospy.Publisher('vis_anchor', MarkerArray, queue_size=10)
        self.pub_surfmesh = rospy.Publisher('vis_surfmesh', Marker, queue_size=10)
        self.pub_contact = rospy.Publisher('vis_contact', Marker, queue_size=10)
        self.pub_photoimage = rospy.Publisher('vis_photoimage', Image, queue_size=10)
        self.visulize_mode = ''
        self.vis_backup_surfmesh = None
        self.vis_backup_points = None

    def tactile_callback(self, data):
        TACTID = data.layout.data_offset
        distactile = np.array(data.data).reshape(-1, 9)

        self.visulize_mode = rospy.get_param_cached("~tactile_display", default="static")
        self.tactile_itpoints = rospy.get_param_cached("~tactile_itpoints", default="false")
        ## visualization
        if self.visulize_mode == 'static':
            shift = [0, -0.03, 0]
            self.deformation_visualization(TACTID, distactile, shift, pscale = 10, dz_scale = 5)
            self.distforce_visualization(TACTID, distactile)
        elif self.visulize_mode == 'mobile':
            shift = [-0.012 + 0.0015, -0.024 + 0.0015, 0]
            self.deformation_visualization(TACTID, distactile, shift)
        else: raise ValueError("visulize_mode for tactile is wrongly set: {}".format(self.visulize_mode))

        if self.tactile_itpoints == 'true':
            self.publish_interpolate_points(TACTID, distactile, discreteness=64)

    def refresh_vis(self, e):
        if self.vis_backup_surfmesh is None: return
        time_gap = rospy.Time.now() - self.vis_backup_surfmesh.header.stamp
        if time_gap < self.publish_period: return
        self.vis_backup_surfmesh.header.stamp = rospy.Time.now()
        self.pub_surfmesh.publish(self.vis_backup_surfmesh)

        for i in range(len(self.vis_backup_points.markers)):
            self.vis_backup_points.markers[i].header.stamp = rospy.Time.now()
        self.pub_deformation.publish(self.vis_backup_points)

    @staticmethod
    def interpolate_positions(distactile, check_locations, shift=(0, 0, 0), scale=(1, 1, 1)):
        points = distactile[:, :3] + distactile[:, 3:6]
        points[:, 0] += shift[0]
        points[:, 1] += shift[1]
        points[:, 2] += shift[2]
        points[:, 0] *= scale[0]
        points[:, 1] *= scale[1]
        points[:, 2] *= scale[2]
        edge = 0
        if isinstance(check_locations, int):
            xi = np.linspace(points[:, 0].min() - edge, points[:, 0].max() + edge, check_locations+2)
            yi = np.linspace(points[:, 1].min() - edge, points[:, 1].max() + edge, check_locations+2)
            xi, yi = np.meshgrid(xi, yi)
            check_locations = (xi, yi)
        zs = griddata((points[:, 0], points[:, 1]), points[:, 2], 
                      check_locations, method='cubic', fill_value=0)
        check_points = np.vstack([check_locations[0].flatten(), check_locations[1].flatten(), zs.flatten()]).T
        return points, check_points
    
    def publish_interpolate_points(self, TACTID, distactile, discreteness):
        ## Interpolation
        ori_points, ip_points = self.interpolate_positions(distactile, discreteness)
        ip_msg = Float32MultiArray()
        ip_msg.layout.data_offset = TACTID
        ip_msg.data = ip_points.flatten().tolist()
        self.pub_itp_surface.publish(ip_msg)

    def distforce_visualization(self, TACTID, distactile, pscale = 10):
        f_max = np.linalg.norm(distactile[:, -3:], axis=-1).max()
        f_color = f_max + 0.005
        marker_array = MarkerArray()
        for id, pf in enumerate(distactile):
            marker_array.markers.append(self.wrap_contact_marker(TACTID+1, id, pf, pscale, f_color))
        self.pub_forcearray.publish(marker_array)
    
    def contact_callback(self, wren_vis):
        if self.visulize_mode == 'static': return 
        TACTID = int(wren_vis.header.frame_id[-1])
        point_data = [0, 0, 0, 0, 0, 0,
            wren_vis.wrench.force.x, wren_vis.wrench.force.y, - wren_vis.wrench.force.z]
        contact_arrow = self.wrap_contact_marker(TACTID, TACTID, point_data, pscale = 0.5, f_color = 2)
        self.pub_contact.publish(contact_arrow)

    @staticmethod
    def wrap_contact_marker(TACTID, id, point_data, pscale, f_color):
        marker = Marker()
        # marker.header.frame_id = 'map'
        marker.header.frame_id = "p{}".format(TACTID)
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pt_{}".format(TACTID)
        marker.id = id + TACTID * 64
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(.20)

        # for remove warning in rviz
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        start = Point()
        start.x = point_data[0] * pscale
        start.y = point_data[1] * pscale
        start.z = point_data[2] * pscale
        marker.points.append(start)
        end = Point()
        end.x = (point_data[0] + point_data[6]) * pscale
        end.y = (point_data[1] + point_data[7]) * pscale
        end.z = (point_data[2] + (point_data[8] if point_data[8] > 0 else 0.001)) * pscale
        marker.points.append(end)
        marker.scale.x = 0.02 * pscale # Shaft diameter
        marker.scale.y = 0.05 * pscale # Head diameter
        marker.scale.z = 0.05 * pscale # Head length
        f_current = np.linalg.norm(point_data[6:])
        marker.color.a = 0.3 + 0.7 * (f_current / f_color)
        marker.color.r = 1.0 * (f_current / f_color)
        marker.color.g = 0.5 - 0.5 * (f_current / f_color)
        marker.color.b = 0.0
        return marker

    def deformation_visualization(self, TACTID, distactile, shift, pscale=1, dz_scale=1):
        # Points on surface
        f_max = np.linalg.norm(distactile[:, -3:], axis=-1).max()
        f_color = f_max + 0.005

        marker_array = MarkerArray()
        for i, pf in enumerate(distactile):
            marker = Marker()
            # marker.header.frame_id = 'map'
            marker.header.frame_id = "p{}".format(TACTID+1)
            marker.header.stamp = rospy.Time.now()
            marker.ns = "p_{}_point".format(TACTID+1)
            marker.id = i + TACTID * 64
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(.20)
            marker.pose.position.x = (pf[0] + pf[3] + shift[0]) * pscale
            marker.pose.position.y = (pf[1] + pf[4] + shift[1]) * pscale
            marker.pose.position.z = (pf[2] + pf[5] + shift[2]) * pscale * dz_scale
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.001 * pscale
            marker.scale.y = 0.001 * pscale
            marker.scale.z = 0.001 * pscale
            f_current = np.linalg.norm(pf[6:])
            marker.color.a = 0.8 + 0.2 * (f_current / f_color)
            marker.color.r = 1.0 * (f_current / f_color)
            marker.color.g = 0.0
            marker.color.b = 1.0 - 1.0 * (f_current / f_color)
            marker_array.markers.append(marker)
        self.pub_deformation.publish(marker_array)
        self.vis_backup_points = marker_array

        # Interpolated surface 
        mesh_list = Marker()
        mesh_list.header.frame_id = "p{}".format(TACTID+1)
        mesh_list.header.stamp = rospy.Time.now()
        mesh_list.type = Marker.TRIANGLE_LIST
        mesh_list.action = Marker.ADD
        mesh_list.ns = "p_{}_surface".format(TACTID+1)
        mesh_list.pose.position.x = 0
        mesh_list.pose.position.y = 0
        mesh_list.pose.position.z = 0
        mesh_list.pose.orientation.x = 0.0
        mesh_list.pose.orientation.y = 0.0
        mesh_list.pose.orientation.z = 0.0
        mesh_list.pose.orientation.w = 1.0
        mesh_list.scale.x = 1.0
        mesh_list.scale.y = 1.0
        mesh_list.scale.z = 1.0
        mesh_list.color.a = 1.0
        mesh_list.color.r = 0.8
        mesh_list.color.g = 0.6
        mesh_list.color.b = 0.6

        ## Interpolation
        _, points = self.interpolate_positions(distactile, 50, shift, scale = (pscale, pscale, pscale * dz_scale))
        
        ## Vertices to the marker
        tri = Delaunay(points[:, :2])
        vertices = points[tri.simplices]
        for triangle in vertices:
            for vertex in triangle:
                p = Point()
                p.x, p.y, p.z = vertex
                mesh_list.points.append(p)
        self.pub_surfmesh.publish(mesh_list)
        self.vis_backup_surfmesh = mesh_list

    def photoimage_visualization(self, data):
        TACTID = data.layout.data_offset
        data_array = np.array(data.data)
        dim_info = data.layout.dim
        dims = []
        for dim in dim_info: dims.append(dim.size)
        photodigits = data_array[:(len(data_array)//2)].reshape(tuple(dims))
        pt_cst = data_array[(len(data_array)//2):].reshape(tuple(dims))

        GRIDSIZE = len(photodigits)
        resize_unit = 30
        if photodigits.shape[-1] == 9: 
            PADPHOTOX = np.array([-1, -1, -1,  0,  0,  0,  1,  1,  1])
            PADPHOTOY = np.array([-1,  0,  1, -1,  0,  1, -1,  0,  1])
        elif photodigits.shape[-1] == 5: 
            PADPHOTOX, PADPHOTOY = np.array([-1,  0,  0,  0,  1]), np.array([ 0, -1,  0,  1,  0])
        elif photodigits.shape[-1] in [1, 8]:
            PADPHOTOX, PADPHOTOY = np.array([0]), np.array([0])
        offx = PADPHOTOX - PADPHOTOX.min()
        offy = PADPHOTOY - PADPHOTOY.min()

        photodigits_clip = np.clip(photodigits - pt_cst, 0, 1)

        list_row_cl = []
        list_row = []
        for row_i in range(GRIDSIZE):
            list_col = []
            list_col_cl = []
            for col_j in range(GRIDSIZE):
                if len(offx) == 9: 
                    list_col.append(np.reshape(photodigits[row_i, col_j], (3,3)))
                    list_col_cl.append(np.reshape(photodigits_clip[row_i, col_j], (3,3)))
                elif len(offx) == 5:
                    padsense = np.zeros((3, 3))
                    padsense[offx, offy] = photodigits[row_i, col_j]
                    list_col.append(padsense)
                    padsense_cl = np.zeros((3, 3))
                    padsense_cl[offx, offy] = photodigits_clip[row_i, col_j]
                    list_col_cl.append(padsense_cl)
                elif len(offx) == 1:
                    list_col.append(photodigits[row_i, col_j])
                    list_col_cl.append(photodigits_clip[row_i, col_j])
            list_row.append(np.hstack(list_col))
            list_row_cl.append(np.hstack(list_col_cl))
        digit_orig = np.vstack(list_row)
        photocon_clip = np.vstack(list_row_cl)

        resize = resize_unit * digit_orig.shape[0]
        digit_orig_rs = cv2.resize(digit_orig, (resize, resize), interpolation = cv2.INTER_NEAREST)
        for pt_line in np.linspace(0, resize, GRIDSIZE + 1):
            pt_line = int(pt_line)
            cv2.line(digit_orig_rs, (pt_line, 0), (pt_line, resize), (0.8))
            cv2.line(digit_orig_rs, (0, pt_line), (resize, pt_line), (0.8))
        for ii in range(digit_orig.shape[0]):
            for jj in range(digit_orig.shape[1]):
                cv2.putText(digit_orig_rs, '{:.3f}'.format(digit_orig[ii, jj]), (int((jj + 0.15)*resize_unit), int((ii + 0.36)*resize_unit)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.0075*resize_unit, (1 if digit_orig[ii, jj] < 0.7 else 0), 1, cv2.LINE_AA)
        resize = resize_unit * photocon_clip.shape[0]
        photocon_clip_rs = cv2.resize(photocon_clip, (resize, resize), interpolation = cv2.INTER_NEAREST)
        self.pattern_threshold = 0.01
        photocon_nml = photocon_clip_rs / photocon_clip_rs.max() if photocon_clip_rs.max() > self.pattern_threshold else photocon_clip_rs
        for pt_line in np.linspace(0, resize, GRIDSIZE + 1):
            pt_line = int(pt_line)
            cv2.line(photocon_clip_rs, (pt_line, 0), (pt_line, resize), (1))
            cv2.line(photocon_clip_rs, (0, pt_line), (resize, pt_line), (1))
            cv2.line(photocon_nml, (pt_line, 0), (pt_line, resize), (0.8))
            cv2.line(photocon_nml, (0, pt_line), (resize, pt_line), (0.8))
        for ii in range(photocon_clip.shape[0]):
            for jj in range(photocon_clip.shape[1]):
                cv2.putText(photocon_clip_rs, '{:.3f}'.format(photocon_clip[ii, jj]), (int((jj + 0.15)*resize_unit), int((ii + 0.6)*resize_unit)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.0075*resize_unit, (1 if photocon_clip[ii, jj] < 0.7 else 0), 1, cv2.LINE_AA)
        
        photoimg = np.concatenate([digit_orig_rs, photocon_clip_rs, photocon_nml], axis=1)
        # photoimg = photocon
        photoimg = cv2.normalize(photoimg*255, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        ros_image = self.roscvbridge.cv2_to_imgmsg(photoimg, encoding="passthrough")
        self.pub_photoimage.publish(ros_image)


if __name__ == '__main__':
    rospy.init_node('tactile_visualization_node', anonymous=True)
    app = TactileDisplay()
    rospy.spin()