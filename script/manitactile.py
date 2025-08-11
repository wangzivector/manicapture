#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt16MultiArray, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from scipy.spatial import Delaunay
from scipy.interpolate import griddata

np.set_printoptions(precision=4, suppress=True)


class ManiTactile:
    def __init__(self):
        self.last_time = [time.time() for i in range(4)]
        self.pt_cst = [[], [], [], []]
        self.pt_opgs = [None, None, None, None]
        self.last_opgs = [None, None, None, None]
        self.pt_sense = [None, None, None, None]
        self.distactile = [None, None, None, None]
        self.displacement = [None, None, None, None]
        self.roscvbridge = CvBridge()

        rospy.Subscriber('hardware_photos', UInt16MultiArray, self.photo_callback)

        self.pub_itp_surface = rospy.Publisher('tt_itp_surface', Float32MultiArray, queue_size=3)
        self.pub_displacement = rospy.Publisher('tt_displacement', Float32MultiArray, queue_size=3)

        self.pub_forcearray = rospy.Publisher('vis_forcearray', MarkerArray, queue_size=10)
        self.pub_deformation = rospy.Publisher('vis_deformation', MarkerArray, queue_size=10)
        self.pub_surfmesh = rospy.Publisher('vis_surfmesh', Marker, queue_size=10)
        self.pub_photoimage = rospy.Publisher('vis_photoimage', Image, queue_size=10)

        self.initial_batchsize = 10
        self.initial_data_var = 2e-4
        self.pattern_threshold = 0.01
        self.method_set = {"gaussian": self.gaussiangd, 'intensity': self.intensitygd}
        self.opt_mehod = ''
        self.visulize_mode = ''
        
        self.check_param(None)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_param)

    def check_param(self, event):
        if self.opt_mehod != rospy.get_param("/manitactile_node/photo_optimization", default="gaussian"):
            self.opt_mehod = rospy.get_param("/manitactile_node/photo_optimization", default="gaussian")
            self.optifun = self.method_set[self.opt_mehod]
            rospy.logwarn("optimization method of photo changed to {}".format(self.opt_mehod))

        if self.visulize_mode != rospy.get_param("/manitactile_node/tactile_display", default="static"):
            self.visulize_mode = rospy.get_param("/manitactile_node/tactile_display", default="static")
            rospy.logwarn("tactile_display changed to {}".format(self.visulize_mode))

    def photo_callback(self, data):
        ## data extraction
        HEADERSIZE = data.data[0]
        TACTSIZE, TACTID, GRIDLENGTH, PADSIZE, ADCRES, SENSEDELAY, SPACING, UDRANGE, SEQUENCE = data.data[1:HEADERSIZE]
        ptdata = np.array(data.data[HEADERSIZE:])
        self.GRIDPHOTO = ptdata[:GRIDLENGTH]
        self.PADPHOTOX = ptdata[GRIDLENGTH:(GRIDLENGTH+PADSIZE)] - 4 # (1, 7) to (-3, - 3)
        self.PADPHOTOY = ptdata[(GRIDLENGTH+PADSIZE):(GRIDLENGTH+PADSIZE*2)] - 4 # (1, 7) to (-3, - 3)
        # self.PADEDGE = np.max(np.max(np.abs([self.PADPHOTOX, self.PADPHOTOY])))
        ptencode = ptdata[(GRIDLENGTH+PADSIZE*2):(GRIDLENGTH+PADSIZE*2+GRIDLENGTH**2)] # GRIDLENGTH * GRIDLENGTH
        ptread = ptdata[(GRIDLENGTH+PADSIZE*2+GRIDLENGTH**2):].astype(np.float) # (GRIDLENGTH, GRIDLENGTH, PADSIZE)
        self.updatept(TACTID, ptencode, ptread, GRIDLENGTH, PADSIZE)

        ## preprocessing
        photodigits_c = self.pt_sense[TACTID] / (np.power(2, ADCRES) - 1) # (GRIDLENGTH, GRIDLENGTH, PADSIZE)
        self.initializephoto(TACTID, photodigits_c, GRIDLENGTH, refresh = (SEQUENCE == 0))
        if not isinstance(self.pt_cst[TACTID], np.ndarray): return False

        ## tactile x-y-h optimization
        start_opt_time = time.time()
        POINTSIZE, PADS = GRIDLENGTH*GRIDLENGTH, [self.PADPHOTOX, self.PADPHOTOY]
        iterations, retSSE, optgs = self.optifun(photodigits_c, self.last_opgs[TACTID], POINTSIZE, PADS)
        self.last_opgs[TACTID] = optgs.copy()
        time_opti = time.time() - start_opt_time

        ## decompose active force
        infostat, self.distactile[TACTID], self.displacement[TACTID] = self.compute_force(self.pt_opgs[TACTID], optgs, SPACING, self.GRIDPHOTO)
        
        ## (interpolated) deformation advertisement
        ip_points = self.advertise_deformation(TACTID, self.distactile[TACTID], self.displacement[TACTID], discreteness=64)
        
        ## visualization
        if self.visulize_mode == 'static':
            if TACTID == rospy.get_param('/machine_server/showrosimage', -1):
                self.photoimage_visualization(TACTID, photodigits_c)
            shift = {'sx': 0, 'sy': -0.03, 'sz': 0}
            self.deformation_visualization(TACTID, self.distactile[TACTID], infostat, shift, pscale = 10, dz_scale = 5)
            self.distforce_visualization(TACTID, self.distactile[TACTID], infostat)
        elif self.visulize_mode == 'mobile':
            shift = {'sx': -0.012 + 0.0015, 'sy': -0.024 + 0.0015, 'sz': 0}
            self.deformation_visualization(TACTID, self.distactile[TACTID], infostat, shift, frame='p')

        else: raise ValueError("visulize_mode for tactile is wrongly set: {}".format(self.visulize_mode))

        ## logging
        timeforud = time.time() - self.last_time[TACTID]
        self.last_time[TACTID] = time.time()
        deepest_point = ip_points[np.argmin(ip_points[:, 2])] * 1000
        rospy.loginfo("ID:{} iter:{} sse:{:.03f} var:{} time:{:.03f}ms".format(
            TACTID, iterations, retSSE.mean(), optgs.shape, (time_opti)*1000))
        rospy.loginfo("ID:{}/{} {:.01f}Hz/{:.01f}ms DELAY:{}us; RES:{} GRID:{}; PAD:{}; pt:{} dp: {}".format(
            TACTID, TACTSIZE, 1/timeforud, timeforud*1000, SENSEDELAY, ADCRES, 
            self.GRIDPHOTO.shape, self.PADPHOTOX.shape, photodigits_c.shape, deepest_point))

    @staticmethod
    def intensitygd(i_data, init_v, POINTSIZE, PADS, MAX_ITERATION=200, TThreshold=1e-4):
        ## reshape to (64 x 9)
        PADSIZE = len(PADS[0])
        dXX = np.tile(np.array(PADS[0]).astype(float), (POINTSIZE, 1)) # (1, 9) -> (64, 9)
        dYY = np.tile(np.array(PADS[1]).astype(float), (POINTSIZE, 1)) # (1, 9) -> (64, 9)
        if i_data.shape != (POINTSIZE, PADSIZE): 
            i_data = np.reshape(i_data, (POINTSIZE, PADSIZE))

        ## Initialization for [A, x0, y0, sigma_x, sigma_y]
        # init_v = None # not appropriate for gaussian fun.
        if init_v is not None:
            he = init_v[:, 0, np.newaxis]
            dex = init_v[:, 1, np.newaxis]
            dey = init_v[:, 2, np.newaxis]
            i0 = init_v[:, 3, np.newaxis]
        else:
            # i0 = i_data.max(axis=1, keepdims=True) # (64, 1)
            i0 = np.ones((i_data.shape[0], 1)) * 0.50 # (64, 1)
            dex = (dXX * i_data).sum(axis=1, keepdims=True) / i_data.sum(axis=1, keepdims=True) # emprical
            dey = (dYY * i_data).sum(axis=1, keepdims=True) / i_data.sum(axis=1, keepdims=True) # emprical
            he = np.ones((i_data.shape[0], 1)) * 1.0 # (64, 1)
        # print("Intialize", np.hstack([he, dex, dey, i0]))

        ## Parameter Optimization
        last_SSE = None
        iterations = 0
        for iter in range(MAX_ITERATION):
            bias_xx = dXX - dex # (64, 9)
            bias_yy = dYY - dey # (64, 9)
            de_sqt = bias_xx**2 + bias_yy**2
            denominator = (he**2 + de_sqt)**1.5 # (64, 9)
            predict_i = (i0 * he) / denominator # (64, 9)
            error = i_data - predict_i # (64, 9)
            o_SSE = (error * error).sum(axis = 1, keepdims=True) # (64, 9) -> (64, 1)

            ## Gradients for variables
            # grad_i0 = 2 * error * (he / denominator)
            gd_denominator = (he**2) + (de_sqt**2.5)
            grad_dex = 2 * error * (3 * i0 * he * bias_xx) / gd_denominator
            grad_dey = 2 * error * (3 * i0 * he * bias_yy) / gd_denominator
            grad_he = 2 * error * (i0 * ((he**2 + de_sqt)**1.5) - (3 * i0 * (he**2) * ((he**2 + de_sqt)**0.5))) / (((he**2) + de_sqt)**3)

            # learning_rate = o_SSE * 0.1 + ((o_SSE < 0.2) * 0.05) # (64, 1)
            learning_rate = 0.1 # (64, 1)
            # i0 += learning_rate * grad_i0.mean(axis=1, keepdims=True) # (64, 9) -> (64,1)
            dex += learning_rate * grad_dex.sum(axis=1, keepdims=True) # (64, 9) -> (64,1)
            dey += learning_rate * grad_dey.sum(axis=1, keepdims=True) # (64, 9) -> (64,1)
            he += learning_rate * grad_he.sum(axis=1, keepdims=True) # (64, 9) -> (64,1)
            iterations += 1
            if last_SSE is not None:
                if np.all(np.abs(last_SSE - o_SSE) < TThreshold): break
            last_SSE = o_SSE
            # print(iterations, o_SSE, i_data, np.hstack([he, dex, dey, i0]))
            # print(iterations, o_SSE.mean(), np.hstack([he, dex, dey, i0])[20:30])
        # exit(0)
        # print(iterations, o_SSE.mean(), np.hstack([he, dex, dey, i0]))
        return iterations, o_SSE, np.hstack([-he, dex, dey, i0])

    @staticmethod
    def gaussiangd(datamat, init_v, POINTSIZE, PADS, MAX_ITERATION=200, TThreshold=1e-4):
        ## reshape to (64 x 9)
        PADSIZE = len(PADS[0])
        XX = np.tile(np.array(PADS[0]).astype(float), (POINTSIZE, 1)) # (1, 9) -> (64, 9)
        YY = np.tile(np.array(PADS[1]).astype(float), (POINTSIZE, 1)) # (1, 9) -> (64, 9)
        if datamat.shape != (POINTSIZE, PADSIZE): 
            datamat = np.reshape(datamat, (POINTSIZE, PADSIZE))
        
        ## Initialization for [A, x0, y0, sigma_x, sigma_y]
        init_v = None # not appropriate for gaussian fun.
        if init_v is not None:
            vA = init_v[:, 0, np.newaxis]
            vx0 = init_v[:, 1, np.newaxis]
            vy0 = init_v[:, 2, np.newaxis]
            vsigma_x = init_v[:, 3, np.newaxis]
            vsigma_y = init_v[:, 4, np.newaxis]
        else:
            vA = datamat.max(axis=1, keepdims=True) # (64, 1)
            vx0 = (XX * datamat).sum(axis=1, keepdims=True) / datamat.sum(axis=1, keepdims=True) # emprical
            vy0 = (YY * datamat).sum(axis=1, keepdims=True) / datamat.sum(axis=1, keepdims=True) # emprical
            vsigma_x = np.ones((datamat.shape[0], 1)) * 0.1 * PADSIZE # (64, 1)
            vsigma_y = np.ones((datamat.shape[0], 1)) * 0.1 * PADSIZE # (64, 1)
        # print("Intialize", np.hstack([vA, vx0, vy0, vsigma_x, vsigma_y]))

        ## Parameter Optimization
        last_SSE = None
        iterations = 0
        for iter in range(MAX_ITERATION):
            bias_xx = XX - vx0 # (64, 9)
            bias_yy = YY - vy0 # (64, 9)
            sq_vsigma_x = vsigma_x * vsigma_x # (64, 1)
            sq_vsigma_y = vsigma_y * vsigma_y # (64, 1)
            predict_Z = vA * np.exp(
                -(bias_xx * bias_xx) / (2 * sq_vsigma_x) 
                -(bias_yy * bias_yy) / (2 * sq_vsigma_y)
                ) # (64, 9)
            error = datamat - predict_Z # (64, 9)
            o_SSE = (error * error).sum(axis = 1, keepdims=True) # (64, 9) -> (64, 1)
            
            ## Gradients for variables
            dA = (2 * error * predict_Z / vA) # (64, 9)
            dx0 = (2 * error * bias_xx / sq_vsigma_x) # (64, 9)
            dy0 = (2 * error * bias_yy / sq_vsigma_y) # (64, 9)
            dsigma_x = (2 * error * (bias_xx * bias_xx) / (sq_vsigma_x * vsigma_x)) # (64, 9)
            dsigma_y = 2 * error * (bias_yy * bias_yy) / (sq_vsigma_y * vsigma_y) # (64, 9)
            
            # learning_rate = o_SSE * 0.1 + ((o_SSE < 0.2) * 0.05) # (64, 1)
            learning_rate = 0.03 # (64, 1)

            # vA += learning_rate * dA.sum(axis=1, keepdims=True) # (64, 9) -> (64,1)
            vx0 += learning_rate * dx0.sum(axis=1, keepdims=True) # (64, 9) -> (64,1)
            vy0 += learning_rate * dy0.sum(axis=1, keepdims=True) # (64, 9) -> (64,1)
            vsigma_x += learning_rate * dsigma_x.sum(axis=1, keepdims=True) # (64, 9) -> (64,1)
            vsigma_y += learning_rate * dsigma_y.sum(axis=1, keepdims=True) # (64, 9) -> (64,1)
            iterations += 1
            if last_SSE is not None:
                if np.all(np.abs(last_SSE - o_SSE) < TThreshold): break
            last_SSE = o_SSE
        # print(iterations, o_SSE, np.hstack([vA, vx0, vy0, vsigma_x, vsigma_y]))
        return iterations, o_SSE, np.hstack([vA, vx0, vy0, vsigma_x, vsigma_y])

    def initializephoto(self, TACTID, photodigits, GRIDLENGTH, refresh):
        if isinstance(self.pt_cst[TACTID], np.ndarray):
            if refresh == True: self.pt_cst[TACTID] = [photodigits]; return False
            if photodigits.shape == self.pt_cst[TACTID].shape:
                return True
            else: self.pt_cst[TACTID] = [photodigits]; return False
        else:
            if len(self.pt_cst[TACTID]) == 0: 
                self.pt_cst[TACTID].append(photodigits); return False
            if photodigits.shape == self.pt_cst[TACTID][-1].shape:
                self.pt_cst[TACTID].append(photodigits)
                if len(self.pt_cst[TACTID]) == self.initial_batchsize:
                    var_buff = np.var(self.pt_cst[TACTID], axis=0).max()
                    if var_buff > self.initial_data_var:
                        rospy.loginfo("[initialize]: data buff high variance: {}, retry ...".format(var_buff)) 
                        self.pt_cst[TACTID] = [photodigits]
                        return False
                    self.pt_cst[TACTID] = np.mean(self.pt_cst[TACTID], axis=0)
                    POINTSIZE, PADS = GRIDLENGTH*GRIDLENGTH, [self.PADPHOTOX, self.PADPHOTOY]
                    self.last_opgs[TACTID] = None
                    _, _, opted = self.optifun(self.pt_cst[TACTID], self.last_opgs[TACTID], POINTSIZE, PADS)
                    self.pt_opgs[TACTID] = opted.copy()
                    return True
                else: return False
            else: self.pt_cst[TACTID] = [photodigits]; return False

    def updatept(self, TACTID, ptencode, ptread, GRIDLENGTH, PADSIZE):
        ptencode_matrix = np.array([[bool(int(digit)) for digit in format(num, '0{}b'.format(PADSIZE))] for num in ptencode])
        ptencode_matrix = np.reshape(np.flip(ptencode_matrix, axis=-1), (GRIDLENGTH, GRIDLENGTH, -1))

        # self.pt_sense (GRIDLENGTH, GRIDLENGTH, PADSIZE)
        if (self.pt_sense[TACTID] is None) or (self.pt_sense[TACTID].shape != ptencode_matrix.shape): 
            self.pt_sense[TACTID] = np.zeros((GRIDLENGTH, GRIDLENGTH, PADSIZE))

        if np.any(ptencode_matrix):
            self.pt_sense[TACTID][ptencode_matrix] = ptread

    @staticmethod
    def compute_force(dftgs, optgs, SPACING, GRID, HEIGHT = 0.0):
        cell_interval = SPACING * 0.0001
        force_scale_xy = 10
        force_scale_z = 0.2
        DX = - (optgs[:, 1] - dftgs[:, 1]) * cell_interval
        DY = - (optgs[:, 2] - dftgs[:, 2]) * cell_interval
        DZ = - (optgs[:, 0] - dftgs[:, 0]) * cell_interval
        FX = DX * force_scale_xy
        FY = DY * force_scale_xy
        FZ = (optgs[:, 0] - dftgs[:, 0]) * force_scale_z

        infostat = {'fmax':0.0, }
        distactile, countpf = [], 0
        displacement = np.zeros((len(GRID), len(GRID), 6))
        for gd_u in GRID:
            for gd_v in GRID:
                photof = dict()
                photof['ox'] = gd_u * cell_interval # m
                photof['oy'] = gd_v * cell_interval # m
                photof['oz'] = HEIGHT # m

                photof['dx'] = DX[countpf] # m
                photof['dy'] = DY[countpf] # m
                photof['dz'] = DZ[countpf] # m
                if (gd_u in [GRID[0], GRID[-1]]) or (gd_v in [GRID[0], GRID[-1]]): 
                    photof['dz'] = DZ[countpf] / 2 # temp solution
                displacement[gd_u, gd_v] = [photof['ox'], photof['oy'], photof['oz'], 
                                        photof['dx'], photof['dy'], photof['dz']]
                
                photof['fx'] = FX[countpf] # m
                photof['fy'] = FY[countpf] # m
                photof['fz'] = FZ[countpf] # m

                f_current = np.sqrt(photof['fx']**2 + photof['fy']**2 + photof['fz']**2)
                photof['fmax'] = f_current
                distactile.append(photof)
                countpf += 1
                if f_current > infostat['fmax']: infostat['fmax'] = f_current
        infostat['fmax_color'] = infostat['fmax'] + 0.005
        return infostat, distactile, displacement # lisf of dict {ox, oy, oz,   dx, dy, dz,   fx, fy, fz}


    @staticmethod
    def interpolate_positions(photof, check_locations, shift={'sx':0, 'sy':0, 'sz':0}, scale=(1, 1, 1)):
        ## Interpolation
        points = []
        for pf in photof:
            points.append([(pf['ox'] + pf['dx'] + shift['sx']) * scale[0], 
                           (pf['oy'] + pf['dy'] + shift['sy']) * scale[1], 
                           (pf['oz'] + pf['dz'] + shift['sz']) * scale[2]])
        points = np.array(points)
        # edge = (points[:, 0].max() - points[:, 0].min()) / check_locations * 8
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

    def advertise_deformation(self, TACTID, photoinfo, d_pointinfo, discreteness):
        ## Interpolation
        ori_points, ip_points = self.interpolate_positions(photoinfo, discreteness)
        ip_msg = Float32MultiArray()
        ip_msg.layout.data_offset = TACTID
        ip_msg.data = ip_points.flatten().tolist()
        self.pub_itp_surface.publish(ip_msg)

        dp_msg = Float32MultiArray()
        dp_msg.layout.data_offset = TACTID
        dp_msg.data = d_pointinfo.flatten().tolist()
        self.pub_displacement.publish(dp_msg)
        return ip_points

    def distforce_visualization(self, TACTID, photoinfo, photostat, pscale = 10, frame='t'):
        marker_array = MarkerArray()
        for i, pf in enumerate(photoinfo):
            marker = Marker()
            # marker.header.frame_id = 'map'
            marker.header.frame_id = frame + "{}".format(TACTID+1)
            marker.header.stamp = rospy.Time.now()
            marker.ns = "pt_{}".format(TACTID+1)
            marker.id = i + TACTID * 64
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(.20)

            # leave for remove warning in rviz
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            start = Point()
            start.x = pf['ox'] * pscale
            start.y = pf['oy'] * pscale
            start.z = pf['oz'] * pscale
            marker.points.append(start)
            end = Point()
            end.x = (pf['ox'] + pf['fx']) * pscale
            end.y = (pf['oy'] + pf['fy']) * pscale
            end.z = (pf['oz'] + (pf['fz'] if pf['fz'] > 0 else 0.001)) * pscale
            marker.points.append(end)
            marker.scale.x = 0.001 * pscale # Shaft diameter
            marker.scale.y = 0.002 * pscale # Head diameter
            marker.scale.z = 0.002 * pscale # Head length
            marker.color.a = 0.3 + 0.7 * (pf['fmax'] / photostat['fmax_color'])
            marker.color.r = 1.0 * (pf['fmax'] / photostat['fmax_color'])
            marker.color.g = 0.5 - 0.5 * (pf['fmax'] / photostat['fmax_color'])
            marker.color.b = 0.0

            marker_array.markers.append(marker)
        self.pub_forcearray.publish(marker_array)

    def deformation_visualization(self, TACTID, photoinfo, photostat, shift, pscale=1, dz_scale=1, frame='t'):
        # Points on surface
        marker_array = MarkerArray()
        for i, pf in enumerate(photoinfo):
            marker = Marker()
            # marker.header.frame_id = 'map'
            marker.header.frame_id = frame + "{}".format(TACTID+1)
            marker.header.stamp = rospy.Time.now()
            marker.ns = frame + "_{}_point".format(TACTID+1)
            marker.id = i + TACTID * 64
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(.20)
            marker.pose.position.x = (pf['ox'] + pf['dx'] + shift['sx']) * pscale
            marker.pose.position.y = (pf['oy'] + pf['dy'] + shift['sy']) * pscale
            marker.pose.position.z = (pf['oz'] + pf['dz'] + shift['sz']) * pscale * dz_scale
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.001 * pscale
            marker.scale.y = 0.001 * pscale
            marker.scale.z = 0.001 * pscale
            marker.color.a = 0.8 + 0.2 * (pf['fmax'] / photostat['fmax_color'])
            marker.color.r = 1.0 * (pf['fmax'] / photostat['fmax_color'])
            marker.color.g = 0.0
            marker.color.b = 1.0 - 1.0 * (pf['fmax'] / photostat['fmax_color'])
            marker_array.markers.append(marker)
        self.pub_deformation.publish(marker_array)

        # Interpolated surface 
        mesh_list = Marker()
        mesh_list.header.frame_id = frame + "{}".format(TACTID+1)
        mesh_list.header.stamp = rospy.Time.now()
        mesh_list.type = Marker.TRIANGLE_LIST
        mesh_list.action = Marker.ADD
        mesh_list.ns = frame + "_{}_surface".format(TACTID+1)
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
        _, points = self.interpolate_positions(photoinfo, 50, shift, scale = (pscale, pscale, pscale * dz_scale))
        
        ## Vertices to the marker
        tri = Delaunay(points[:, :2])
        vertices = points[tri.simplices]
        for triangle in vertices:
            for vertex in triangle:
                p = Point()
                p.x, p.y, p.z = vertex
                mesh_list.points.append(p)
        self.pub_surfmesh.publish(mesh_list)
        return points

    def photoimage_visualization(self, TACTID, photodigits, is_pubmsg = True):
        GRIDSIZE = len(photodigits)
        resize_unit = 30
        offx = self.PADPHOTOX - (self.PADPHOTOX).min()
        offy = self.PADPHOTOY - (self.PADPHOTOY).min()

        if isinstance(self.pt_cst[TACTID], np.ndarray):
            photodigits_clip = np.clip(photodigits - self.pt_cst[TACTID], 0, 1)
        else:
            photodigits_clip = np.clip(photodigits - self.pt_cst[TACTID][-1], 0, 1)

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
        if is_pubmsg:
            photoimg = cv2.normalize(photoimg*255, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            ros_image = self.roscvbridge.cv2_to_imgmsg(photoimg, encoding="passthrough")
            self.pub_photoimage.publish(ros_image)
        else:
            cv2.imshow('photoshow-{}'.format(TACTID+1), photoimg)
            cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('manitactile_node', anonymous=True)
    app = ManiTactile()
    rospy.spin()