import bpy
import numpy as np
import os
#import shutil
import datetime
import itertools as it
import rowan as rw
import subprocess
from rclpy.node import Node
from ..sim_data_types import State, Action


class Visualization:


    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node

        # initial rotation to ensure that Z<0 is in front, Y>0 is up, and X>0 is right
        self._rot_world_2_cam = np.array([
            [ 0, 0,-1, 0],
            [-1, 0, 0, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 0, 1],
        ])

        ## blender 
        # import crazyflie object 
        bpy.ops.import_scene.obj(filepath="src/crazyswarm2/crazyflie_sim/data/cf.obj", axis_forward="Y", axis_up="Z")
        self.cf_default = bpy.data.objects["cf"]
        # save scene
        self.scene = bpy.context.scene
        self.scene.render.resolution_x = 320
        self.scene.render.resolution_y = 320
        self.scene.render.pixel_aspect_x = 1.0
        self.scene.render.pixel_aspect_y = 1.0
        self.scene.render.image_settings.file_format = "JPEG"
        self.scene.unit_settings.length_unit = "METERS"
        self.scene.unit_settings.system = "METRIC"
        self.scene.unit_settings.scale_length = 1.0
        #self.scene.render.threads = 2  # max CPU cores to use to render

        # remove default objects
        bpy.data.objects.remove(bpy.data.objects["Cube"])
        bpy.data.objects.remove(bpy.data.objects["Light"])
        # create lamp
        lamp_data = bpy.data.lights.new(name="Lamp", type="SUN")
        lamp_data.energy = 1.5
        lamp_data.angle = 0.19198621809482574  # 11 deg
        self.lamp = bpy.data.objects.new(name="Lamp", object_data=lamp_data)
        bpy.context.collection.objects.link(self.lamp)
        bpy.context.view_layer.objects.active = self.lamp
        # camera
        self.camera = bpy.data.objects["Camera"]
        self.camera.data.lens = 0.7376461029052734
        self.camera.data.lens_unit = "FOV"
        self.camera.data.sensor_fit = "AUTO"
        self.camera.data.sensor_width = 1.4
        self.camera.data.sensor_height = 18
        self.camera.data.angle = 1.518436431884765  # 87 deg
        self.camera.data.clip_start = 1.1e-6

        self.cf_default.hide_render = False
        # set rotation mode to quaternion
        self.cf_default.rotation_mode = "QUATERNION"
        self.camera.rotation_mode = "QUATERNION"
        self.lamp.rotation_mode = "QUATERNION"
        # use `self.node.get_clock().now().to_msg()` or similar for time instead?
        base = "simulation_results"  
        self.path = base + "/" + datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        os.makedirs(self.path, exist_ok=True) 
        self.ts = []
        self.frame = 0

        self.names = names
        self.n = len(names)
        self.state_filenames = []
        self.cam_state_filenames = []

        for name in names:
            os.mkdir(self.path + "/" + name + "/")  # create dir for every cf for saving images
            csf = f"{self.path}/{name}/{name}.csv"
            cam_sf = f"{self.path}/{name}/cam_{name}.csv"
            # initialize <robot_name>/<robot_name>.csv
            self.state_filenames.append(csf)
            self.cam_state_filenames.append(cam_sf)
            with open(csf, "w") as file:
                file.write("image_name,timestamp,x,y,z,qw,qx,qy,qz\n")
            with open(cam_sf, "w") as file:
                file.write("image_name,timestamp,x,y,z,qw,qx,qy,qz\n")

        # dictionary with (name, idx) pairs to later find corresponding cf
        self.names_idx_map = dict()
        # init list
        self.cf_list = [self.cf_default]

        for idx, (name, state) in enumerate(zip(names, states)):
            self.names_idx_map[name] = idx
            if idx == 0:
                self.cf_default.location = np.array(state.pos)
                # TODO: no initial orientation?
                #self.cf_default.rotation_quaternion = state.quat
            else:
                # cf copies
                cf_copy = self.cf_default.copy()
                bpy.context.collection.objects.link(cf_copy)
                self.cf_list.append(cf_copy)
                # place copy
                cf_copy.location = np.array(state.pos)
                # TODO: no initial orientation?
                #cf_copy.rotation_quaternion = state.quat

    def step(self, t, states: list[State], states_desired: list[State], actions: list[Action]):
        # only render and record every 150th frame:
        # TODO: configure interval (read from parameters)
        if self.frame % 150 == 0: 
            # TODO: configure which cfs have cameras on them (read from parameters)
            # quaternion matrix
            Q = np.zeros((self.n, 4))  
            # position matrix
            P = np.zeros((self.n, 3))
            # first put everything in place
            for name, state in zip(self.names, states):
                idx = self.names_idx_map[name]
                # set rotations
                Q[idx] = np.array(state.quat)
                self.cf_list[idx].rotation_quaternion = Q[idx]
                # set positions
                P[idx] = np.array(state.pos) 
                self.cf_list[idx].location = P[idx]

                
                # record states 
                image_name = f"{self.names[idx]}_{self.frame:05}.jpg"  # image capturing scene from cf's pov
                # record cfi's state in world frame
                with open(self.state_filenames[idx], "a") as file:
                    file.write(f"{image_name},{t},{P[idx,0]},{P[idx,1]},{P[idx,2]},{Q[idx,0]},{Q[idx,1]},{Q[idx,2]},{Q[idx,3]}\n")
                
                # render image from cf's pov
                # camera and lamp axes need to be adjusted for blender to capture the image as expected
                q_cam = rw.from_matrix(rw.to_matrix(Q[idx]) @ self._rot_world_2_cam[:3,:3])
                self.camera.rotation_quaternion = q_cam
                self.lamp.rotation_quaternion = q_cam
                # set camera and light positions
                p_cam = rw.to_matrix(Q[idx]) @ (P[idx] + np.array([0.025, 0.0, 0.01]))
                self.camera.location = p_cam
                self.lamp.location = p_cam
                # record camera state in world frame
                with open(self.cam_state_filenames[idx], "a") as file:
                    file.write(f"{image_name},{t},{p_cam[0]},{p_cam[1]},{p_cam[2]},{Q[idx,0]},{Q[idx,1]},{Q[idx,2]},{Q[idx,3]}\n")
                # hide corresponding cf for rendering
                self.cf_list[idx].hide_render = True
                # Render image
                self.scene.render.filepath = f"{self.path}/{self.names[idx]}/{image_name}"
                bpy.ops.render.render(write_still=True)
                # show again after rendering
                self.cf_list[idx].hide_render = False

            self.ts.append(t)
        self.frame += 1

    def shutdown(self):
        for idx in range(1,self.n):
            bpy.data.objects.remove(self.cf_list[idx])
