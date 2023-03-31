from __future__ import annotations

from rclpy.node import Node
from ..sim_data_types import State, Action

import os
from pathlib import Path
import math

import ffmpeg
import numpy as np
import rowan as rw
from vispy import scene, app, io, geometry
from vispy.color import Color
from vispy.visuals import transforms
from vispy.scene.cameras import TurntableCamera


CF_PATH = Path(__file__).resolve().parent / "data/model/cf.obj"


class Visualization:
    """Renders video with vispy """

    def __init__(self, node: Node, params: dict, names: list[str], states: list[State]):
        self.node = node

        # read parameters
        self.outfile = params["output_file"]
        self.fps = params["fps"]    # frames per second
        self.fov = params["camera"]["fov"]
        self.elevation = params["camera"]["elevation_angle"]    # in degrees
        self.azimuth = params["camera"]["azimuth_angle"]        # in degrees
        self.cam_distance = params["camera"]["distance"]
        self.center = params["camera"]["center"]
        dpi = params["dpi"]
        show_window = params["show_window"]

        # initialize lists and state variables
        self.frame = 0
        self.ts = []
        self.names = names
        self.n = len(names)
        self.state_filenames = []
        self.cam_state_filenames = []

        # FIXME make window appear
        # init canvas and view
        self.canvas = scene.SceneCanvas(
            #keys='interactive',
            #size=(1024, 768), 
            #show=show_window,
            #resizable=False,
            dpi=dpi,
        )
        self.plane_color = 0.8 * np.ones((1, 3))
        self.bg_color = 0.9 * np.ones((1, 3))
        self.view = self.canvas.central_widget.add_view()
        self.view.bgcolor = self.bg_color
        self.view.camera = TurntableCamera(
            fov=self.fov, elevation=self.elevation, azimuth=self.azimuth, center=self.center, distance=self.cam_distance
        )
        self.cam_state = self.view.camera.get_state()
        self.fheight, self.fwidth = self.canvas.render(alpha=False).shape[:2]

        # axes
        scene.visuals.XYZAxis(parent=self.view.scene)
        # ground plane
        scene.visuals.Plane(
            30.0,
            30.0,
            direction="+z",
            width_segments=30,
            height_segments=30,
            color=self.plane_color,
            parent=self.view.scene,
        )

        # dictionary with (name, idx) pairs to later find corresponding cf
        self.names_idx_map = dict()
        # list of craziflies
        self.cf_list = []
        # read cf mesh
        self.cf_verts, self.cf_faces, normals, _ = io.read_mesh(CF_PATH)
        # set initial positions for all cfs 
        for idx, (name, state) in enumerate(zip(names, states)):
            self.names_idx_map[name] = idx
            # initialize cfs
            mesh = scene.visuals.Mesh(
                parent=self.view.scene,
                vertices=self.cf_verts,
                faces=self.cf_faces,
                shading="smooth",
            )
            mesh.unfreeze()
            mesh.light_dir = (0.1, 0.1, 1.0)
            mesh.shininess = 0.01
            mesh.ambient_light_color = [0.5] * 3
            T = np.eye(4)
            T[:3, :3] = rw.to_matrix(state.quat).T
            T[3, :3] = state.pos
            mesh.transform = transforms.MatrixTransform(T)

            self.cf_list.append(mesh)

        # initialize async ffmpeg process
        self.ffmpegProcess = (
            ffmpeg
            .input("pipe:", format="rawvideo", pix_fmt="rgb24", s=f"{self.fwidth}x{self.fheight}", r=self.fps)
            .output(self.outfile, vcodec="libx264", crf=18)
            .overwrite_output()
            .run_async(pipe_stdin=True)
        ) 


    def step(self, t, states: list[State], states_desired: list[State], actions: list[Action]):
        self.ts.append(t)
        # render and record at `self.fps` frames per second 
        if t * self.fps - self.frame >= 1:
            # place cfs 
            for name, state in zip(self.names, states):
                idx = self.names_idx_map[name]
                T = np.eye(4)
                # rotation
                T[:3, :3] = rw.to_matrix(state.quat).T
                # position
                T[3, :3] = state.pos
                # update cf position
                self.cf_list[idx].transform = transforms.MatrixTransform(T)

            # render frame without alpha channel 
            frame = self.canvas.render(alpha=False)
            # add frame to video 
            fb = frame.astype(np.uint8).tobytes()
            self.ffmpegProcess.stdin.write(fb)
            # increase frame count
            self.frame += 1

    def shutdown(self):
        # close output
        self.ffmpegProcess.stdin.close()
        self.ffmpegProcess.wait()
        self.ffmpegProcess = None
        print(f"wrote {self.frame} frames to {self.outfile}")


