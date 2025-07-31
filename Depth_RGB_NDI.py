import cv2
import numpy as np
import depthai as dai
import threading
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from ttkbootstrap.tooltip import ToolTip
from tkinter import scrolledtext
import time
import NDIlib as ndi

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("OAK-D Camera Control Panel")
        self.device = None
        self.pipeline = None
        self.streams = []
        self.running = False
        self.stereo_node = None
        self.cv_windows = set()
        self.stop_event = threading.Event()
        self.thread = None

        from ttkbootstrap import StringVar, BooleanVar, IntVar

        self.startup_settings = {
            'resolution': StringVar(value='400'),
            'fps': IntVar(value=60),
            'rgb': BooleanVar(value=True)
        }

        self.settings = {
            'median': StringVar(value='7x7'),
            'lrcheck': BooleanVar(value=True),
            'extended': BooleanVar(value=True),
            'subpixel': BooleanVar(value=True),
        }

        self.status_var = StringVar(value="Disconnected")
        self.show_previews = BooleanVar(value=True)

        self.make_ui()
        self.build_pipeline()
        self.start_pipeline()

    def update_median_ui_state(self):
        if self.settings['extended'].get() or self.settings['subpixel'].get():
            self.median_combo.config(state="disabled")
            self.log("UI: Median filter disabled due to extended/subpixel mode.")
        else:
            self.median_combo.config(state="readonly")

            
    def make_ui(self):
        row = 0
        self.root.columnconfigure(1, weight=1)
       
        cam_frame = ttk.Labelframe(self.root, text="Live Stereo Settings (Adjustable During Stream)", padding=10)
        cam_frame.grid(row=row, column=0, columnspan=2, padx=10, pady=10, sticky="ew")
        row += 1

        ttk.Label(cam_frame, text="Disparity Median Filter").grid(row=0, column=0, sticky="w")
        self.median_combo = ttk.Combobox(cam_frame, textvariable=self.settings['median'], values=["OFF", "3x3", "5x5", "7x7"], width=10)
        self.median_combo.grid(row=0, column=1, sticky="ew")
        # ttk.Combobox(cam_frame, textvariable=self.settings['median'], values=["OFF", "3x3", "5x5", "7x7"], width=10).grid(row=0, column=1, sticky="ew")

        flags_frame = ttk.Labelframe(self.root, text="Stereo Depth Options", padding=10)
        flags_frame.grid(row=row, column=0, columnspan=2, padx=10, pady=(0,10), sticky="ew")
        row += 1

        label_map = {
            'lrcheck': "Left-Right Consistency Check",
            'extended': "Extended Disparity Range",
            'subpixel': "High Accuracy Subpixel Mode",
        }

        r = 0
        for key in label_map:
            cb = ttk.Checkbutton(flags_frame, text=label_map[key], variable=self.settings[key])
            cb.grid(row=r, column=0, columnspan=2, sticky="w", pady=2)
            ToolTip(cb, text=f"Toggle {label_map[key].lower()}")
            cb.config(command=self.update_median_ui_state)
            r += 1

        button_frame = ttk.Frame(self.root)
        button_frame.grid(row=row, column=0, columnspan=2, pady=(0, 10))
        ttk.Button(button_frame, text="Apply Live Settings", command=self.apply_live_settings, bootstyle=SUCCESS).grid(row=0, column=0, padx=5)
        row += 1

        preview_check = ttk.Checkbutton(self.root, text="Show Preview Windows", variable=self.show_previews)
        preview_check.grid(row=row, column=0, columnspan=2, sticky="w", padx=10, pady=(0,10))
        ToolTip(preview_check, text="Toggle preview display of RGB and depth windows")
        row += 1

        log_frame = ttk.Labelframe(self.root, text="Log", padding=10)
        log_frame.grid(row=row, column=0, columnspan=2, padx=10, pady=(0, 10), sticky="nsew")
        self.log_widget = scrolledtext.ScrolledText(log_frame, height=6, wrap="word", state="disabled")
        self.log_widget.pack(fill="both", expand=True)
        row += 1
        row += 1

        status_frame = ttk.Frame(self.root)
        status_frame.grid(row=row, column=0, columnspan=2, sticky="ew", padx=10, pady=(0, 5))
        ttk.Label(status_frame, text="Status:").pack(side="left")
        ttk.Label(status_frame, textvariable=self.status_var, bootstyle=INFO).pack(side="left")
        row += 1

        self.update_median_ui_state()

        
        
        

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        full_message = f"[{timestamp}] {message}\n"
        self.log_widget.configure(state="normal")
        self.log_widget.insert("end", full_message)
        self.log_widget.configure(state="disabled")
        self.log_widget.see("end")

    def build_pipeline(self):
        resolutionMap = {"800": (1280, 800), "720": (1280, 720), "400": (640, 400)}
        medianMap = {
            "OFF": dai.StereoDepthProperties.MedianFilter.MEDIAN_OFF,
            "3x3": dai.StereoDepthProperties.MedianFilter.KERNEL_3x3,
            "5x5": dai.StereoDepthProperties.MedianFilter.KERNEL_5x5,
            "7x7": dai.StereoDepthProperties.MedianFilter.KERNEL_7x7,
        }

        resolution = resolutionMap[self.startup_settings['resolution'].get()]
        median = medianMap[self.settings['median'].get()]
        fps = self.startup_settings['fps'].get()

        pipeline = dai.Pipeline()

        camLeft = pipeline.create(dai.node.MonoCamera)
        camRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setRuntimeModeSwitch(True)
        xoutDisparity = pipeline.create(dai.node.XLinkOut)
        xoutDisparity.setStreamName("disparity")

        camLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        camRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        res_enum = {
            800: dai.MonoCameraProperties.SensorResolution.THE_800_P,
            720: dai.MonoCameraProperties.SensorResolution.THE_720_P,
            400: dai.MonoCameraProperties.SensorResolution.THE_400_P
        }[resolution[1]]

        camLeft.setResolution(res_enum)
        camRight.setResolution(res_enum)

        stereo.initialConfig.setMedianFilter(median)
        stereo.setLeftRightCheck(self.settings['lrcheck'].get())
        stereo.setExtendedDisparity(self.settings['extended'].get())
        stereo.setSubpixel(self.settings['subpixel'].get())

        if self.settings['extended'].get() or self.settings['subpixel'].get():
            stereo.initialConfig.setMedianFilter(dai.StereoDepthProperties.MedianFilter.MEDIAN_OFF)
            self.log("Median filter disabled automatically due to extended/subpixel disparity.")
        else:
            stereo.initialConfig.setMedianFilter(median)

        camLeft.out.link(stereo.left)
        camRight.out.link(stereo.right)
        stereo.disparity.link(xoutDisparity.input)

        streams = ["disparity"]

        if self.startup_settings['rgb'].get():
            camRgb = pipeline.create(dai.node.ColorCamera)
            rgbOut = pipeline.create(dai.node.XLinkOut)
            rgbOut.setStreamName("rgb")
            camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setFps(fps)
            camRgb.setVideoSize(640, 400)
            camRgb.isp.link(rgbOut.input)
            streams.append("rgb")

        self.pipeline = pipeline
        self.streams = streams
        self.stereo_node = stereo
        config_in = pipeline.create(dai.node.XLinkIn)
        config_in.setStreamName("stereo_config")
        config_in.out.link(stereo.inputConfig)
        self.stereo_config_stream = "stereo_config"

    def start_pipeline(self):
        self.status_var.set("Waiting for device...")
        self.stop_event.clear()
        self.device = dai.Device(self.pipeline)
        self.qList = [self.device.getOutputQueue(name, 8, blocking=False) for name in self.streams]
        self.config_queue = self.device.getInputQueue(self.stereo_config_stream)
        self.running = True
        self.cv_windows.clear()
        self.thread = threading.Thread(target=self.display_frames, daemon=True)
        self.thread.start()
        self.status_var.set("Running...")
        self.log("Pipeline started")

    def display_frames(self):
        ndi_rgb, vf_rgb = self.init_ndi("ndi-rgb") if 'rgb' in self.streams else (None, None)
        ndi_disp, vf_disp = self.init_ndi("ndi-disparity")

        while self.running and not self.stop_event.is_set():
            for q in self.qList:
                name = q.getName()
                frame = q.get().getCvFrame()

                if name == "disparity":
                    maxDisp = 96
                    frame = (frame * (255.0 / maxDisp)).astype(np.uint8)
                    if ndi_disp:
                        self.send_ndi(frame, ndi_disp, vf_disp)
                elif name == "rgb" and ndi_rgb:
                    self.send_ndi(frame, ndi_rgb, vf_rgb, color=True)

                if name not in self.cv_windows:
                    self.cv_windows.add(name)

                if self.show_previews.get():
                    cv2.imshow(name, frame)

            if cv2.waitKey(1) == ord('q'):
                self.running = False
                self.log("Preview manually stopped (GUI still running)")
                return

        if ndi_rgb: ndi.send_destroy(ndi_rgb)
        if ndi_disp: ndi.send_destroy(ndi_disp)
        ndi.destroy()
        cv2.destroyAllWindows()

    def apply_live_settings(self):
        if not hasattr(self, 'config_queue') or self.config_queue is None:
            self.log("Live settings not available (no config queue).")
            return

        cfg = dai.StereoDepthConfig()
        medianMap = {
            "OFF": dai.StereoDepthProperties.MedianFilter.MEDIAN_OFF,
            "3x3": dai.StereoDepthProperties.MedianFilter.KERNEL_3x3,
            "5x5": dai.StereoDepthProperties.MedianFilter.KERNEL_5x5,
            "7x7": dai.StereoDepthProperties.MedianFilter.KERNEL_7x7,
        }
        cfg.setMedianFilter(medianMap[self.settings['median'].get()])
        cfg.setLeftRightCheck(self.settings['lrcheck'].get())
        cfg.setExtendedDisparity(self.settings['extended'].get())
        cfg.setSubpixel(self.settings['subpixel'].get())

        if self.settings['extended'].get() or self.settings['subpixel'].get():
            cfg.setMedianFilter(dai.StereoDepthProperties.MedianFilter.MEDIAN_OFF)
            self.log("Live: Median filter disabled due to extended/subpixel disparity.")

        self.config_queue.send(cfg)
        self.log("Live settings applied")

    def init_ndi(self, name):
        if not ndi.initialize():
            self.log(f"NDI init failed for {name}")
            return None, None
        settings = ndi.SendCreate()
        settings.ndi_name = name
        sender = ndi.send_create(settings)
        frame = ndi.VideoFrameV2()
        return sender, frame

    def send_ndi(self, frame, sender, video_frame, color=False):
        if color:
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
        else:
            img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGRA)
        video_frame.data = img
        video_frame.FourCC = ndi.FOURCC_VIDEO_TYPE_BGRX
        ndi.send_send_video_v2(sender, video_frame)

if __name__ == "__main__":
    root = ttk.Window(themename="superhero")
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()
