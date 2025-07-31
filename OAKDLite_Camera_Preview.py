import depthai as dai
import cv2
import threading
import tkinter as tk
from ttkbootstrap import Style
from ttkbootstrap.widgets import Scale, Label, Combobox

class OakMultiCamApp:
    def __init__(self, root):
        self.root = root
        self.root.title("OAK-D Lite Multi-Camera Control")
        self.style = Style("flatly")

        self.device = None
        self.running = True
        self.selected_stream = tk.StringVar(value="CAM_A")

        self.control_queues = {}
        self.video_queues = {}
        self.stream_frames = {}

        self.setup_ui()
        self.build_pipeline()

        self.update_thread = threading.Thread(target=self.update_video, daemon=True)
        self.update_thread.start()

    def setup_ui(self):
        Label(self.root, text="View Camera").grid(row=0, column=0, sticky="w")
        stream_select = Combobox(self.root, textvariable=self.selected_stream,
                                 values=["CAM_A", "CAM_B", "CAM_C"])
        stream_select.grid(row=0, column=1)
        stream_select.bind("<<ComboboxSelected>>", self.update_displayed_stream)

        # Exposure
        Label(self.root, text="Exposure (µs)").grid(row=1, column=0, sticky="w")
        self.exposure_slider = Scale(self.root, from_=100, to=33000, orient="horizontal", command=self.send_settings)
        self.exposure_slider.set(10000)
        self.exposure_slider.grid(row=1, column=1)

        # ISO
        Label(self.root, text="ISO").grid(row=2, column=0, sticky="w")
        self.iso_slider = Scale(self.root, from_=100, to=1600, orient="horizontal", command=self.send_settings)
        self.iso_slider.set(400)
        self.iso_slider.grid(row=2, column=1)

        # White Balance
        Label(self.root, text="White Balance (K) [Color only]").grid(row=3, column=0, sticky="w")
        self.wb_slider = Scale(self.root, from_=1000, to=12000, orient="horizontal", command=self.send_settings)
        self.wb_slider.set(4000)
        self.wb_slider.grid(row=3, column=1)

    def build_pipeline(self):
        pipeline = dai.Pipeline()

        # CAM_A - Color
        color_cam = pipeline.createColorCamera()
        color_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        color_cam.setFps(30)

        xout_color = pipeline.createXLinkOut()
        xout_color.setStreamName("video_CAM_A")
        color_cam.video.link(xout_color.input)

        ctrl_color = pipeline.createXLinkIn()
        ctrl_color.setStreamName("control_CAM_A")
        ctrl_color.out.link(color_cam.inputControl)

        # CAM_B - Mono
        mono_b = pipeline.createMonoCamera()
        mono_b.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_b.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        mono_b.setFps(30)

        xout_b = pipeline.createXLinkOut()
        xout_b.setStreamName("video_CAM_B")
        mono_b.out.link(xout_b.input)

        ctrl_b = pipeline.createXLinkIn()
        ctrl_b.setStreamName("control_CAM_B")
        ctrl_b.out.link(mono_b.inputControl)

        # CAM_C - Mono
        mono_c = pipeline.createMonoCamera()
        mono_c.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_c.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        mono_c.setFps(30)

        xout_c = pipeline.createXLinkOut()
        xout_c.setStreamName("video_CAM_C")
        mono_c.out.link(xout_c.input)

        ctrl_c = pipeline.createXLinkIn()
        ctrl_c.setStreamName("control_CAM_C")
        ctrl_c.out.link(mono_c.inputControl)

        # Start device
        self.device = dai.Device(pipeline)

        # Setup queues
        for cam in ["CAM_A", "CAM_B", "CAM_C"]:
            self.control_queues[cam] = self.device.getInputQueue(f"control_{cam}")
            self.video_queues[cam] = self.device.getOutputQueue(f"video_{cam}", maxSize=4, blocking=False)
            self.stream_frames[cam] = None

        self.send_settings()

    def send_settings(self, *_):
        cam = self.selected_stream.get()
        if cam not in self.control_queues:
            return

        ctrl = dai.CameraControl()
        ctrl.setManualExposure(int(self.exposure_slider.get()), int(self.iso_slider.get()))
        if cam == "CAM_A":
            ctrl.setManualWhiteBalance(int(self.wb_slider.get()))
        self.control_queues[cam].send(ctrl)

    def update_displayed_stream(self, *_):
        self.send_settings()

    def update_video(self):
        while self.running:
            for cam in ["CAM_A", "CAM_B", "CAM_C"]:
                try:
                    if self.video_queues[cam].has():
                        self.stream_frames[cam] = self.video_queues[cam].get().getCvFrame()
                except:
                    continue

            selected = self.selected_stream.get()
            frame = self.stream_frames.get(selected)
            if frame is not None:
                cv2.imshow("OAK-D Lite MultiCam", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                self.root.quit()
                break

        cv2.destroyAllWindows()

    def stop(self):
        self.running = False
        if self.device:
            self.device.close()

if __name__ == "__main__":
    root = tk.Tk()
    app = OakMultiCamApp(root)
    try:
        root.mainloop()
    finally:
        app.stop()
