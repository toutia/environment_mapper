import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
from threading import Thread, Lock
import configparser
import pyds
import numpy as np
import os
import sys


class ObjectDetectionPipeline:

    def __init__(self):

        Gst.init(None)
        self.pipeline = Gst.Pipeline()
        self.loop = GLib.MainLoop()
        self.target_object = "person"
        self.lables = []
        self.init_pipeline()
        self.depth_buffer = None
        self.depth_lock = Lock()
        self.depth_unit = 0.0010000000474974513

    def get_by_name(self, name):
        return self.pipeline.get_by_name(name)

    def load_labels(
        self,
        file_path=os.path.join(
            os.path.dirname(__file__), "Primary_Detector/labels.txt"
        ),
    ):
        with open(file_path, "r") as f:
            self.labels = [line.strip() for line in f.readlines()]

    def init_pipeline(self):

        self.load_labels()

        source = Gst.ElementFactory.make("appsrc", "rs-source")
        if not source:
            sys.stderr.write(" Unable to create Source \n")

        caps_v4l2src = Gst.ElementFactory.make("capsfilter", "v4l2src_caps")
        if not caps_v4l2src:
            sys.stderr.write(" Unable to create v4l2src capsfilter \n")

        vidconvsrc = Gst.ElementFactory.make("videoconvert", "convertor_src1")
        if not vidconvsrc:
            sys.stderr.write(" Unable to create videoconvert \n")

        # nvvideoconvert to convert incoming raw buffers to NVMM Mem (NvBufSurface API)
        nvvidconvsrc = Gst.ElementFactory.make("nvvideoconvert", "convertor_src2")
        if not nvvidconvsrc:
            sys.stderr.write(" Unable to create Nvvideoconvert \n")

        caps_vidconvsrc = Gst.ElementFactory.make("capsfilter", "nvmm_caps")
        if not caps_vidconvsrc:
            sys.stderr.write(" Unable to create capsfilter \n")

        # Create nvstreammux instance to form batches from one or more sources.
        streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
        if not streammux:
            sys.stderr.write(" Unable to create NvStreamMux \n")

        # Use nvinfer to run inferencing on camera's output,
        # behaviour of inferencing is set through config file
        pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
        if not pgie:
            sys.stderr.write(" Unable to create pgie \n")

        tracker = Gst.ElementFactory.make("nvtracker", "tracker")
        if not tracker:
            sys.stderr.write(" Unable to create tracker \n")

        # Use convertor to convert from NV12 to RGBA as required by nvosd
        nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
        if not nvvidconv:
            sys.stderr.write(" Unable to create nvvidconv \n")

        # Create OSD to draw on the converted RGBA buffer
        nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
        if not nvosd:
            sys.stderr.write(" Unable to create nvosd \n")

        # redirect to defaulmt diplay
        sink = Gst.ElementFactory.make("nv3dsink", "nv3d-sink")
        if not sink:
            sys.stderr.write(" Unable to create egl sink \n")
        sink.set_property("sync", False)

        caps_v4l2src.set_property(
            "caps",
            Gst.Caps.from_string(
                "video/x-raw, framerate=30/1, width=1280, height=1440, format=RGB"
            ),
        )
        caps_vidconvsrc.set_property(
            "caps", Gst.Caps.from_string("video/x-raw(memory:NVMM)")
        )
        source.set_property("is-live", True)
        source.set_property("format", Gst.Format.TIME)

        nvvidconvsrc.set_property("compute-hw", 1)
        streammux.set_property(
            "width", 1280
        )  # 1920 is a standard 16:9 full HD resolution
        streammux.set_property("height", 1440)  # Similarly, for 1080p
        streammux.set_property("batch-size", 1)
        streammux.set_property("batched-push-timeout", 10000)
        pgie.set_property(
            "config-file-path",
            os.path.join(os.path.dirname(__file__), "pgie_config.txt"),
        )
        # Set properties of tracker
        config = configparser.ConfigParser()
        config.read(os.path.join(os.path.dirname(__file__), "tracker_config.txt"))
        config.sections()
        for key in config["tracker"]:
            if key == "tracker-width":
                tracker_width = config.getint("tracker", key)
                tracker.set_property("tracker-width", tracker_width)
            if key == "tracker-height":
                tracker_height = config.getint("tracker", key)
                tracker.set_property("tracker-height", tracker_height)
            if key == "gpu-id":
                tracker_gpu_id = config.getint("tracker", key)
                tracker.set_property("gpu_id", tracker_gpu_id)
            if key == "ll-lib-file":
                tracker_ll_lib_file = config.get("tracker", key)
                tracker.set_property("ll-lib-file", tracker_ll_lib_file)
            if key == "ll-config-file":
                tracker_ll_config_file = config.get("tracker", key)
                tracker.set_property("ll-config-file", tracker_ll_config_file)

        print("Adding elements to Pipeline \n")
        self.pipeline.add(source)
        self.pipeline.add(caps_v4l2src)
        self.pipeline.add(vidconvsrc)
        self.pipeline.add(nvvidconvsrc)
        self.pipeline.add(caps_vidconvsrc)
        self.pipeline.add(streammux)
        self.pipeline.add(pgie)
        self.pipeline.add(tracker)
        self.pipeline.add(nvvidconv)
        self.pipeline.add(nvosd)
        self.pipeline.add(sink)

        print("Linking elements in the Pipeline \n")
        source.link(caps_v4l2src)
        caps_v4l2src.link(vidconvsrc)
        vidconvsrc.link(nvvidconvsrc)
        nvvidconvsrc.link(caps_vidconvsrc)
        sinkpad = streammux.request_pad_simple("sink_0")
        if not sinkpad:
            sys.stderr.write(" Unable to get the sink pad of streammux \n")
        srcpad = caps_vidconvsrc.get_static_pad("src")
        if not srcpad:
            sys.stderr.write(" Unable to get source pad of caps_vidconvsrc \n")
        srcpad.link(sinkpad)
        streammux.link(pgie)
        pgie.link(tracker)
        tracker.link(nvvidconv)
        nvvidconv.link(nvosd)
        nvosd.link(sink)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.bus_call)

        # Lets add probe to get informed of the meta data generated, we add probe to
        # the sink pad of the osd element, since by that time, the buffer would have
        # had got all the metadata.
        osdsinkpad = nvosd.get_static_pad("sink")
        if not osdsinkpad:
            sys.stderr.write(" Unable to get sink pad of nvosd \n")
        # passing the pitch element here to be able to control it dynamically
        osdsinkpad.add_probe(Gst.PadProbeType.BUFFER, self.osd_sink_pad_buffer_probe)

    def start(self):
        # Start the pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

        # Start the main loop in a separate thread
        Thread(target=self.loop.run, daemon=True).start()

    def stop(self):

        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()

    def set_target(self, target_object):
        self.target_object = target_object

    def bus_call(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            sys.stdout.write("End-of-stream\n")
            self.loop.quit()
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            sys.stderr.write("Warning: %s: %s\n" % (err, debug))
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            sys.stderr.write("Error: %s: %s\n" % (err, debug))
            self.loop.quit()
        return True

    def osd_sink_pad_buffer_probe(self, pad, info):

        frame_number = 0
        # Initialize object counter for all classes
        obj_counter = {class_id: 0 for class_id in range(len(self.labels))}
        num_rects = 0

        gst_buffer = info.get_buffer()
        if not gst_buffer:
            print("Unable to get GstBuffer")
            return

        # Fetch and process the latest depth frame corresponding to the current GStreamer pipeline's probe
        with self.depth_lock:
            current_depth = (
                self.depth_buffer.copy() if self.depth_buffer is not None else None
            )

        if current_depth is None:
            print("No depth data available yet.")
            return Gst.PadProbeReturn.OK

        # Retrieve batch metadata from the gst_buffer
        batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))

        # Create summary display text with counts of all detected objects
        display_meta = pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        display_meta.num_labels = 1
        py_nvosd_text_params = display_meta.text_params[0]

        l_frame = batch_meta.frame_meta_list
        while l_frame is not None:
            try:
                frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
            except StopIteration:
                break
            target_found = False
            average_depth = 6
            # timestamp= frame_meta.ntp_timestamp
            # print(timestamp)
            frame_number = frame_meta.frame_num
            num_rects = frame_meta.num_obj_meta
            l_obj = frame_meta.obj_meta_list
            while l_obj is not None:
                try:
                    obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
                except StopIteration:
                    break

                # Increment the object count for the detected class
                obj_counter[obj_meta.class_id] += 1
                # Use the class label from labels.txt
                class_name = self.labels[obj_meta.class_id]

                if class_name == self.target_object:
                    target_found = True
                    # Extract bounding box coordinates
                    left = int(obj_meta.rect_params.left)
                    top = int(obj_meta.rect_params.top)
                    width = int(obj_meta.rect_params.width)
                    height = int(obj_meta.rect_params.height)

                    if current_depth is not None:
                        # Define ROI based on bounding box and calculate average depth
                        target_depth_region = current_depth[
                            top : top + height, left : left + width
                        ]
                        average_depth = (
                            np.mean(target_depth_region) * self.depth_unit
                        )  # Average depth
                        print(
                            f"Average depth for target object: {average_depth } meters"
                        )
                        # Calculate center coordinates
                        # u = left + width // 2
                        # v = top + height // 2
                        # spatial_coordinates = get_spatial_coordinates(u, v, average_depth, camera_intrinsics)
                        # x,y,z= spatial_coordinates
                        # description= generate_spatial_directive(x,y,z)
                        # print(description)

                    break  # Exit
                py_nvosd_text_params.display_text = class_name
                pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)

                try:
                    l_obj = l_obj.next
                except StopIteration:
                    break

            # Display detected class names and their counts
            detected_classes = [
                f"{self.labels[class_id]}: {count}"
                for class_id, count in obj_counter.items()
                if count > 0
            ]
            summary_text = f"Frame {frame_number}, Objects: {num_rects}\n" + ", ".join(
                detected_classes
            )
            py_nvosd_text_params.display_text = summary_text
            py_nvosd_text_params.x_offset = 10
            py_nvosd_text_params.y_offset = 12
            py_nvosd_text_params.font_params.font_name = "Serif"
            py_nvosd_text_params.font_params.font_size = 10
            py_nvosd_text_params.font_params.font_color.set(1.0, 1.0, 1.0, 1.0)
            py_nvosd_text_params.set_bg_clr = 1
            py_nvosd_text_params.text_bg_clr.set(0.0, 0.0, 0.0, 1.0)
            pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)

            try:
                l_frame = l_frame.next
            except StopIteration:
                break

            # change_pitch(target_found, pitch, volume, average_depth)

        # #past tracking meta data
        # l_user=batch_meta.batch_user_meta_list
        # while l_user is not None:
        #     try:
        #         # Note that l_user.data needs a cast to pyds.NvDsUserMeta
        #         # The casting is done by pyds.NvDsUserMeta.cast()
        #         # The casting also keeps ownership of the underlying memory
        #         # in the C code, so the Python garbage collector will leave
        #         # it alone
        #         user_meta=pyds.NvDsUserMeta.cast(l_user.data)
        #     except StopIteration:
        #         break
        #     if(user_meta and user_meta.base_meta.meta_type==pyds.NvDsMetaType.NVDS_TRACKER_PAST_FRAME_META):
        #         try:
        #             # Note that user_meta.user_meta_data needs a cast to pyds.NvDsTargetMiscDataBatch
        #             # The casting is done by pyds.NvDsTargetMiscDataBatch.cast()
        #             # The casting also keeps ownership of the underlying memory
        #             # in the C code, so the Python garbage collector will leave
        #             # it alone
        #             pPastDataBatch = pyds.NvDsTargetMiscDataBatch.cast(user_meta.user_meta_data)
        #         except StopIteration:
        #             break
        #         for miscDataStream in pyds.NvDsTargetMiscDataBatch.list(pPastDataBatch):
        #             print("streamId=",miscDataStream.streamID)
        #             print("surfaceStreamID=",miscDataStream.surfaceStreamID)
        #             for miscDataObj in pyds.NvDsTargetMiscDataStream.list(miscDataStream):
        #                 print("numobj=",miscDataObj.numObj)
        #                 print("uniqueId=",miscDataObj.uniqueId)
        #                 print("classId=",miscDataObj.classId)
        #                 print("objLabel=",miscDataObj.objLabel)
        #                 for miscDataFrame in pyds.NvDsTargetMiscDataObject.list(miscDataObj):
        #                     print('frameNum:', miscDataFrame.frameNum)
        #                     print('tBbox.left:', miscDataFrame.tBbox.left)
        #                     print('tBbox.width:', miscDataFrame.tBbox.width)
        #                     print('tBbox.top:', miscDataFrame.tBbox.top)
        #                     print('tBbox.right:', miscDataFrame.tBbox.height)
        #                     print('confidence:', miscDataFrame.confidence)
        #                     print('age:', miscDataFrame.age)
        #     try:
        #         l_user=l_user.next
        #     except StopIteration:
        #         break

        return Gst.PadProbeReturn.OK


if __name__ == "__main__":
    pipeline = ObjectDetectionPipeline()
    pipeline.start()
