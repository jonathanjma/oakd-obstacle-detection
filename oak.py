import depthai as dai
import cv2
import numpy as np


pipeline = dai.Pipeline()

# rgb_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
left_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
right_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

# rgb_stream = rgb_cam.requestOutput(size=(1280, 720), type=dai.ImgFrame.Type.NV12)
left_stream = left_cam.requestOutput(size=(1280, 800), fps=30, type=dai.ImgFrame.Type.NV12)
right_stream = right_cam.requestOutput(size=(1280, 800), fps=30, type=dai.ImgFrame.Type.NV12)

# Unfiltered depth stream.
stereo_raw = pipeline.create(dai.node.StereoDepth).build(left_stream, right_stream)
# stereo_raw.initialConfig.postProcessing.thresholdFilter.maxRange = 10000
stereo_raw.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)

# Filtered depth stream (confidence + temporal).
stereo_filtered = pipeline.create(dai.node.StereoDepth).build(left_stream, right_stream)
stereo_filtered.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
stereo_filtered.initialConfig.setConfidenceThreshold(40)
stereo_filtered.initialConfig.postProcessing.temporalFilter.enable = True
stereo_filtered.initialConfig.postProcessing.temporalFilter.alpha = 0.65
stereo_filtered.initialConfig.postProcessing.temporalFilter.persistencyMode = dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_3

depthQueueRaw = stereo_raw.depth.createOutputQueue()
depthQueueFiltered = stereo_filtered.depth.createOutputQueue()

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_TURBO)
colorMap[0] = [0, 0, 0]  # to make zero-depth pixels black

hover_depth = 0
def on_mouse(event, x, y, flags, param):
    global hover_depth
    if event == cv2.EVENT_MOUSEMOVE:
        frame = param
        if y < frame.shape[0] and x < frame.shape[1]:
            hover_depth = frame[y, x]/1000.0  # Convert from mm to meters

# Create a RemoteConnection (Visualizer server)
# visualizer = dai.RemoteConnection()
# # visualizer.addTopic("rgb", rgb_stream)
# visualizer.addTopic("depth", stereo.depth)
# visualizer.addTopic("confidence", stereo.confidenceMap)
# pipeline.build()
# visualizer.registerPipeline(pipeline)

with pipeline:
    pipeline.start()
    maxDepthRaw = 1
    maxDepthFiltered = 1
    while pipeline.isRunning():
        depthRaw = depthQueueRaw.get()
        depthFiltered = depthQueueFiltered.get()

        npDepthRaw = depthRaw.getCvFrame()
        npDepthFiltered = depthFiltered.getCvFrame()

        maxDepthRaw = max(maxDepthRaw, np.max(npDepthRaw))
        maxDepthFiltered = max(maxDepthFiltered, np.max(npDepthFiltered))

        colorizedDepthRaw = cv2.applyColorMap(((npDepthRaw / maxDepthRaw) * 255).astype(np.uint8), colorMap)
        colorizedDepthFiltered = cv2.applyColorMap(((npDepthFiltered / maxDepthFiltered) * 255).astype(np.uint8), colorMap)

        cv2.setMouseCallback("depth", on_mouse, param=npDepthRaw)
        cv2.putText(
            colorizedDepthRaw,
            f"Depth at mouse: {hover_depth}m",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
        )

        cv2.imshow("depth_raw", colorizedDepthRaw)
        cv2.imshow("depth_filtered", colorizedDepthFiltered)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break

        # Check for key presses in the Visualizer
        # if visualizer.waitKey(1) == ord('q'):
        #     # If 'q' is pressed, stop the pipeline
        #     pipeline.stop()
