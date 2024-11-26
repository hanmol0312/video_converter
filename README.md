# video_converter
This package has a image conversion server that will let user to change modes and thereby the modes of image.

# 1. Introduction:

- This document gives a detailed overview of how to build and execute this package to convert the video frame to a desired format.

# 2. Installation:

1. Create your workspace:

```
mkdir your_ws
cd your_ws
mkdir src
cd src
```

1. Clone the repository

```

git clone https://github.com/hanmol0312/video_converter.git
```

1. Build the package

```
cd ..
colcon build

#Source the ws in bash rc
nano ~/.bashrc
source ~/my_ws/install/setup.bash
source ~/.bashrc
```

# 3. Testing:

1. Once the package is built launch the usb_cam, making sure the usb_cam package is already installed.

```
ros2 launch image_conv_pkg image_conversion.launch.py image_server_input_topic:=cam/image_raw image_server_output_topic:=cam/converted
# Change the topics as desired
```

1. User interface to change the modes run the following node:

```
ros2 run image_conv_pkg image_conversion_node

# Now this will be visible:
[INFO] [1732643833.689225186] [image_conversion_node]: User Interface Node started. Waiting for service...

Select Image Mode:
1. Grayscale
2. Color
0. Exit
Enter your choice: 

```

1. Run rqt_image_view 

```
ros2 run rqt_image_view rqt_image_view
# Can check the output images here in gray and color as desired
```

# 4. Conclusion

- The desired outputs can be visualized by giving the inputs in the terminal.
- Check the image in rqt_image_view with video_feed
- If required while launching the image_conversion.launch.py the arguments can be changed for input and output topic.