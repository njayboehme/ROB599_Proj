Follow steps here for turtlebot simulation install: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup AND https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

To run
1) Create a ROS2 workspace
2) git clone the repo in the workspace
3) Create a virtual env in the workspace:
    virtualenv -p python3 ./venv
4) colcon build in the workspace
5) Open a new window
6) Source the venv
    source ./venv/bin/activate
7) pip install -q -U google-genai (https://ai.google.dev/api?lang=python)
8) export PYTHONPATH="<PATH_TO_VENV>/lib/python<VERSION>/site-packages"
9) source install/setup.bash
10) ros2 run project vlm
11) Open another window and source it (no need to do the export)
12) ros2 run project text_client
13) Follow instructions on the text_client

# TODO
1) Get image from turtlebot env, save it, and send the path to the image to the VLM.
2) Given text and image, the VLM should produce a goal pose
3) Publish a goal pose to the turtlebot
4) Another goal pose should only be published when the robot has reached its previous goal pose.
5) I'd like to do this for the three turtlebot environments (https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/), but the house environment might be too hard. Minimum working should be the easiest environment.