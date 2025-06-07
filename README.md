# VLM-Guided Path Planning

This project implements an end-to-end ROS 2 system to leverage the semantic reasoning ability of VLMs to guide a robot's path planning in a 2D maze.

## How to run this project locally
1. Create a new ROS 2 workspace and an `src` directory in its root.
2. Navigate to the `src` directy with `cd src` and clone this repository.
3. [Optional] Create a new Python3 virtual environment (`python3 -m venv /path/to/new/virtual/environment`) and source it (`source /path/to/new/virtual/environment/bin/activate`).
4. Install the python dependencies using `pip install -r requirement.txt`. This also includes the `google-genai` package for the Gemini Flash 2.0 VLM.
5. Add the current virtual environment to your path so that the native python installation (required for ROS 2) talks with the virtual environment: `export PYTHONPATH="/path/to/virtual/environment/lib/python3.12/site-packages"`.
6. Navigate to the workspace root `cd ..` and build the ROS 2 workspace with: `colcon build`.
7. Source the workspace with `source install/setup.bash #or setup.zsh`.
8. The `vlm_nav`, `vlm_nav_msgs`, `nav_game`, and `nav_game_msgs` packages should now be ready to use.
9. Run the VLM node with `ros2 run vlm_nav vlm`.
10. In a new terminal, again do `export PYTHONPATH="/path/to/virtual/environment/lib/python3.12/site-packages"`.
11. Source the workspace again with `source install/setup.bash`.
12. Launch the `base_prompter` and `problem_manager` nodes with `ros2 launch vlm_nav vlm_nav_scaffolding.launch.py`.
13. In a new terminal, again do `export PYTHONPATH="/path/to/virtual/environment/lib/python3.12/site-packages"` and `source install/setup.bash`.
14. Finally, run the 2D maze navigation game with `ros2 run nav_game nav_game`.
