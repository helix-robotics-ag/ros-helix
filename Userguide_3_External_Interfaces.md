# Controlling the robot through Foxglove
After launching the system on the Pi, go to `<IP_of_Pi>:8080` in a browser on your device. Choose 'Open Connection' and use `ws://<IP_of_Pi>:8765`. You should find and interface as below:

![foxglove_layout](https://github.com/helix-robotics-ag/ros-helix/assets/95340175/b5bd3271-5c53-42a2-936d-4171c3a0dfbd)

The elements of the interface are:
- A: All available topics are shown in this sidebar, or if the Panel tab is selected, settings for the currently highlighted panel are shown
- B: A 3D visualisation of the robot
- C: A plot showing each of the tendon state positions
- D: A table with all information from the `/tendon_states` topic. (This seems to be the best way to display all this information, despite the `name` field taking up a lot of space. If the fields under `position` etc just say `Object`, click to expand them)
- E: This panel will allow you to call the service chosen in the settings. Fill the Request field if needed.
- F: This panel will allow you to publish messages to a topic, which can be chosen in its settings (you may also need to set the correct message schema there)
- G: This panel will display the messages received on the topic name entered in the bar at the top of the panel, though the actual data can be hard to read
- H: Teleop panel (currently unused)

_Note if the interface is different your browser may have cached an old or modified version, you will need to clear your cookies or use Incognito mode to load the default one._

# Controlling the robot through `roslibpy`
[`roslibpy`](https://roslibpy.readthedocs.io/en/latest/#) can be used to access ROS topics, publish messages and make service calls through Python scripts. See [this full step by step demo notebook](https://github.com/helix-robotics-ag/main/blob/main/demos/roslibpy_demo.ipynb), which covers how to use all of these functions, and provides examples of basic Helix control.
