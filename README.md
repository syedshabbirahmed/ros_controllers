# robotic-coursework-f2022

# How to run


# base_controller [A8]

[1] Design Details

    Planning algoritm

    Planner: Potential Field Planner (Attractive and Repulsive) with intermediate targets specified

    Obstacle avoidance: Repulsive potentail field planner + Logic - turn in the opposite direction of obstacle

[2] Sensors

    2D Lidar: HUSKY_LMS1XX_ENABLED

[3] How to run:

    In terminal enter: catkin build base_controller

    In a separate terminal enter: export HUSKY_LMS1XX_ENABLED=1

    In the same terminal as above enter: roslaunch base_controller xxx.launch
    where xxx = mud, cafe, agriculture, office

    In a separate terminal enter: rosservice call /base_controller/start

[4] Limitation: 

    The epsilon in the *.yaml files represent the tolerance on
    the x-y coordinate of the target location that the robot has to reach to. 
    If it is less than 0.5, due to the physical limitations of the robot,
    the robot will turn around and try to reorient itself for a number of times till it reaches the desired location.

    This may take a long while, 3 to 4 mins, but the robot will eventually 
    reach the target location. For your conveniece, I have set it at 0.5 or the smallest value so that the robot reaches the target location in its first try for now.
    
    If you need a higher accuracy, plese reduce epsilon 
    to anywhere between 0.3 to 0.5. Please dont not change epsilon
    if you desire that the robot stops at the target location in its first try
    with a tolerance of 0.5. If the robot completely fails in reaching the 
    target for a long tim, please restart the run.



# highlevel_controller [A7]

[1] Design Details
    
    Planner: Potential Field Planner
    Controller: Inverse Dynamics Controller

[2] Problem Handling:
    
    Used ROS actions. Have two executables highlevel_controller_client and highlevel_controller_server each running the following:

    highlevel_controller_client: highlevel_controller_client.cpp, ActionClient.cpp 

    highlevel_controller_server: highlevel_controller_server.cpp, ActionServer.cpp  

[3] How to run A7:

    In the terminal run: roslaunch highlevel_controller a7.launch

[4] Modifying Pick and Place sequence

    Step 1:
    Open the following file in vscode or a text editor:
    ~/catkin_ws/src/robotic-coursework-f2022/highlevel_controller/config/a7.yaml

    Step 2:
    Pick and place sequence:
        1. home,
        2. default_position,
        3. open_gripper,
        4. pregrasp_position,
        5. grasp,
        6. release_position,
        7. release

    Change x, y, and z co-ordinates under:
    home, default_position, pregrasp_position, release_position

    or change position and max-effort under:
    open_gripper, grasp, release

    The names of the sequences are self explanatory.

[5] Gripper details
    
    Opening position: default_position [end-effectorposition: (0.0,0.6,0.4)] , (position:0.4, max_effort:0.5)
    Closing position: pregrasp_position [end-effector position: (0.0,0.6,0.25)], (position:0.8, max_effort:1.0)
    
    Wait 7 seconds between closing position and moving to release position

[6] End-effector Target: 

    End-effector target: release_position [end-effector position: (0.5,0.3,0.29)], (position:0.5, max_effort:0.5)

[7] Success rate

    The object is successfully grasped and placed in the bowl at all times.

[8] Limitation

    The target end-effector orientation is always (0,0,0) for now.

# highlevel_controller [A6]

[1] Design Details
    Planner: Potential Field Planner
    Controller: Inverse Dynamics Controller

[2] Problem Handling:
    Used ROS actions. Have two executables highlevel_controller_client and highlevel_controller_server each running the following:

    highlevel_controller_client: highlevel_controller_client.cpp, ActionClient.cpp 

    highlevel_controller_server: highlevel_controller_server.cpp, ActionServer.cpp  

[3] world file location:
    ~/catkin_ws/src/robotic-coursework-f2022/highlevel_controller/worlds

[4] How to run A6:

    Step 1:
        bash setup:
        Option 1:
            [1] Open ~/.bashrc using a text editor in terminal: gedit ~/.bashrc  
            [2] Paste the following line in ~/.bashrc and save file:
                export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/robotic-coursework-f2022/highlevel_controller
        Option 2:
            [1] Open terminal and run: 
                export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/robotic-coursework-f2022/highlevel_controller

    Step 2:
    In the same terminal run: roslaunch highlevel_controller a6.launch

[5] Modifying target positions

    Step 1:
    Open the following file in vscode or a text editor:
    ~/catkin_ws/src/robotic-coursework-f2022/highlevel_controller/config/a6.yaml

    Step 2:
    Change x, y, and z co-ordinates under:
    target_1, target2, target_3

[6] Limitation
    The target end-effector orientation is always (0,0,0) for now.


# highlevel_controller (A5)
To move the robot to the desired location:

In a terminal input the following: [1] roslaunch highlevel_controller a5.launch
                            
No need to edit the urdf file or any other parameters in the code.


# inverse_dynamics_controller
For the robot to move to the desired location:

In different terminals input the following: [1] roslaunch kortex_gazebo gen3_dynamics.launch
                                            [2] roslaunch inverse_dynamics_controller a4.launch
                                            [3] rosservice call /planner/move_to 0.5 0.0 0.3
No need to edit the urdf file or any other parameters in the code.


<!-- ## Getting started -->

<!-- To make it easy for you to get started with GitLab, here's a list of recommended next steps.

Already a pro? Just edit this README.md and make it your own. Want to make it easy? [Use the template at the bottom](#editing-this-readme)! -->

<!-- ## Add your files -->

<!-- - [ ] [Create](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#upload-a-file) files
- [ ] [Add files using the command line](https://docs.gitlab.com/ee/gitlab-basics/add-file.html#add-a-file-using-the-command-line) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin https://gitlab.cs.mcgill.ca/sahmed98/robotic-coursework-f2022.git
git branch -M main
git push -uf origin main
``` -->

<!-- ## Integrate with your tools -->

<!-- - [ ] [Set up project integrations](https://gitlab.cs.mcgill.ca/sahmed98/robotic-coursework-f2022/-/settings/integrations) -->

<!-- ## Collaborate with your team -->

<!-- - [ ] [Invite team members and collaborators](https://docs.gitlab.com/ee/user/project/members/)
- [ ] [Create a new merge request](https://docs.gitlab.com/ee/user/project/merge_requests/creating_merge_requests.html)
- [ ] [Automatically close issues from merge requests](https://docs.gitlab.com/ee/user/project/issues/managing_issues.html#closing-issues-automatically)
- [ ] [Enable merge request approvals](https://docs.gitlab.com/ee/user/project/merge_requests/approvals/)
- [ ] [Automatically merge when pipeline succeeds](https://docs.gitlab.com/ee/user/project/merge_requests/merge_when_pipeline_succeeds.html) -->

<!-- ## Test and Deploy -->

<!-- Use the built-in continuous integration in GitLab.

- [ ] [Get started with GitLab CI/CD](https://docs.gitlab.com/ee/ci/quick_start/index.html)
- [ ] [Analyze your code for known vulnerabilities with Static Application Security Testing(SAST)](https://docs.gitlab.com/ee/user/application_security/sast/)
- [ ] [Deploy to Kubernetes, Amazon EC2, or Amazon ECS using Auto Deploy](https://docs.gitlab.com/ee/topics/autodevops/requirements.html)
- [ ] [Use pull-based deployments for improved Kubernetes management](https://docs.gitlab.com/ee/user/clusters/agent/)
- [ ] [Set up protected environments](https://docs.gitlab.com/ee/ci/environments/protected_environments.html) -->

***

<!-- # Editing this README -->

<!-- When you're ready to make this README your own, just edit this file and use the handy template below (or feel free to structure it however you want - this is just a starting point!). Thank you to [makeareadme.com](https://www.makeareadme.com/) for this template. -->

<!-- ## Suggestions for a good README -->
<!-- Every project is different, so consider which of these sections apply to yours. The sections used in the template are suggestions for most open source projects. Also keep in mind that while a README can be too long and detailed, too long is better than too short. If you think your README is too long, consider utilizing another form of documentation rather than cutting out information. -->

<!-- ## Name -->
<!-- Choose a self-explaining name for your project. -->

<!-- ## Description -->
<!-- Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors. -->

<!-- ## Badges -->
<!-- On some READMEs, you may see small images that convey metadata, such as whether or not all the tests are passing for the project. You can use Shields to add some to your README. Many services also have instructions for adding a badge. -->

<!-- ## Visuals -->
<!-- Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method. -->

<!-- ## Installation -->
<!-- Within a particular ecosystem, there may be a common way of installing things, such as using Yarn, NuGet, or Homebrew. However, consider the possibility that whoever is reading your README is a novice and would like more guidance. Listing specific steps helps remove ambiguity and gets people to using your project as quickly as possible. If it only runs in a specific context like a particular programming language version or operating system or has dependencies that have to be installed manually, also add a Requirements subsection. -->

<!-- ## Usage -->
<!-- Use examples liberally, and show the expected output if you can. It's helpful to have inline the smallest example of usage that you can demonstrate, while providing links to more sophisticated examples if they are too long to reasonably include in the README. -->

<!-- ## Support -->
<!-- Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc. -->

<!-- ## Roadmap -->
<!-- If you have ideas for releases in the future, it is a good idea to list them in the README. -->

<!-- ## Contributing -->
<!-- State if you are open to contributions and what your requirements are for accepting them. -->

<!-- For people who want to make changes to your project, it's helpful to have some documentation on how to get started. Perhaps there is a script that they should run or some environment variables that they need to set. Make these steps explicit. These instructions could also be useful to your future self. -->

<!-- You can also document commands to lint the code or run tests. These steps help to ensure high code quality and reduce the likelihood that the changes inadvertently break something. Having instructions for running tests is especially helpful if it requires external setup, such as starting a Selenium server for testing in a browser. -->

<!-- ## Authors and acknowledgment -->
<!-- Show your appreciation to those who have contributed to the project. -->

<!-- ## License -->
<!-- For open source projects, say how it is licensed. -->

<!-- ## Project status -->
<!-- If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers. -->
