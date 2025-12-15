module.exports = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/module-1-ros2/introduction-to-ros2',
        'modules/module-1-ros2/ros2-nodes-topics-services',
        'modules/module-1-ros2/python-agents-to-ros-controllers-via-rclpy',
        'modules/module-1-ros2/understanding-urdf-for-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/module-2-digital-twin/introduction-to-digital-twins',
        'modules/module-2-digital-twin/physics-simulation-in-gazebo',
        'modules/module-2-digital-twin/high-fidelity-rendering-in-unity',
        'modules/module-2-digital-twin/simulated-sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/module-3-isaac/introduction-to-nvidia-isaac-ecosystem',
        'modules/module-3-isaac/isaac-sim-photorealistic-simulation',
        'modules/module-3-isaac/isaac-ros-perception-vslam',
        'modules/module-3-isaac/nav2-for-humanoid-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/module-4-vla/understanding-vla-robotics',
        'modules/module-4-vla/voice-to-action-with-whisper',
        'modules/module-4-vla/cognitive-planning-using-llms',
        'modules/module-4-vla/capstone-project-the-autonomous-humanoid',
      ],
    },
  ],
};