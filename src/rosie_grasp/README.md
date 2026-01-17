# Rosie learned grasping

`rosie_grasp` connects Rosie’s RGB-D, YOLO, and TF2 streams to Contact-GraspNet
inference. Run the model in a dedicated environment so its research dependencies
remain separate from the ROS 2 Humble workspace.

Install the official Contact-GraspNet checkout and create its environment on the
machine that has the checkpoint and GPU:

```bash
git clone https://github.com/NVlabs/contact_graspnet.git /opt/contact_graspnet
conda env create -f /opt/contact_graspnet/contact_graspnet_env.yml
conda activate contact_graspnet
source /opt/ros/humble/setup.bash
source install/setup.bash
python /path/to/rosie_grasp/scripts/grasp_pose_node.py --ros-args \
  -p checkpoint_path:=/opt/contact_graspnet/checkpoints/<checkpoint>.pt
```

The backend accepts either `predict_scene(points, colors)` or `predict(points,
colors)` and returns candidates containing `position`, `quaternion`, and `score`
(or `confidence`), with optional `collision_free` metadata.
