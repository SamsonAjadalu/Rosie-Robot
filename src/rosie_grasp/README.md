# Rosie learned grasping

The ROS node is an adapter, not a vendored copy of Contact-GraspNet. This keeps
ROS2 Humble dependencies separate from the older research environment and avoids
silently installing incompatible TensorFlow/PyTorch packages into the workspace.

Install the selected Contact-GraspNet checkout in a dedicated environment, then
source ROS and run the node with that environment's Python on the machine that
has the checkpoint/GPU:

```bash
python3 -m venv /opt/rosie-contact-graspnet-venv
source /opt/rosie-contact-graspnet-venv/bin/activate
git clone <your-tested-contact-graspnet-checkout> /opt/contact_graspnet
pip install -r /opt/contact_graspnet/requirements.txt
source /opt/ros/humble/setup.bash
source install/setup.bash
python /path/to/rosie_grasp/scripts/grasp_pose_node.py --ros-args \
  -p checkpoint_path:=/opt/contact_graspnet/checkpoints/contact_graspnet.pt
```

The adapter accepts a backend class with either `predict_scene(points, colors)`
or `predict(points, colors)` and expects dictionaries containing `position`,
`quaternion`, `score` (or `confidence`), and optional `collision_free`. A local
Contact-GraspNet fork may need a tiny adapter at `GraspBackend.predict`; no
successful inference is claimed when the checkpoint or backend is unavailable.
