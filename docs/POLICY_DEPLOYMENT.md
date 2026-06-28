# POLICY DEPLOYMENT & EVALUATION

This document outlines the end-to-end process from synthetic data generation to policy training (ACT / Diffusion Policy) and deployment in the MuJoCo simulation.

## 1. Data Collection

Run the synthetic data batch generator:
```bash
ros2 launch teleop_bringup data_collection.launch.py randomize:=true
ros2 run synth_data_gen batch_generator --episodes 50 --seed 2026
```

This will output LeRobot v2 compatible datasets in `data/episodes/`.

## 2. Policy Training

Use the `lerobot` framework to train an ACT policy. Ensure you have activated the correct training environment (e.g. `conda activate ros2-teleop`).

```bash
# Example training command
python lerobot/scripts/train.py \
  --dataset.repo_id="local://data/episodes" \
  --policy.type=act \
  --training.batch_size=8 \
  --training.epochs=50
```

## 3. Inference / Deployment

Once trained, the model can be evaluated in simulation (Sim2Sim). 
You need an inference node (e.g., `policy_inference_node.py`) that:
1. Subscribes to `/camera/color/image_raw` and `/joint_states`.
2. Runs the policy forward pass.
3. Publishes action to `/teleop/cmd_pose`.

> **Note:** The inference node replaces the `teleop_input_node`.

```bash
# Run simulation
ros2 launch teleop_bringup m1_control_sim.launch.py

# Run policy inference
ros2 run my_policy_pkg policy_inference_node --model-path ./checkpoints/act_model
```
