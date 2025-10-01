source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot

HF_USER=$(hf auth whoami | head -n 1)
echo $HF_USER

python src/lerobot/scripts/server/robot_client.py \
    --server_address=127.0.0.1:8080 \
    --robot.type=piper_follower \
    --robot.port=can0 \
    --robot.id=black \
    --robot.cameras="{ \
        top: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, \
        left: {type: opencv, index_or_path: 4, width: 640, height: 480, fps: 30}}" \
    --task="Grasp the object and put it in the bin" \
    --policy_type=smolvla \
    --pretrained_name_or_path=outputs/train/piper_smolvla_pick_yellow_cars/checkpoints/020000/pretrained_model \
    --policy_device=cuda \
    --actions_per_chunk=50  \
    --chunk_size_threshold=0.5 \
    --aggregate_fn_name=weighted_average \
    --debug_visualize_queue_size=True
