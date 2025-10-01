source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot

HF_USER=$(hf auth whoami | head -n 1)
echo $HF_USER

lerobot-replay \
--robot.type=piper_follower \
--robot.port=can0 \
--robot.id=black \
--dataset.repo_id=${HF_USER}/piper_pick_yellow_car \
--dataset.episode=0
