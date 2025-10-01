source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot

HF_USER=$(hf auth whoami | head -n 1)
echo $HF_USER

lerobot-record \
--robot.type=piper_follower \
--robot.port=can0 \
--robot.cameras="{ \
top: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, \
left: {type: opencv, index_or_path: 4, width: 640, height: 480, fps: 30}}" \
--robot.id=black   \
--dataset.num_episodes=50  \
--dataset.single_task="Grab the yellow car and put in the box"  \
--display_data=true   \
--dataset.repo_id=${HF_USER}/piper_pick_yellow_car
