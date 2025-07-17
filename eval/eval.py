import json
import glob
import os
import re
import motmetrics as mm
import pandas as pd
import argparse

# ==== Dataset Configuration ====
dataset_configs = {
    'Straight': {'frame_start': 260, 'frame_end': 340},
    'Intersection': {'frame_start': 120, 'frame_end': 200},
    'Aq-car': {'frame_start': 70, 'frame_end': 140},
    'Aeva-car': {'frame_start': 300, 'frame_end': 450}
    # 可以继续添加其他数据集配置
}

def get_class_id(obj_type):
    mapping = {'Pedestrian': 1, 'Bicycle': 2, 'Car': 3, 'Truck': 4}
    return mapping.get(obj_type, 1)

def filter_frames(file_path, frame_start, frame_end, output_path):
    data = pd.read_csv(file_path, header=None)
    data = data.iloc[:, :10]
    filtered_data = data[(data[0] >= frame_start) & (data[0] <= frame_end)]
    filtered_data.to_csv(output_path, header=False, index=False, sep=',')

def filter_frames_pre(file_path, frame_start, frame_end, output_path):
    data = pd.read_csv(file_path, header=None)
    data = data.iloc[:, :10]
    filtered_data = data[(data[0] >= frame_start) & (data[0] <= frame_end)].copy()

    swap_mask = filtered_data[4] < filtered_data[5]
    filtered_data.loc[swap_mask, [4, 5]] = filtered_data.loc[swap_mask, [5, 4]].values
    filtered_data.loc[swap_mask, 8] += 1.57

    filtered_data.to_csv(output_path, header=False, index=False, sep=',')

def run_evaluation(dataset_name):
    if dataset_name not in dataset_configs:
        print(f"Error: Unknown dataset '{dataset_name}'. Please add it to dataset_configs.")
        return

    config = dataset_configs[dataset_name]
    frame_start, frame_end = config['frame_start'], config['frame_end']

    # ==== Part 1: Label Generation ====
    folder_path = f'./output'
    json_files = sorted(glob.glob(os.path.join(folder_path, '*.json')))

    if not json_files:
        print(f"No JSON files found in {folder_path}")
        return

    out_folder = os.path.join(folder_path, 'out')
    os.makedirs(out_folder, exist_ok=True)
    output_txt = os.path.join(out_folder, 'out.txt')

    with open(output_txt, 'w') as txt_file:
        for json_file in json_files:
            frame_id = int(re.search(r'(\d+)\.json$', json_file).group(1))
            with open(json_file, 'r') as f:
                data = json.load(f)
                for obj in data:
                    if obj.get('obj_attr') == 'static':
                        continue

                    x = "{:.3f}".format(obj['psr']['position']['x'])
                    y = "{:.3f}".format(obj['psr']['position']['y'])
                    width = "{:.3f}".format(obj['psr']['scale']['x'])
                    height = "{:.3f}".format(obj['psr']['scale']['y'])
                    class_id = get_class_id(obj['obj_type'])
                    yaw = "{:.3f}".format(obj['psr']['rotation']['z'])

                    if float(width) < float(height):
                        height = "{:.3f}".format(obj['psr']['scale']['x'])
                        width = "{:.3f}".format(obj['psr']['scale']['y'])
                        yaw = str(float(yaw) + 1.57)

                    line = f"{frame_id},{obj['obj_id']},{x},{y},{width},{height},{1},{class_id},{yaw},-1\n"
                    txt_file.write(line)

    # ==== Part 2: MOTA Evaluation ====
    gt_file = f"./groundtruth/{dataset_name}.txt"
    prediction_file = output_txt
    temp_dir = "./tmp"
    os.makedirs(temp_dir, exist_ok=True)
    temp_gt_file = os.path.join(temp_dir, f"{dataset_name}_temp_gt.txt")
    temp_ts_file = os.path.join(temp_dir, f"{dataset_name}_temp_ts.txt")

    # PCD 路径
    pcd_path = f'../data/dynamic_pcd/{dataset_name}'

    if not os.path.exists(gt_file):
        print(f"Groundtruth file not found: {gt_file}")
        return

    filter_frames(gt_file, frame_start, frame_end, temp_gt_file)
    filter_frames_pre(prediction_file, frame_start, frame_end, temp_ts_file)

    gt = mm.io.loadtxt(temp_gt_file, fmt="mot15-2D")
    ts = mm.io.loadtxt(temp_ts_file, fmt="mot15-2D")

    print(f"\n[Start Evaluation for {dataset_name}] ===============================")
    acc = mm.utils.compare_to_groundtruth(gt, ts, pcd_path, 'iou', distth=0.4)

    mh = mm.metrics.create()
    summary = mh.compute_many([acc], metrics=mm.metrics.motchallenge_metrics, names=[dataset_name])

    strsummary = mm.io.render_summary(
        summary,
        formatters=mh.formatters,
        namemap=mm.io.motchallenge_metric_names
    )

    print("\n[Result] ===============================")
    print(strsummary)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate tracking results for a dataset.")
    parser.add_argument("dataset_name", type=str, help="Name of the dataset (e.g., Aeva-car)")

    args = parser.parse_args()
    run_evaluation(args.dataset_name)