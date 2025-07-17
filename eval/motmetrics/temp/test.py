import motmetrics as mm
import glob
import os
import pandas as pd

# gt_file = './motmetrics/data/TUD-Campus/gt.txt'
# dt_file = './motmetrics/data/TUD-Campus/test.txt'
# summary = mm.simpeval(gt_file, dt_file)

## list is ok 
## ['FrameId', 'Id', 'X', 'Y', 'Width', 'Height', 'Confidence', 'ClassId', 'Visibility', 'unused']



gt_file="/home/zengyb/dataset/fmcw_data/data/pcd_car2/label/out/out.txt"
"""  文件格式如下
1,0,1255,50,71,119,1,1,1
2,0,1254,51,71,119,1,1,1
3,0,1253,52,71,119,1,1,1
...
"""

# ts1_file="/home/zengyb/project/SimpleTrack/mot_results/demo/summary/vehicle/json/out"
ts1_file="/home/zengyb/project/MOT_fmcw_track/search_tracking/build/release/bin/label/out"
# ts1_file="/home/zengyb/dataset/experiment/track_test/aeva_car"
# ts1_file="/home/zengyb/dataset/fmcw_data/data/pcd_car2/label/out"



# 设定要筛选的帧号范围
i, j = 350, 351# 可以根据需要调整这里的值


print("范围在：",i,"到",j,"之间")

# 定义两个固定的临时文件路径
temp_gt_file = "./tmp/temp_gt.txt"
temp_ts_file = "./tmp/temp_ts.txt"

# 定义一个函数来读取并筛选文件，然后写入到固定的临时文件
def filter_frames(file_path, frame_start, frame_end, output_path):
    # 读取并筛选文件
    data = pd.read_csv(file_path, header=None)
    filtered_data = data[(data[0] >= frame_start) & (data[0] <= frame_end)]
    # print(filtered_data.shape)
    # 将筛选后的数据保存到指定路径
    filtered_data.to_csv(output_path, header=False, index=False, sep=',')

txt_files = glob.glob(os.path.join(ts1_file, "*.txt"))

for file_path in txt_files:
    print("找到文件：", file_path)

    # 筛选并写入到临时文件
    filter_frames(gt_file, i, j, temp_gt_file)
    filter_frames(file_path, i, j, temp_ts_file)

    # 使用临时文件路径读取数据
    gt = mm.io.loadtxt(temp_gt_file, fmt="mot15-2D")
    ts = mm.io.loadtxt(temp_ts_file, fmt="mot15-2D")


    # gt = mm.io.loadtxt(gt_file, fmt="mot15-2D")
    # ts = mm.io.loadtxt(file_path, fmt="mot15-2D")


    # 筛选X坐标大于50的GT数据
    gt_50 = gt[gt['X'] > 50]

    # 筛选X坐标大于50的跟踪结果
    ts_50 = ts[ts['X'] > 50]

    acc=mm.utils.compare_to_groundtruth(gt, ts, 'iou',distth=0.9)  # 根据GT和自己的结果，生成accumulator，distth是距离阈值
    # print(acc)
    acc_50=mm.utils.compare_to_groundtruth(gt_50, ts_50, 'iou',distth=0.9)  # 根据GT和自己的结果，生成accumulator，distth是距离阈值

    mh = mm.metrics.create()

    # 打印单个accumulator
    summary = mh.compute_many([acc,acc_50],
                            metrics=mm.metrics.motchallenge_metrics,
                            names=['full','x_50'])

    strsummary = mm.io.render_summary(
        summary,
        formatters=mh.formatters,
        namemap=mm.io.motchallenge_metric_names
    )
    print(strsummary)
