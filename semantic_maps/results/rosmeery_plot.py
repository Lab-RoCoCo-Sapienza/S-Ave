
import argparse
import dataclasses
import json
import logging
import matplotlib
import os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


from typing import Any, Dict, List, Tuple


matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
# matplotlib.rcParams['text.usetex'] = True
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

Writer = animation.writers['ffmpeg']
writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)


key_to_plot_title = {
    "ori_surface": "ORI",
    "ori_confidence": "c-ORI",
    "ori_distance": "weighted-ORI",
    "opi": "OPI",
    "timestep": "timestep",
    "time_s": "time"
}

alg_name_to_legend_label = {
    "frontier_exploration": "frontier-exploration",
    "teleop": "user",
    "random_walk": "random",
    "ave": "ave",
    "s-ave": "s-ave"
}

# todo(rfr4n) remove this shit, it is due the fact that ori is not computed correctly in json
experiment_id_to_objects_number = {
    "rococo_lab": 21,
    "apartment_4": 35,
    "phd_office": 41,
    "prof_office": 25
}

# values
color_map = {
    "red": np.array([251, 180, 174])/255.0,
    "blue": np.array([179, 205, 227])/255.0,
    "green": np.array([204, 235, 197])/255.0,
    "purple": np.array([222, 203, 228])/255.0,
    "orange": np.array([254, 217, 166])/255.0,
    "yellow": np.array([255, 255, 204])/255.0,
    "brown": np.array([229, 216, 189])/255.0,
    "pink": np.array([253, 218, 236])/255.0,
    "white": np.array([255, 255, 255])/255.0,
    "pgray": np.array([240, 240, 240])/255.0,
    "gray": np.array([150, 150, 150])/255.0
}
# colors pick order
colors = ["blue", "red", "green", "purple", "orange",
          "yellow", "brown", "ping", "white", "gray", "black"]



@dataclasses.dataclass(frozen=True)
class ExperimentSpecs:
    experiment_id: str
    names: List[str]
    value: np.ndarray

def update_line(num, data, line):
    line.set_data(data[..., :num])
    return line,

def define_arguments() -> Any:
    parser = argparse.ArgumentParser(prog='motion_model_training',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--save-dir', type=str, default='/tmp/', help='Results save directory.')
    parser.add_argument('--json-load-path', nargs='+', default=[], help='List of selected files.')
    parser.add_argument('--metric-keys', nargs='+', default=[], help='List of metrics keys.')
    parser.add_argument('--debug', type=bool, default=False, help='Debug.')

    return parser.parse_args()


def array_form_lists(experiment_id: str, input_list: List[List[List[float]]]) -> np.ndarray:
    max_iteration = 10e10

    all_metric_data = []
    for i in input_list:
        for j in i:
            if len(j) < max_iteration:
                max_iteration = len(j)

    for i in range(len(input_list)):
        for j in range(len(input_list[i])):
            input_list[i][j] = input_list[i][j][:max_iteration]

        metric_data = np.array(input_list[i], dtype=np.float32)
        if experiment_id in experiment_id_to_objects_number:
            metric_data *= 1./experiment_id_to_objects_number[experiment_id]
        metric_data_avg = np.expand_dims(np.mean(metric_data, axis=0), axis=-1)
        metric_data_stdd = np.expand_dims(np.std(metric_data, axis=0), axis=-1)
        all_metric_data.append(np.concatenate([metric_data_avg, metric_data_stdd], axis=-1))

    return np.array(all_metric_data, dtype=np.float32)


def load_metrics(file_paths: List[str], metric_keys: List[str]) -> Dict[str, ExperimentSpecs]:
    metrics_dict = {}

    for m in metric_keys:
        metric_names_list = []
        metric_data_list = []

        experiment_id = None
        for p in file_paths:
            experiment_data = []
            experiment_file_paths = [os.path.join(p, f) for f in os.listdir(p)
                                     if os.path.isfile(os.path.join(p, f)) and f.endswith('.json')]

            if len(experiment_file_paths) == 0:
                continue

            for efp in experiment_file_paths:
                with open(efp) as json_file:
                    json_data = json.load(json_file)
                    assert m in json_data, f'Metric {m} not in json file with keys: {json_data.keys()}.'
                    experiment_data.append(json_data[m])

            experiment_id = p.split('/')[-3 if p.endswith('/') else -2]
            metric_names_list.append(p.split('/')[-2 if p.endswith('/') else -1])
            metric_data_list.append(experiment_data)

        metric_data = array_form_lists(experiment_id=experiment_id if 'ori' in m else 'none',
                                       input_list=metric_data_list)
        assert experiment_id is not None, f'Experiment id not found.'
        metrics_dict.update({m: ExperimentSpecs(experiment_id=experiment_id,
                                                names=metric_names_list, value=metric_data)})
        logging.info(f'Parsed metric {m} -- {metric_data.shape}')

    return metrics_dict


def plot(plot_handle: Any, metrics_data: ExperimentSpecs) -> None:
    for i, n in enumerate(metrics_data.names):
        metric_data_mean = metrics_data.value[i, ..., 0]
        metric_data_ub = metric_data_mean + metrics_data.value[i, ..., 1]
        metric_data_lb = metric_data_mean - metrics_data.value[i, ..., 1]
        metric_data_xticks = np.arange(metric_data_mean.shape[0])

        color = color_map[colors[i % len(color_map)]]
        plot_handle.fill_between(metric_data_xticks, metric_data_ub, metric_data_lb,
                                 alpha=0.3, edgecolor=0.7*color,
                                 interpolate=True,
                                 facecolor=np.array(color, ndmin=2))
        plot_handle.plot(metric_data_mean, label=alg_name_to_legend_label[n],
                         marker='^', markersize='8', markevery=10, color=color)


def configure_plot_info(plot_handle: Any, title: str, xtick_labels: List[str], x_label: str, y_label: str,
                        legend_pose: str, font_sz: int, legend_font_sz: int, ticks_font_sz: int) -> None:
    plot_handle.legend(loc=legend_pose, fontsize=legend_font_sz, ncol=1)
    plot_handle.xlabel(x_label, fontsize=font_sz)
    plot_handle.ylabel(y_label, fontsize=font_sz)
    plot_handle.yticks(fontsize=ticks_font_sz)
    plot_handle.xticks(fontsize=ticks_font_sz)
    plot_handle.title(title, fontsize=font_sz)
    plot_handle.tight_layout()

    axis_handle = plot_handle.gca()
    axis_handle.grid(color='gainsboro', linestyle='--', linewidth=0.1, alpha=0.5)
    axis_handle.set_axisbelow(True)
    axis_handle.ticklabel_format(axis='y', useOffset=False)
    axis_handle.set_xticks([x for x in range(len(xtick_labels))])
    axis_handle.set_xticklabels([r'' + str(int(t)) if i % 3 == 1 else '' for i,
                                 t in enumerate(xtick_labels)], fontsize=ticks_font_sz)


def plot_metrics(save_dir: str, metrics_data: Dict[str, ExperimentSpecs],
                 font_sz: int = 18, legend_font_sz: int = 20, ticks_font_sz: int = 16) -> None:
    xtick_labels = metrics_data['time_s'].value[0, :, 0].astype(dtype=np.int32)
    xtick_labels -= xtick_labels[0]
    xtick_labels = xtick_labels.astype(dtype=np.str)

    for metric_key, metric_results in metrics_data.items():
        if metric_key == 'time_s':
            continue

        figure = plt.figure(metric_key)
        l, = plot(plot_handle=plt, metrics_data=metric_results)
        configure_plot_info(plot_handle=plt, title=key_to_plot_title[metric_key], xtick_labels=xtick_labels,
                            x_label="time [s]", y_label="Score", legend_pose="upper left",
                            font_sz=18, legend_font_sz=16, ticks_font_sz=12)
        save_path = os.path.join(save_dir,
                                 f'{metrics_data[metric_key].experiment_id}_{metric_key}.pdf')
        # plt.show()
        plt.savefig(save_path, format='pdf')
        logging.info(f'Figure saved at: {save_path}')
        
        line_ani = animation.FuncAnimation(figure, update_line, 25, fargs=(metric_results, l),
                                   interval=50, blit=True)
        line_ani.save(save_path+'.mp4', writer=writer)


if __name__ == '__main__':
    args: argparse.Namespace = define_arguments()
    logging_level = logging.DEBUG if args.debug else logging.INFO
    logging.getLogger().setLevel(logging_level)

    metrics_data = load_metrics(file_paths=args.json_load_path, metric_keys=args.metric_keys)
    plot_metrics(save_dir=args.save_dir, metrics_data=metrics_data,
                 font_sz=18, legend_font_sz=20, ticks_font_sz=16)
