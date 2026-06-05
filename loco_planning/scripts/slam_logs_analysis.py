#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import glob
import numpy as np

import matplotlib
matplotlib.use("TkAgg", force=False)

import matplotlib.pyplot as plt


def get_arg_value(name, default_value):
    private_name = "_" + name + ":="
    normal_name = name + "="

    for arg in sys.argv:
        if arg.startswith(private_name):
            return arg.split(":=", 1)[1]

        if arg.startswith(normal_name):
            return arg.split("=", 1)[1]

    return default_value


def str_to_bool(value):
    if isinstance(value, bool):
        return value

    value = str(value).lower()

    if value in ["true", "1", "yes", "y"]:
        return True

    return False


def find_latest_run(log_dir):
    runs = sorted(
        glob.glob(os.path.join(log_dir, "run_*")),
        reverse=True
    )

    if len(runs) == 0:
        return None

    return runs[0]


def load_csv(path):
    if not os.path.isfile(path):
        return None

    data = np.genfromtxt(
        path,
        delimiter=",",
        names=True,
        dtype=None,
        encoding=None
    )

    if data.size == 0:
        return None

    if data.shape == ():
        data = np.array([data], dtype=data.dtype)

    return data


def get_column(data, name):
    if data is None:
        return None

    if name not in data.dtype.names:
        return None

    return data[name]


def save_or_close_figure(path, save_plots, show_plots):
    if save_plots:
        plt.savefig(path, dpi=150, bbox_inches="tight")

    if not show_plots:
        plt.close()


def plot_robot_trajectory(robot, landmark, run_dir, save_plots, show_plots):
    x = get_column(robot, "x_ukf")
    y = get_column(robot, "y_ukf")

    if x is None or y is None:
        return

    plt.figure()
    plt.plot(x, y, label="UKF robot trajectory")

    if landmark is not None:
        landmark_ids = sorted(set(get_column(landmark, "landmark_id")))

        for landmark_id in landmark_ids:
            mask = get_column(landmark, "landmark_id") == landmark_id

            lm_x = get_column(landmark, "x")[mask]
            lm_y = get_column(landmark, "y")[mask]

            if lm_x.shape[0] > 0:
                plt.plot(
                    lm_x[-1],
                    lm_y[-1],
                    marker="o",
                    linestyle="None",
                    label="landmark %d final" % int(landmark_id)
                )

    plt.xlabel("x map [m]")
    plt.ylabel("y map [m]")
    plt.title("UKF robot trajectory in map frame")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    path = os.path.join(run_dir, "robot_trajectory.png")
    save_or_close_figure(path, save_plots, show_plots)


def plot_landmark_positions_over_time(landmark, run_dir, save_plots, show_plots):
    if landmark is None:
        return

    landmark_ids = sorted(set(get_column(landmark, "landmark_id")))

    plt.figure()

    for landmark_id in landmark_ids:
        mask = get_column(landmark, "landmark_id") == landmark_id

        x = get_column(landmark, "x")[mask]
        y = get_column(landmark, "y")[mask]

        plt.plot(
            x,
            y,
            marker=".",
            linestyle="None",
            label="landmark %d" % int(landmark_id)
        )

    plt.xlabel("x map [m]")
    plt.ylabel("y map [m]")
    plt.title("Landmark estimated positions over time")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    path = os.path.join(run_dir, "landmark_positions_over_time.png")
    save_or_close_figure(path, save_plots, show_plots)


def plot_landmark_covariance_components(landmark, run_dir, save_plots, show_plots):
    if landmark is None:
        return

    if "Pxx" not in landmark.dtype.names:
        return

    if "Pyy" not in landmark.dtype.names:
        return

    landmark_ids = sorted(set(get_column(landmark, "landmark_id")))

    plt.figure()

    for landmark_id in landmark_ids:
        mask = get_column(landmark, "landmark_id") == landmark_id

        t = get_column(landmark, "t")[mask]
        Pxx = get_column(landmark, "Pxx")[mask]
        Pyy = get_column(landmark, "Pyy")[mask]

        plt.plot(
            t,
            Pxx,
            linestyle="-",
            label="landmark %d Pxx" % int(landmark_id)
        )

        plt.plot(
            t,
            Pyy,
            linestyle="--",
            label="landmark %d Pyy" % int(landmark_id)
        )

    plt.xlabel("time [s]")
    plt.ylabel("variance [m^2]")
    plt.title("Landmark covariance components over time")
    plt.grid(True)
    plt.legend()

    path = os.path.join(run_dir, "landmark_covariance_components.png")
    save_or_close_figure(path, save_plots, show_plots)


def plot_landmark_relative_covariance_components(landmark, run_dir, save_plots, show_plots):
    if landmark is None:
        return

    if "Pxx_relative" not in landmark.dtype.names:
        return

    if "Pyy_relative" not in landmark.dtype.names:
        return

    landmark_ids = sorted(set(get_column(landmark, "landmark_id")))

    plt.figure()

    for landmark_id in landmark_ids:
        mask = get_column(landmark, "landmark_id") == landmark_id

        t = get_column(landmark, "t")[mask]
        Pxx_relative = get_column(landmark, "Pxx_relative")[mask]
        Pyy_relative = get_column(landmark, "Pyy_relative")[mask]

        plt.plot(
            t,
            Pxx_relative,
            linestyle="-",
            label="landmark %d Pxx rel" % int(landmark_id)
        )

        plt.plot(
            t,
            Pyy_relative,
            linestyle="--",
            label="landmark %d Pyy rel" % int(landmark_id)
        )

    plt.xlabel("time [s]")
    plt.ylabel("relative variance [m^2]")
    plt.title("Relative robot-landmark covariance components over time")
    plt.grid(True)
    plt.legend()

    path = os.path.join(run_dir, "landmark_relative_covariance_components.png")
    save_or_close_figure(path, save_plots, show_plots)


def write_summary(robot, landmark, run_dir):
    path = os.path.join(run_dir, "analysis_summary.txt")

    with open(path, "w") as f:
        f.write("SLAM RELATIVE LOCALIZATION ANALYSIS\n")
        f.write("===================================\n\n")

        if robot is not None:
            f.write("Robot samples: %d\n\n" % robot.shape[0])

            x = get_column(robot, "x_ukf")
            y = get_column(robot, "y_ukf")
            theta = get_column(robot, "theta_ukf")
            n_landmarks = get_column(robot, "n_landmarks")

            if x is not None and y is not None and theta is not None:
                f.write("Final robot UKF state:\n")
                f.write("  x     = %.6f m\n" % x[-1])
                f.write("  y     = %.6f m\n" % y[-1])
                f.write("  theta = %.6f rad\n\n" % theta[-1])

                f.write("UKF trajectory extents:\n")
                f.write("  x range = [%.3f, %.3f] m, width  %.3f m\n" %
                        (np.min(x), np.max(x), np.max(x) - np.min(x)))
                f.write("  y range = [%.3f, %.3f] m, height %.3f m\n\n" %
                        (np.min(y), np.max(y), np.max(y) - np.min(y)))

            if n_landmarks is not None:
                f.write("Final number of landmarks: %d\n\n" % int(n_landmarks[-1]))

        if landmark is not None:
            f.write("Landmark samples: %d\n\n" % landmark.shape[0])

            landmark_ids = sorted(set(get_column(landmark, "landmark_id")))

            f.write("Final landmarks:\n")

            for landmark_id in landmark_ids:
                mask = get_column(landmark, "landmark_id") == landmark_id

                lm = landmark[mask]
                last = lm[-1]

                f.write(
                    "  id=%d x=%.6f y=%.6f n_obs=%d Pxx=%.9f Pyy=%.9f" %
                    (
                        int(last["landmark_id"]),
                        float(last["x"]),
                        float(last["y"]),
                        int(last["n_observations"]),
                        float(last["Pxx"]),
                        float(last["Pyy"])
                    )
                )

                if "Pxx_relative" in landmark.dtype.names and "Pyy_relative" in landmark.dtype.names:
                    f.write(
                        " Pxx_relative=%.9f Pyy_relative=%.9f" %
                        (
                            float(last["Pxx_relative"]),
                            float(last["Pyy_relative"])
                        )
                    )

                f.write("\n")

            f.write("\n")

        f.write("Generated plots:\n")
        f.write("  robot_trajectory.png\n")
        f.write("  landmark_positions_over_time.png\n")
        f.write("  landmark_covariance_components.png\n")
        f.write("  landmark_relative_covariance_components.png\n")


def main():
    log_dir = get_arg_value(
        "log_dir",
        "/root/slam_logs"
    )

    run_dir = get_arg_value(
        "run_dir",
        ""
    )

    save_plots = str_to_bool(
        get_arg_value("save_plots", "true")
    )

    show_plots = str_to_bool(
        get_arg_value("show_plots", "true")
    )

    if run_dir == "":
        run_dir = find_latest_run(log_dir)

    if run_dir is None:
        print("No run directory found.")
        return

    robot_path = os.path.join(run_dir, "robot_history.csv")
    landmark_path = os.path.join(run_dir, "landmark_history.csv")

    robot = load_csv(robot_path)
    landmark = load_csv(landmark_path)

    if robot is None:
        print("Missing or empty robot_history.csv")
        return

    plot_robot_trajectory(
        robot,
        landmark,
        run_dir,
        save_plots,
        show_plots
    )

    plot_landmark_positions_over_time(
        landmark,
        run_dir,
        save_plots,
        show_plots
    )

    plot_landmark_covariance_components(
        landmark,
        run_dir,
        save_plots,
        show_plots
    )

    plot_landmark_relative_covariance_components(
        landmark,
        run_dir,
        save_plots,
        show_plots
    )

    write_summary(
        robot,
        landmark,
        run_dir
    )

    print("Analysis completed.")
    print("Run directory:")
    print(run_dir)

    if show_plots:
        plt.show()


if __name__ == "__main__":
    main()