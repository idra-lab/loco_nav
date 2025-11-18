
import matplotlib.pyplot as plt
import numpy as np

lw_des=7
lw_act=4
marker_size= 0

def plotFrameLinear(name, time_log, des_Pose_log=None, Pose_log=None, des_Twist_log=None, Twist_log=None,
               title=None, frame=None, sharex=True, sharey=False, start=0, end=-1, wrapp_labels=None, custom_labels=None):
    plot_var_log = None
    plot_var_des_log = None
    labels = ["", "", ""]
    if name == 'position':
        labels = ["x", "y", "z"]
        lin_unit = '[m]'
        if Pose_log is not None:
            if Pose_log.shape[0] == 6:
                plot_var_log = u.linPart(Pose_log)
            elif Pose_log.shape[0] == 3:
                plot_var_log = Pose_log
        if (des_Pose_log is not None):
            if des_Pose_log.shape[0] == 6:
                plot_var_des_log = u.linPart(des_Pose_log)
            elif des_Pose_log.shape[0] == 3:
                plot_var_des_log = des_Pose_log

    elif name == 'velocity':
        labels = ["x", "y", "z"]
        lin_unit = '[m/s]'
        if Twist_log is not None:
            if Twist_log.shape[0] == 6:
                plot_var_log = u.linPart(Twist_log)
            elif Twist_log.shape[0] == 3:
                plot_var_log = Twist_log
        if (des_Twist_log is not None):
            if des_Twist_log.shape[0] == 6:
                plot_var_des_log = u.linPart(des_Twist_log)
            elif des_Twist_log.shape[0] == 3:
                plot_var_des_log = des_Twist_log

    else:
       print("wrong choice")

    if custom_labels is not None:
        labels = custom_labels

    if title is None:
        title = name
    else:
        title = title + ' ' + name
    if frame is not None:
        title+= ' ' + frame

    dt = np.round(time_log[1] - time_log[0], 3)
    if type(start) == str:
        start = max(0, int(float(start) / dt + 1))
    if type(end) == str:
        end = min(int(float(end) / dt + 1), time_log.shape[0])

    if len(plt.get_fignums()) == 0:
        figure_id = 1
    else:
        figure_id = max(plt.get_fignums()) + 1
    fig = plt.figure(figure_id)
    fig.suptitle(title, fontsize=20)
    ax = subplot(3, 1, 1, sharex=False, sharey=False, ax_to_share=None)
    plt.ylabel(labels[0] + " "+lin_unit)
    if (plot_var_des_log is not None):
        plt.plot(time_log[start:end], plot_var_des_log[0, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[0, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='blue')
    plt.grid()

    subplot(3, 1, 2, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[1] + " "+lin_unit)
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[1, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[1, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()

    subplot(3, 1, 3, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[2] + " "+lin_unit)
    plt.xlabel("Time [s]")
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[2, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[2, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()

    fig.align_ylabels(fig.axes[:3])

    return fig

#plot functions
def subplot(n_rows, n_cols, n_subplot, sharex=False, sharey=False, ax_to_share=None):
    if sharex and sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot, sharex=ax_to_share, sharey=ax_to_share)
    if sharex and not sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot, sharex=ax_to_share)
    if not sharex and sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot, sharey=ax_to_share)
    if not sharex and not sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot)
    return ax