import pandas as pd
import matplotlib.pyplot as plt


class AutotunePlot:

    def __init__(self, ref_name, target_name):
        self.plot(ref_name, target_name)

    def plot(self, ref_name, target_name):
        data_target = pd.read_csv(target_name)
        data_ref = pd.read_csv(ref_name)

        dx_ref = data_ref['d_x'].to_numpy()
        dy_ref = data_ref['d_y'].to_numpy()

        dx_tar = data_target['d_x'].to_numpy()
        dy_tar = data_target['d_y'].to_numpy()

        h2_norm = data_target['h2_norm'].to_numpy()
        h2_norm_dx = data_target['h2_norm_x'].to_numpy()
        h2_norm_dy = data_target['h2_norm_y'].to_numpy()
        time_stamp = data_target['timestamp'].to_numpy()

        plt.figure(1)
        plt.plot(time_stamp, h2_norm_dx)
        plt.xlabel('time, [s]')
        plt.ylabel('h2 norm x')
        plt.title('h2 norm x')
        plt.figure(2)
        plt.plot(time_stamp, h2_norm_dy)
        plt.xlabel('time, [s]')
        plt.ylabel('h2 norm y')
        plt.title('h2 norm y')
        plt.figure(3)
        plt.plot(time_stamp, h2_norm)
        plt.xlabel('time, [s]')
        plt.ylabel('h2 norm')
        plt.title('h2 norm')

        plt.figure(4)
        plt.plot(time_stamp, dx_ref)
        plt.plot(time_stamp, dx_tar)
        plt.xlabel('time, [s]')
        plt.ylabel('dx, [m]')
        plt.legend(['dx ref', 'dx target'])
        plt.title('dx')
        plt.figure(5)
        plt.plot(time_stamp, dy_ref)
        plt.plot(time_stamp, dy_tar)
        plt.xlabel('time, [s]')
        plt.ylabel('dy, [m]')
        plt.legend(['dy ref', 'dy target'])
        plt.show()

