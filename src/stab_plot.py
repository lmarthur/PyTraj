import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import scienceplots

plt.style.use(['science'])

def stab_plot(stab_data):
    """
    Function to plot the stability data.

    INPUTS:
    ----------
        stab_data: pandas.DataFrame
            The stability data.
    """
    x = np.array([-1, 0, 1])

    # get the values into numpy arrays
    
    cep_pos = stab_data['cep'][0:3].values
    cep_vel = stab_data['cep'][3:6].values
    cep_ang = stab_data['cep'][6:9].values
    cep_acc = stab_data['cep'][9:12].values
    cep_gyrob = stab_data['cep'][12:15].values
    cep_gyron = stab_data['cep'][15:18].values
    cep_total = stab_data['cep'][18:21].values

    
    plt.figure(figsize=(5,5))
    ax = plt.gca()
    

    params = {
       'axes.labelsize': 8,
       'font.size': 8,
       'font.family': 'serif',
       'legend.fontsize': 8,
       'xtick.labelsize': 10,
       'ytick.labelsize': 10,
       'text.usetex': True,
       }

    plt.rcParams.update(params)
    # set color palette
    colors = plt.cm.viridis(np.linspace(0, 1, 7))

    plt.plot(x, cep_pos, label='Initial Position')
    plt.plot(x, cep_vel, label='Initial Velocity')
    plt.plot(x, cep_ang, label='Initial Angle')
    plt.plot(x, cep_acc, label='Acc Scale')
    plt.plot(x, cep_gyrob, label='Gyro Bias')
    plt.plot(x, cep_gyron, label='Gyro Noise')
    plt.plot(x, cep_total, label='Total')

    plt.yscale('log')
    # Add categorical xticks
    plt.xticks(x, ['$ E/10 $ ', 'E', '$10 E$'])

    plt.xlabel('Estimated Parameter (E)')
    plt.ylabel('Mean Miss Distance (m)')
    plt.title('Mean Miss Contribution From Error Parameters Without GNSS')

    # legend with shadow and no frame
    plt.legend()
    plt.tight_layout()
    plt.savefig('./output/sensitivity_plot_ICBM.pdf')
    plt.close()
