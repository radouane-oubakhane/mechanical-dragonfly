"""Plot flight telemetry (requires optional matplotlib)."""
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
def plot(directory=ROOT/'telemetry',output=ROOT/'renders'):
    directory,output=Path(directory),Path(output);output.mkdir(parents=True,exist_ok=True)
    h=np.genfromtxt(directory/'flight_hover.csv',delimiter=',',names=True);f=np.genfromtxt(directory/'flight_forward.csv',delimiter=',',names=True)
    fig,ax=plt.subplots(figsize=(9,4.5));ax.plot(h['time_s'],h['z_m'],label='Hover');ax.plot(f['time_s'],f['z_m'],label='Forward flight',linestyle='--');ax.axhline(1,linestyle=':',label='Target altitude');ax.set(xlabel='Time (s)',ylabel='Altitude (m)',title='Step 28 | Altitude control');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(output/'flight_altitude.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,4.5));ax.plot(f['time_s'],f['vx_m_s'],label='Simulated forward velocity');ax.plot(f['time_s'],f['target_vx_m_s'],label='Commanded velocity',linestyle='--');ax.set(xlabel='Time (s)',ylabel='Velocity (m/s)',title='Step 28 | Forward flight response');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(output/'flight_velocity.png',dpi=160);plt.close(fig)
if __name__=='__main__':plot()
