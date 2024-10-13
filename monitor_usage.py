import psutil
import time
import matplotlib.pyplot as plt
import numpy as np

# Constants
DURATION = 30  # seconds
INTERVAL = 1  # second

# Initialize data structures
cpu_count = psutil.cpu_count()
cpu_data = [[] for _ in range(cpu_count)]
ram_data = []
time_data = []

def collect_data():
    cpu_percents = psutil.cpu_percent(percpu=True)
    for i, cpu_percent in enumerate(cpu_percents):
        cpu_data[i].append(cpu_percent)
    
    ram_percent = psutil.virtual_memory().percent
    ram_data.append(ram_percent)
    
    current_time = len(time_data)
    time_data.append(current_time)

def calculate_averages():
    avg_ram = np.mean(ram_data)
    avg_cpu = np.mean([np.mean(cpu) for cpu in cpu_data])
    return avg_cpu, avg_ram

def plot_data():
    avg_cpu, avg_ram = calculate_averages()
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Plot CPU usage
    for i, cpu in enumerate(cpu_data):
        ax1.plot(time_data, cpu, label=f'CPU {i+1}')
    ax1.set_title(f'CPU Usage Over Time [Avg: {avg_cpu:.2f}%]')
    ax1.set_xlabel('Time (seconds)')
    ax1.set_ylabel('Usage (%)')
    ax1.set_xlim(0, DURATION)
    ax1.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=6)
    
    # Plot RAM usage
    ax2.plot(time_data, ram_data, label='RAM')
    ax2.set_title(f'RAM Usage Over Time [Avg: {avg_ram:.2f}%]')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Usage (%)')
    ax2.set_xlim(0, DURATION)
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig('usage_graph.png', bbox_inches='tight')
    print("Graph saved as 'usage_graph.png'")

print(f"Collecting data for {DURATION} seconds...")

start_time = time.time()
for _ in range(DURATION):
    collect_data()
    time.sleep(INTERVAL)

plot_data()
print("Data collection complete. Graph generated.")