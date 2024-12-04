import rosbag2_py
import pandas as pd
import matplotlib.pyplot as plt
import struct

def read_rosbag2(bag_path):
    # Setup for reading the ROSbag
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    data1 = []
    data2 = []

    # Loop through messages in the ROSbag
    while reader.has_next():
        topic, serialized_msg, timestamp = reader.read_next()
        
        if topic == '/fused_position':
            # Unpack serialized binary data for a Point message (assuming float64 for x, y, z)
            x, y, z = struct.unpack('ddd', serialized_msg[4:28])
            
            # Convert timestamp from nanoseconds to seconds
            time_in_sec = timestamp / 1e9
            data1.append((time_in_sec, x, z))
            
        elif topic == '/reflict_center':
            # Unpack serialized binary data for a Point message (assuming float64 for x, y, z)
            x, y, z = struct.unpack('ddd', serialized_msg[4:28])
            
            # Convert timestamp from nanoseconds to seconds
            time_in_sec = timestamp / 1e9
            data2.append((time_in_sec, x, z))

    df1 = pd.DataFrame(data1, columns=['time', 'x', 'z'])
    df2 = pd.DataFrame(data2, columns=['time', 'x', 'z'])
    return df1, df2

def plot_data(df1, df2):
    plt.figure(figsize=(12, 12))
    
    # Convert to NumPy arrays before plotting
    x1, z1 = df1['x'].to_numpy(), df1['z'].to_numpy()
    x2, z2 = df2['x'].to_numpy(), df2['z'].to_numpy()
    
    plt.scatter(x1, z1, marker='o', color='blue', label='Fused Position')  # Plot z against x for df1
    plt.scatter(x2, z2, marker='x', color='orange', label='Reflect Center')  # Plot z against x for df2
    plt.title('Plot of Z vs X from ROSbag2')
    plt.xlabel('X')
    plt.ylabel('Z')
    plt.grid()
    plt.axis('equal')
    plt.legend()
    
    plt.show()

if __name__ == '__main__':
    bag_path = '/home/gxh/Desktop/bags/rosbag2_2024_11_08-22_30_55'
    df1, df2 = read_rosbag2(bag_path)
    plot_data(df1, df2)
