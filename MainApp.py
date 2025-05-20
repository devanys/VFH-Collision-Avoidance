import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
# Parameter dasar
wheel_radius = 0.1
robot_length = 0.5
dt = 0.1
tolerance = 0.2
obstacle_radius = 0.5
v = 2
obstacle_speed = 0.01

# State awal
x, y, theta = 0.0, 0.0, 0.0
target = np.array([10.0, 10.0])
obstacles = np.array([])
obstacle_directions = np.array([])

# Parameter sensor
num_beams = 40
sensor_range = 8.0
field_of_view = np.pi / 3 
avoidance_threshold = 1.5
smoothing_factor = 0.8
sector_collision_threshold = 0.3  

def pure_pursuit(target, x, y, theta):
    dx = target[0] - x
    dy = target[1] - y
    alpha = np.arctan2(dy, dx) - theta
    return alpha

def vector_field_histogram(obstacles, x, y, theta):
    angles = np.linspace(-field_of_view, field_of_view, num_beams) + theta
    distances = np.full_like(angles, sensor_range)
    
    for obstacle in obstacles:
        dist = np.hypot(obstacle[0] - x, obstacle[1] - y) - obstacle_radius
        angle_to_obstacle = np.arctan2(obstacle[1] - y, obstacle[0] - x) - theta
        angle_to_obstacle = np.arctan2(np.sin(angle_to_obstacle), np.cos(angle_to_obstacle))
        
        if -field_of_view <= angle_to_obstacle <= field_of_view and dist < sensor_range:
            beam_idx = np.argmin(np.abs(angles - angle_to_obstacle))
            distances[beam_idx] = min(distances[beam_idx], dist)
    
    sector_labels = ['LC', 'CL', 'C', 'CR', 'RC']
    sector_ranges = [(-field_of_view, -0.75*field_of_view),
                    (-0.75*field_of_view, -0.25*field_of_view),
                    (-0.25*field_of_view, 0.25*field_of_view),
                    (0.25*field_of_view, 0.75*field_of_view),
                    (0.75*field_of_view, field_of_view)]
    
    sector_distances = []
    sector_angles = []
    sector_collision_flags = []  
    
    for start, end in sector_ranges:
        mask = (angles >= start+theta) & (angles <= end+theta)
        sector_dist = np.min(distances[mask]) if np.any(mask) else sensor_range
        sector_distances.append(sector_dist)
        sector_angles.append(np.mean([start, end]) + theta)
        
        sector_angle = np.mean([start, end]) + theta
        sector_line_collision = False
        
        for obstacle in obstacles:
            obstacle_vector = np.array([obstacle[0] - x, obstacle[1] - y])
            sector_vector = np.array([np.cos(sector_angle), np.sin(sector_angle)])
            
            
            projection = np.dot(obstacle_vector, sector_vector)
            
            if projection > 0:  
                perpendicular_distance = np.linalg.norm(obstacle_vector - projection * sector_vector)
                
                if perpendicular_distance < sector_collision_threshold + obstacle_radius and projection < sensor_range:
                    sector_line_collision = True
                    break
        
        sector_collision_flags.append(sector_line_collision)
    
    min_dist = np.min(distances)
    
    # Prioritaskan menghindar jika ada garis sektor yang menabrak obstacle
    immediate_avoidance = None
    for i, (collision, angle) in enumerate(zip(sector_collision_flags, sector_angles)):
        if collision:
            immediate_avoidance = angle + np.pi/2 if i < 2 else angle - np.pi/2  # Belok berlawanan arah
            break
    
    if immediate_avoidance is not None:
        avoidance_angle = immediate_avoidance
    elif min_dist < avoidance_threshold:
        left_clearance = np.mean(sector_distances[:2])  
        right_clearance = np.mean(sector_distances[3:]) 
        
        if left_clearance < right_clearance:
            avoidance_angle = sector_angles[np.argmax(sector_distances[3:])+3]  # Pilih sektor kanan yang paling aman
        else:
            avoidance_angle = sector_angles[np.argmax(sector_distances[:2])]  # Pilih sektor kiri yang paling aman
    else:
        avoidance_angle = None
    
    return avoidance_angle, min_dist, distances, sector_labels, sector_distances, sector_angles, sector_collision_flags

def update_obstacles(obstacles, directions):
    directions += np.random.randn(*directions.shape) * 0.1
    directions = directions / np.linalg.norm(directions, axis=1, keepdims=True)
    obstacles += directions * obstacle_speed
    obstacles = np.clip(obstacles, 1 + obstacle_radius, 9 - obstacle_radius)
    return obstacles, directions

def get_direction_label(norm_angle):
    if norm_angle < -0.75:
        return "LC"  
    elif norm_angle < -0.25:
        return "CL"
    elif norm_angle <= 0.25:
        return "C"   
    elif norm_angle <= 0.75:
        return "CR"  
    else:
        return "RC"  

# Fungsi untuk input obstacle statis
def input_obstacle_static(num_obstacles):
    obstacles = []
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_title(f"Klik pada posisi untuk {num_obstacles} obstacle (Max 3)")
    ax.set_aspect('equal')
    ax.grid(True)
    
    plt.plot(target[0], target[1], 'bo', markersize=10, label="Target")
    
    def onclick(event):
        if event.xdata is not None and event.ydata is not None:
            if len(obstacles) < num_obstacles:
                obstacles.append([event.xdata, event.ydata])
                circle = plt.Circle((event.xdata, event.ydata), obstacle_radius, color='purple', fill=True)
                ax.add_patch(circle)
                plt.draw()
                
                if len(obstacles) == num_obstacles:
                    plt.close()
    
    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()
    return np.array(obstacles)

def show_scenario_menu():
    fig, ax = plt.subplots(figsize=(8, 4))
    plt.subplots_adjust(bottom=0.2)
    ax.axis('off')
    ax.set_title("Pilih Skenario Simulasi", pad=20)
    
    ax_static = plt.axes([0.3, 0.5, 0.4, 0.2])
    btn_static = Button(ax_static, 'Skenario Statis')
    
    ax_dynamic = plt.axes([0.3, 0.2, 0.4, 0.2])
    btn_dynamic = Button(ax_dynamic, 'Skenario Dinamis')
    
    scenario_choice = {'choice': None}
    
    def static_selected(event):
        scenario_choice['choice'] = 'static'
        plt.close()
    
    def dynamic_selected(event):
        scenario_choice['choice'] = 'dynamic'
        plt.close()
    
    btn_static.on_clicked(static_selected)
    btn_dynamic.on_clicked(dynamic_selected)
    
    plt.show()
    return scenario_choice['choice']

# Main program
if __name__ == "__main__":
    scenario = show_scenario_menu()
    
    if scenario == 'static':
        num_obstacles = 0
        while num_obstacles < 1 or num_obstacles > 3:
            try:
                num_obstacles = int(input("Masukkan jumlah obstacle (1-3): "))
            except ValueError:
                print("Harap masukkan angka antara 1-3")
        
        obstacles = input_obstacle_static(num_obstacles)
        obstacle_directions = np.zeros_like(obstacles)
        
    elif scenario == 'dynamic':
        obstacles = np.array([[5.0, 5.0]])
        obstacle_directions = np.random.rand(1, 2) * 2 - 1
    else:
        exit()
    
    plt.figure(figsize=(14, 8))
    plt.subplots_adjust(right=0.7)
    
    # Simulasi utama
    for i in range(500):
        alpha = pure_pursuit(target, x, y, theta)
        avoidance_angle, min_dist, distances, sector_labels, sector_distances, sector_angles, sector_collisions = vector_field_histogram(obstacles, x, y, theta)
        
        if avoidance_angle is not None:
            theta = (smoothing_factor * theta + (1 - smoothing_factor) * avoidance_angle)
        else:
            theta += alpha * dt * 0.5  
        
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        
        if np.hypot(target[0] - x, target[1] - y) < tolerance:
            print("Target Finis!")
            break
        
        if scenario == 'dynamic':
            obstacles, obstacle_directions = update_obstacles(obstacles, obstacle_directions)
        
        plt.clf()
        
        main_ax = plt.axes([0.1, 0.1, 0.6, 0.8])
        
        main_ax.plot(x, y, 'go', markersize=10, label="Robot")
        main_ax.plot(target[0], target[1], 'bo', markersize=10, label="Target")
        
        for idx, obstacle in enumerate(obstacles):
            circle = plt.Circle(obstacle, obstacle_radius, color='purple', fill=True)
            main_ax.add_patch(circle)
            if scenario == 'dynamic':
                label_x = get_direction_label(obstacle_directions[idx][0])
                label_y = get_direction_label(obstacle_directions[idx][1])
                main_ax.text(obstacle[0], obstacle[1] + 0.6, f"{label_x}-{label_y}", 
                            fontsize=8, ha='center', va='center')
        
        main_ax.arrow(x, y, 0.5 * np.cos(theta), 0.5 * np.sin(theta), 
                     head_width=0.2, head_length=0.3, fc='green', ec='green')
        
        angles = np.linspace(-field_of_view, field_of_view, num_beams) + theta
        for angle in angles:
            end_x = x + sensor_range * np.cos(angle)
            end_y = y + sensor_range * np.sin(angle)
            main_ax.plot([x, end_x], [y, end_y], 'r--', alpha=0.3, linewidth=0.5)
        
        for angle, collision in zip(sector_angles, sector_collisions):
            end_x = x + 2 * np.cos(angle)
            end_y = y + 2 * np.sin(angle)
            color = 'm-' if collision else 'b-' 
            main_ax.plot([x, end_x], [y, end_y], color, alpha=0.8, linewidth=2)
        
        main_ax.set_xlim(-2, 12)
        main_ax.set_ylim(-2, 12)
        main_ax.set_aspect('equal', adjustable='box')
        main_ax.set_title("Simulasi VFH dengan Sektor Collision Detection")
        main_ax.legend(loc='upper left')
        
        # Panel
        info_ax = plt.axes([0.72, 0.1, 0.25, 0.8])
        info_ax.axis('off')
        
        info_y = 0.95
        info_ax.text(0.1, info_y, "Robot Info:", fontsize=12, weight='bold')
        info_ax.text(0.1, info_y-0.05, f"Posisi: ({x:.2f}, {y:.2f})", fontsize=10)
        info_ax.text(0.1, info_y-0.10, f"Orientasi: {theta:.2f} rad", fontsize=10)
        info_ax.text(0.1, info_y-0.15, f"Kecepatan: {v:.2f} m/s", fontsize=10)
        
        info_ax.text(0.1, info_y-0.25, "Target Info:", fontsize=12, weight='bold')
        info_ax.text(0.1, info_y-0.30, f"Posisi: ({target[0]}, {target[1]})", fontsize=10)
        info_ax.text(0.1, info_y-0.35, f"Jarak: {np.hypot(target[0]-x, target[1]-y):.2f}m", fontsize=10)
        
        info_ax.text(0.1, info_y-0.45, "Obstacle Info:", fontsize=12, weight='bold')
        info_ax.text(0.1, info_y-0.50, f"Jarak Min: {min_dist:.2f}m", fontsize=10)
        
        info_ax.text(0.1, info_y-0.60, "Sektor Collision:", fontsize=12, weight='bold')
        for i, (label, collision) in enumerate(zip(sector_labels, sector_collisions)):
            info_ax.text(0.1, info_y-0.65-(i*0.05), f"{label}: {'HIT' if collision else 'clear'}", 
                        color='red' if collision else 'black', fontsize=10)
        
        # Polar histogram visual
        polar_ax = plt.axes([0.72, 0.1, 0.25, 0.25])
        polar_angles = np.linspace(-field_of_view, field_of_view, num_beams)
        bars = polar_ax.bar(polar_angles, distances, width=0.1, color='red', alpha=0.6)
        
        # Garis pembatas sektor
        for angle in np.linspace(-field_of_view, field_of_view, 6):
            polar_ax.axvline(angle, color='blue', linestyle='--', alpha=0.5, linewidth=0.5)
        
        polar_ax.set_xticks(np.linspace(-field_of_view, field_of_view, 5))
        polar_ax.set_xticklabels(sector_labels, fontsize=8)
        polar_ax.set_title("Polar Histogram", fontsize=10)
        polar_ax.set_ylabel("Jarak (m)", fontsize=8)
        polar_ax.grid(True)
        
        plt.pause(0.01)

    plt.show()