import numpy as np
import matplotlib.pyplot as plt

wheel_radius = 0.1
robot_length = 0.5
dt = 0.1
tolerance = 0.2
obstacle_radius = 0.5
v = 1.0
obstacle_speed = 0.01
x, y, theta = 0.0, 0.0, 0.0
target = np.array([10.0, 10.0])
num_obstacles = 5
obstacles = np.random.rand(num_obstacles, 2) * 8 + 1
obstacle_directions = np.random.rand(num_obstacles, 2) * 2 - 1
num_beams = 40
sensor_range = 8.0
field_of_view = np.pi / 3
avoidance_threshold = 1.5
smoothing_factor = 0.8

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
    
    safe_beam_idx = np.argmax(distances)
    safe_angle = angles[safe_beam_idx]
    
    return safe_angle, np.min(distances), distances

def update_obstacles(obstacles, directions):
    directions += np.random.randn(*directions.shape) * 0.1
    directions = directions / np.linalg.norm(directions, axis=1, keepdims=True)
    obstacles += directions * obstacle_speed
    obstacles = np.clip(obstacles, 1 + obstacle_radius, 9 - obstacle_radius)
    return obstacles, directions

for i in range(500):
    alpha = pure_pursuit(target, x, y, theta)
    avoidance_angle, min_dist, distances = vector_field_histogram(obstacles, x, y, theta)
    
    if min_dist < avoidance_threshold:
        theta = (smoothing_factor * theta + (1 - smoothing_factor) * avoidance_angle)
    else:
        theta += alpha * dt

    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt

    if np.hypot(target[0] - x, target[1] - y) < tolerance:
        print("Target Finish!")
        break

    obstacles, obstacle_directions = update_obstacles(obstacles, obstacle_directions)

    plt.clf()
    
    plt.plot(x, y, 'go', label="Robot")
    plt.plot(target[0], target[1], 'bo', label="Target")
    
    for obstacle in obstacles:
        circle = plt.Circle(obstacle, obstacle_radius, color='purple', fill=True, label="Obstacle")
        plt.gca().add_patch(circle)
    
    plt.arrow(x, y, 0.5 * np.cos(theta), 0.5 * np.sin(theta), head_width=0.2, head_length=0.3, fc='green', ec='green')
    
    angles = np.linspace(-field_of_view, field_of_view, num_beams) + theta
    for angle in angles:
        end_x = x + sensor_range * np.cos(angle)
        end_y = y + sensor_range * np.sin(angle)
        plt.plot([x, end_x], [y, end_y], 'r--', alpha=0.5)
    
    plt.text(16, 12, f"Robot Position: ({x:.2f}, {y:.2f})", fontsize=10)
    plt.text(16, 11, f"Robot Orientation (theta): {theta:.2f} rad", fontsize=10)
    plt.text(16, 10, f"Target Position: ({target[0]}, {target[1]})", fontsize=10)
    plt.text(16, 9, f"Distance to Target: {np.hypot(target[0] - x, target[1] - y):.2f}", fontsize=10)
    plt.text(16, 8, f"Min Distance to Obstacle: {min_dist:.2f}", fontsize=10)
    
    plt.xlim(-2, 16)
    plt.ylim(-2, 12)
    plt.legend(loc='upper left')
    plt.gca().set_aspect('equal', adjustable='box')

    inset_ax = plt.axes([0.82, 0.05, 0.15, 0.15])
    polar_angles = np.linspace(-field_of_view, field_of_view, num_beams)
    inset_ax.bar(polar_angles, distances, width=0.1, color='red', alpha=0.6)
    inset_ax.set_title("Polar Histogram", fontsize=6)
    inset_ax.tick_params(axis='both', which='major', labelsize=6)
    inset_ax.grid(True)
    
    plt.pause(0.01)

plt.show()
