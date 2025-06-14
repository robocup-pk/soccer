import sys
import signal
import numpy as np
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QLabel, QDoubleSpinBox, QPushButton,
                               QGroupBox, QGridLayout)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPainter, QPen, QBrush, QColor, QPolygon
import math

class OmniwheelSimulator(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Omniwheel Robot Simulator")
        self.setGeometry(100, 100, 1400, 1000)  # Larger window
        
        # Robot parameters (meters)
        self.wheel_radius = 0.05  # 5cm wheel radius
        self.robot_size = 0.3     # 30cm robot size for display
        
        # Default wheel configuration - square with wheels at corners
        # Positions relative to robot center (meters)
        self.wheel_positions = np.array([
            [0.15, 0.15],   # wheel 1: front-right
            [-0.15, 0.15],  # wheel 2: front-left
            [-0.15, -0.15], # wheel 3: rear-left
            [0.15, -0.15]   # wheel 4: rear-right
        ])
        
        # Wheel angles (radians) - wheels pointing perpendicular to radial direction
        self.wheel_angles = np.array([
            -np.pi/4,     # -45° (wheel 1)
            np.pi/4,      # 45° (wheel 2)
            3*np.pi/4,    # 135° (wheel 3)
            -3*np.pi/4    # -135° (wheel 4)
        ])
        
        # Wheel RPMs - default values to make robot go straight forward
        self.wheel_rpms = np.array([60.0, 60.0, 60.0, 60.0])  # All same RPM for straight motion
        
        # Robot state (world coordinates)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # heading angle
        
        # Simulation parameters
        self.dt = 0.05  # 50ms update rate
        self.scale = 200  # pixels per meter for display
        
        # Calculate Jacobian pseudoinverse
        self.update_jacobian()
        
        # Initialize jacobian display after UI is set up
        QTimer.singleShot(100, self.update_jacobian_display)
        
        # Setup UI
        self.setup_ui()
        
        # Setup timer for simulation
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.simulation_running = False
        
    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Left panel for controls
        control_panel = QWidget()
        control_panel.setFixedWidth(350)
        control_layout = QVBoxLayout()
        control_panel.setLayout(control_layout)
        
        # Robot parameters group
        robot_group = QGroupBox("Robot Parameters")
        robot_layout = QGridLayout()
        robot_group.setLayout(robot_layout)
        
        # Wheel radius
        robot_layout.addWidget(QLabel("Wheel Radius (m):"), 0, 0)
        self.wheel_radius_spin = QDoubleSpinBox()
        self.wheel_radius_spin.setRange(0.01, 0.2)
        self.wheel_radius_spin.setValue(self.wheel_radius)
        self.wheel_radius_spin.setSingleStep(0.01)
        self.wheel_radius_spin.setDecimals(3)
        self.wheel_radius_spin.valueChanged.connect(self.update_wheel_radius)
        robot_layout.addWidget(self.wheel_radius_spin, 0, 1)
        
        control_layout.addWidget(robot_group)
        
        # Wheel positions and angles group
        wheels_group = QGroupBox("Wheel Configuration")
        wheels_layout = QGridLayout()
        wheels_group.setLayout(wheels_layout)
        
        # Headers
        wheels_layout.addWidget(QLabel("Wheel"), 0, 0)
        wheels_layout.addWidget(QLabel("X (m)"), 0, 1)
        wheels_layout.addWidget(QLabel("Y (m)"), 0, 2)
        wheels_layout.addWidget(QLabel("Angle (°)"), 0, 3)
        
        self.pos_spins = []
        self.angle_spins = []
        
        # Angle range should allow negative values
        for i in range(4):
            # Wheel label
            wheels_layout.addWidget(QLabel(f"Wheel {i+1}:"), i+1, 0)
            
            # X position
            x_spin = QDoubleSpinBox()
            x_spin.setRange(-0.5, 0.5)
            x_spin.setValue(self.wheel_positions[i][0])
            x_spin.setSingleStep(0.01)
            x_spin.setDecimals(3)
            x_spin.valueChanged.connect(self.update_wheel_positions)
            wheels_layout.addWidget(x_spin, i+1, 1)
            
            # Y position
            y_spin = QDoubleSpinBox()
            y_spin.setRange(-0.5, 0.5)
            y_spin.setValue(self.wheel_positions[i][1])
            y_spin.setSingleStep(0.01)
            y_spin.setDecimals(3)
            y_spin.valueChanged.connect(self.update_wheel_positions)
            wheels_layout.addWidget(y_spin, i+1, 2)
            
            # Angle (allow negative values)
            angle_spin = QDoubleSpinBox()
            angle_spin.setRange(-180, 180)
            angle_spin.setValue(self.wheel_angles[i] * 180 / np.pi)
            angle_spin.setSingleStep(1)
            angle_spin.setDecimals(1)
            angle_spin.valueChanged.connect(self.update_wheel_angles)
            wheels_layout.addWidget(angle_spin, i+1, 3)
            
            self.pos_spins.extend([x_spin, y_spin])
            self.angle_spins.append(angle_spin)
        
        # Reset config button
        reset_config_layout = QHBoxLayout()
        reset_config_layout.addStretch()
        reset_config_button = QPushButton("Reset Configuration")
        reset_config_button.clicked.connect(self.reset_configuration)
        reset_config_layout.addWidget(reset_config_button)
        reset_config_layout.addStretch()
        
        wheels_layout.addLayout(reset_config_layout, 6, 0, 1, 4)  # Span across all columns
            
        control_layout.addWidget(wheels_group)
        
        # Wheel RPMs group
        rpm_group = QGroupBox("Wheel RPMs")
        rpm_layout = QGridLayout()
        rpm_group.setLayout(rpm_layout)
        
        self.rpm_spins = []
        for i in range(4):
            rpm_layout.addWidget(QLabel(f"Wheel {i+1}:"), i, 0)
            rpm_spin = QDoubleSpinBox()
            rpm_spin.setRange(-300, 300)
            rpm_spin.setValue(self.wheel_rpms[i])
            rpm_spin.setSingleStep(10)
            rpm_spin.setDecimals(1)
            rpm_layout.addWidget(rpm_spin, i, 1)
            self.rpm_spins.append(rpm_spin)
            
        control_layout.addWidget(rpm_group)
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.start_simulation)
        button_layout.addWidget(self.start_button)
        
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_simulation)
        self.stop_button.setEnabled(False)
        button_layout.addWidget(self.stop_button)
        
        button_layout.addWidget(QLabel("  "))  # Spacer
        
        reset_button = QPushButton("Reset Position")
        reset_button.clicked.connect(self.reset_position)
        button_layout.addWidget(reset_button)
        
        control_layout.addLayout(button_layout)
        
        # Status display
        status_group = QGroupBox("Robot Status")
        status_layout = QGridLayout()
        status_group.setLayout(status_layout)
        
        self.position_label = QLabel(f"Position: ({self.robot_x:.3f}, {self.robot_y:.3f})")
        self.heading_label = QLabel(f"Heading: {self.robot_theta:.3f} rad")
        
        status_layout.addWidget(self.position_label, 0, 0)
        status_layout.addWidget(self.heading_label, 1, 0)
        
        control_layout.addWidget(status_group)
        
        # Jacobian display
        jacobian_group = QGroupBox("Jacobian Matrix")
        jacobian_layout = QVBoxLayout()
        jacobian_group.setLayout(jacobian_layout)
        
        self.jacobian_label = QLabel()
        self.jacobian_label.setFont(QApplication.font())
        font = self.jacobian_label.font()
        font.setFamily("Courier")  # Monospace font for better matrix alignment
        font.setPointSize(8)
        self.jacobian_label.setFont(font)
        self.jacobian_label.setWordWrap(True)
        self.jacobian_label.setAlignment(Qt.AlignLeft)
        jacobian_layout.addWidget(self.jacobian_label)
        
        control_layout.addWidget(jacobian_group)
        
        control_layout.addStretch()
        
        main_layout.addWidget(control_panel)
        
        # Right panel for visualization
        self.canvas = SimulationCanvas(self)
        main_layout.addWidget(self.canvas)
        
    def closeEvent(self, event):
        """Handle window close event"""
        self.timer.stop()
        event.accept()
        
    def update_wheel_radius(self):
        self.wheel_radius = self.wheel_radius_spin.value()
        
    def update_wheel_positions(self):
        for i in range(4):
            self.wheel_positions[i][0] = self.pos_spins[i*2].value()
            self.wheel_positions[i][1] = self.pos_spins[i*2+1].value()
        self.update_jacobian()
        self.canvas.update()  # Force canvas redraw
        
    def update_wheel_angles(self):
        for i in range(4):
            self.wheel_angles[i] = self.angle_spins[i].value() * np.pi / 180
        self.update_jacobian()
        self.canvas.update()  # Force canvas redraw
        
    def update_jacobian(self):
        """Compute Jacobian matrix and its pseudoinverse"""
        J = []
        for i in range(4):
            x_i, y_i = self.wheel_positions[i]
            beta_i = self.wheel_angles[i]
            
            row = [
                np.cos(beta_i),
                np.sin(beta_i),
                x_i * np.sin(beta_i) - y_i * np.cos(beta_i)
            ]
            J.append(row)
        
        self.J = np.array(J)
        self.J_pinv = np.linalg.pinv(self.J)
        self.update_jacobian_display()
        
    def update_jacobian_display(self):
        """Update the Jacobian matrix display"""
        if hasattr(self, 'jacobian_label'):
            jacobian_text = "J = \n"
            for i, row in enumerate(self.J):
                jacobian_text += f"[{row[0]:7.3f} {row[1]:7.3f} {row[2]:7.3f}]\n"
            
            jacobian_text += "\nJ⁺ = \n"
            for i, row in enumerate(self.J_pinv):
                jacobian_text += f"[{row[0]:7.3f} {row[1]:7.3f} {row[2]:7.3f} {row[3]:7.3f}]\n"
            
            self.jacobian_label.setText(jacobian_text)
        
    def reset_position(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
    def set_straight_motion(self):
        """Set all wheels to same RPM for straight forward motion"""
        for rpm_spin in self.rpm_spins:
            rpm_spin.setValue(60.0)
            
    def reset_configuration(self):
        """Reset wheel configuration to default values"""
        # Reset wheel positions to default square configuration
        default_positions = np.array([
            [0.15, 0.15],   # wheel 1: front-right
            [-0.15, 0.15],  # wheel 2: front-left
            [-0.15, -0.15], # wheel 3: rear-left
            [0.15, -0.15]   # wheel 4: rear-right
        ])
        
        # Reset wheel angles to default
        default_angles = np.array([
            -np.pi/4,     # -45° (wheel 1)
            np.pi/4,      # 45° (wheel 2)
            3*np.pi/4,    # 135° (wheel 3)
            -3*np.pi/4    # -135° (wheel 4)
        ])
        
        # Update the arrays
        self.wheel_positions = default_positions.copy()
        self.wheel_angles = default_angles.copy()
        
        # Update the UI controls
        for i in range(4):
            self.pos_spins[i*2].setValue(self.wheel_positions[i][0])     # X
            self.pos_spins[i*2+1].setValue(self.wheel_positions[i][1])   # Y
            self.angle_spins[i].setValue(self.wheel_angles[i] * 180 / np.pi)  # Angle in degrees
        
        # Recalculate jacobian
        self.update_jacobian()
        
    def start_simulation(self):
        """Start the simulation"""
        self.simulation_running = True
        self.timer.start(int(self.dt * 1000))
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        
    def stop_simulation(self):
        """Stop the simulation"""
        self.simulation_running = False
        self.timer.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        # Ensure canvas shows current state when stopped
        self.canvas.repaint()
        
    def update_simulation(self):
        # Get current wheel RPMs
        for i in range(4):
            self.wheel_rpms[i] = self.rpm_spins[i].value()
        
        # Convert RPM to rad/s
        wheel_angular_velocities = self.wheel_rpms * 2 * np.pi / 60
        
        # Inverse kinematics to get body velocities
        body_velocities = self.wheel_radius * self.J_pinv @ wheel_angular_velocities
        
        v_x_body, v_y_body, omega_body = body_velocities
        
        # Transform to global frame
        cos_theta = np.cos(self.robot_theta)
        sin_theta = np.sin(self.robot_theta)
        
        v_x_global = cos_theta * v_x_body - sin_theta * v_y_body
        v_y_global = sin_theta * v_x_body + cos_theta * v_y_body
        omega_global = omega_body
        
        # Integrate position
        self.robot_x += v_x_global * self.dt
        self.robot_y += v_y_global * self.dt
        self.robot_theta += omega_global * self.dt
        
        # Normalize angle
        self.robot_theta = np.arctan2(np.sin(self.robot_theta), np.cos(self.robot_theta))
        
        # Update status display
        self.position_label.setText(f"Position: ({self.robot_x:.3f}, {self.robot_y:.3f})")
        self.heading_label.setText(f"Heading: {self.robot_theta:.3f} rad")
        
        # Update canvas
        self.canvas.update()


class SimulationCanvas(QWidget):
    def __init__(self, simulator):
        super().__init__()
        self.simulator = simulator
        self.setMinimumSize(800, 800)  # Larger canvas
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Get canvas dimensions
        width = self.width()
        height = self.height()
        center_x = width // 2
        center_y = height // 2
        
        # Clear background
        painter.fillRect(self.rect(), QColor(240, 240, 240))
        
        # Draw coordinate system
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        
        # Grid lines (finer grid for larger arena)
        grid_spacing = 40  # Smaller grid spacing for more detail
        for i in range(0, width, grid_spacing):
            painter.drawLine(i, 0, i, height)
        for i in range(0, height, grid_spacing):
            painter.drawLine(0, i, width, i)
            
        # Main axes
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.drawLine(0, center_y, width, center_y)  # X axis
        painter.drawLine(center_x, 0, center_x, height)  # Y axis
        
        # Axis labels
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        painter.drawText(width - 20, center_y - 10, "X")
        painter.drawText(center_x + 10, 20, "Y")
        
        # Convert robot position to screen coordinates
        robot_screen_x = center_x + self.simulator.robot_x * self.simulator.scale
        robot_screen_y = center_y - self.simulator.robot_y * self.simulator.scale  # Flip Y
        
        # Draw robot body (circular)
        painter.setPen(QPen(QColor(0, 0, 255), 2))
        painter.setBrush(QBrush(QColor(0, 0, 255, 50)))
        
        robot_radius_pixels = self.simulator.robot_size * self.simulator.scale / 2
        
        # Rotate coordinate system for robot
        painter.save()
        painter.translate(robot_screen_x, robot_screen_y)
        painter.rotate(-self.simulator.robot_theta * 180 / np.pi)  # Negative for screen coordinates
        
        # Draw circular robot body
        painter.drawEllipse(-robot_radius_pixels, -robot_radius_pixels, 
                          robot_radius_pixels * 2, robot_radius_pixels * 2)
        
        # Draw robot heading arrow
        painter.setPen(QPen(QColor(255, 0, 0), 3))
        arrow_length = robot_radius_pixels * 0.7
        painter.drawLine(0, 0, arrow_length, 0)
        # Arrow head
        painter.drawLine(arrow_length, 0, arrow_length - 10, -5)
        painter.drawLine(arrow_length, 0, arrow_length - 10, 5)
        
        # Draw wheels as rectangles
        painter.setPen(QPen(QColor(0, 150, 0), 2))
        painter.setBrush(QBrush(QColor(0, 150, 0)))
        
        # Calculate wheel dimensions based on actual wheel radius
        wheel_radius_pixels = self.simulator.wheel_radius * self.simulator.scale
        wheel_width = wheel_radius_pixels * 2  # Diameter for width
        wheel_height = wheel_radius_pixels * 0.6  # Thinner height for rectangle
        
        for i, (pos, angle) in enumerate(zip(self.simulator.wheel_positions, self.simulator.wheel_angles)):
            # Wheel position relative to robot center
            wheel_x = pos[0] * self.simulator.scale
            wheel_y = -pos[1] * self.simulator.scale  # Flip Y
            
            # Save painter state for wheel rotation
            painter.save()
            painter.translate(wheel_x, wheel_y)
            painter.rotate(-angle * 180 / np.pi)  # Rotate wheel to its orientation
            
            # Draw rectangular wheel
            painter.drawRect(-wheel_width/2, -wheel_height/2, wheel_width, wheel_height)
            
            # Draw wheel orientation line (along the wheel's rolling direction)
            painter.setPen(QPen(QColor(0, 100, 0), 2))
            line_length = wheel_width * 0.4
            painter.drawLine(-line_length/2, 0, line_length/2, 0)
            
            # Restore for number drawing
            painter.restore()
            
            # Draw wheel number - positioned in the direction of wheel rolling
            painter.setPen(QPen(QColor(255, 255, 255), 1))
            painter.setBrush(QBrush(QColor(255, 255, 255)))
            font = painter.font()
            font.setPointSize(10)
            font.setBold(True)
            painter.setFont(font)
            
            # Calculate number position offset in wheel direction
            offset_distance = 25  # Distance from wheel center
            number_x = wheel_x + offset_distance * np.cos(angle)
            number_y = wheel_y - offset_distance * np.sin(angle)  # Flip Y for screen coords
            
            # Draw white background circle for number
            number_bg_size = 16
            painter.drawEllipse(int(number_x - number_bg_size/2), int(number_y - number_bg_size/2), 
                              number_bg_size, number_bg_size)
            
            # Draw wheel number text
            painter.setPen(QPen(QColor(0, 0, 0), 1))
            text_rect = painter.boundingRect(int(number_x - 10), int(number_y - 10), 20, 20, 
                                           Qt.AlignCenter, str(i + 1))
            painter.drawText(text_rect, Qt.AlignCenter, str(i + 1))
        
        painter.restore()
        
        # Draw robot trail
        # (For now, just show current position)
        painter.setPen(QPen(QColor(255, 0, 0), 4))
        painter.drawEllipse(int(robot_screen_x - 2), int(robot_screen_y - 2), 4, 4)


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\nReceived interrupt signal. Closing application...")
    QApplication.quit()

def main():
    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    app = QApplication(sys.argv)
    
    # Create a timer to allow Python to process signals
    timer = QTimer()
    timer.start(100)  # Check for signals every 100ms
    timer.timeout.connect(lambda: None)
    
    simulator = OmniwheelSimulator()
    simulator.show()
    
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting...")
        app.quit()

if __name__ == "__main__":
    main()