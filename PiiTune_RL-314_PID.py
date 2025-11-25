import pygame
import numpy as np
import sys

class PIDBalanceVisualizer:
    def __init__(self):
        pygame.init()
        self.WIDTH, self.HEIGHT = 1000, 600
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("PiiTune RL‑314 PID – Hypervolume Performance Explorer")
        
        # Colors
        self.BG_COLOR = (20, 20, 30)
        self.AXIS_COLOR = (100, 100, 150)
        self.BALANCE_POINT_COLOR = (255, 255, 0)
        self.MASS_COLOR = (0, 200, 255)
        self.SETPOINT_COLOR = (255, 100, 100)
        self.TEXT_COLOR = (255, 255, 255)
        self.GRID_COLOR = (50, 50, 70)
        self.COMPLETE_COLOR = (0, 255, 0)
        
        # Tuning parameters - matching C code
        self.TUNE_RELAY_HERTZ = 0.5
        self.TUNE_RELAY_HALF_PERIOD_RADIANS = 0.2
        self.TUNE_RELAY_FULL_PERIOD_RADIANS = 0.4
        self.TUNE_RELAY_HALF_PERIOD_US = int(1000000 / (self.TUNE_RELAY_HERTZ * 2))
        self.TUNE_RELAY_FULL_PERIOD_US = int(1000000 / self.TUNE_RELAY_HERTZ)
        
        self.TUNE_P_GAIN_INCREMENT = 0.1
        self.TUNE_D_GAIN_INCREMENT = 0.01
        self.TUNE_I_GAIN_INCREMENT = 0.001
        self.TUNE_HYPERVOLUME_CONVERGENCE = 0.10  # 10% relative threshold
        self.TUNE_RELATIVE_CHANGE_THRESHOLD = 0.10  # 10% relative change for zero-crossing
        self.FLT_EPSILON = 1.1920928955078125e-7
        
        # Physics parameters
        self.center_x = self.WIDTH // 2
        self.mass_radius = 20
        self.mass_x = self.center_x
        self.max_position = self.WIDTH * 0.4
        
        # PID parameters
        self.pid_gains = [0.0, 0.0, 0.0]  # [P, I, D]
        self.pid_setpoint = 0.0
        self.pid_integral = 0.0
        self.prev_error = 0.0
        
        # System dynamics
        self.velocity = 0.0
        self.position = 0.0
        self.dt = 1.0 / 211  # ≈ 0.00474
        
        # Tuning state - EXACTLY matching C structure
        self.state = {
            'relay_time': 0,
            'step_time': 0, 
            'settle_time': 0,
            'hv_prev': 0.0,
            'max_os': 0.0,
            'prev_err': 0.0,
            'stage': 0,
            'zero_crossings': 0,
            'active': False,
            'step_active': False,
            'measuring': False
        }
        
        # Tuning stages - matching C structure
        self.stages = [
            {'gain_idx': 0, 'inc': self.TUNE_P_GAIN_INCREMENT},  # P
            {'gain_idx': 2, 'inc': self.TUNE_D_GAIN_INCREMENT},  # D  
            {'gain_idx': 1, 'inc': self.TUNE_I_GAIN_INCREMENT}   # I
        ]
        
        # Visualization
        self.trail_positions = []
        self.sim_time = 0.0
        self.us_time = 0
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 20)
        self.large_font = pygame.font.Font(None, 36)
        
        # Start tuning
        self.start_tuning()
    
    def start_tuning(self):
        """Initialize tuning exactly like C code"""
        self.state['active'] = True
        self.state['step_active'] = True  
        self.state['measuring'] = True
        self.pid_setpoint = self.TUNE_RELAY_HALF_PERIOD_RADIANS
        self.state['relay_time'] = self.us_time
        self.state['step_time'] = self.us_time
        self.state['prev_err'] = self.position - self.pid_setpoint
        self.state['zero_crossings'] = 0
    
    def system_dynamics(self, control_input):
        """System being controlled"""
        mass = 1.0
        damping = 0.8
        spring_constant = 1.5
        
        acceleration = (control_input - damping * self.velocity - spring_constant * self.position) / mass
        self.velocity += acceleration * self.dt
        self.position += self.velocity * self.dt
        
        # Small noise like real system
        self.position += np.random.normal(0, 0.005)
        
        return self.position
    
    def pid_control(self):
        """Updated PID controller matching new C implementation"""
        # Constants (set to match your C defines)
        PID_LOOP_HZ = 1.0 / self.dt
        D_ALPHA = 0.75  # Example value, set to match your C code
        PID_I_MIN = -58.33
        PID_I_MAX = 58.33
        PID_OUT_MIN = -116.67
        PID_OUT_MAX = 116.67

        sp = self.pid_setpoint
        pv = self.position
        Kp, Ki, Kd = self.pid_gains

        # Calculate terms
        P = sp - pv
        D = -(pv - getattr(self, 'prev_pv', 0.0)) * PID_LOOP_HZ

        # Derivative filter
        self.pid_derivative = getattr(self, 'pid_derivative', 0.0)
        self.pid_derivative += (D - self.pid_derivative) * D_ALPHA

        # Integral calculation and clamping
        I = self.pid_integral + P * self.dt
        I = np.clip(I, PID_I_MIN, PID_I_MAX)

        # Output calculation
        out = Kp * P + Ki * I + Kd * self.pid_derivative
        update_integral = PID_OUT_MIN < out < PID_OUT_MAX
        self.pid_integral = I if update_integral else self.pid_integral

        # Final output calculation and clamping
        out = Kp * P + Ki * self.pid_integral + Kd * self.pid_derivative
        out = np.clip(out, PID_OUT_MIN, PID_OUT_MAX)

        # Store previous process value for next derivative calculation
        self.prev_pv = pv

        return out
    
    def pid_tune_step(self):
        """EXACT Python implementation matching C code - Pure Relative Detection"""
        now = self.us_time
        err = self.position - self.pid_setpoint
        
        # Initialize tuning state on first call
        if not self.state['active']:
            self.state = {
                'relay_time': now, 'step_time': now, 'settle_time': 0,
                'hv_prev': 0.0, 'max_os': 0.0, 'prev_err': err,
                'stage': 0, 'zero_crossings': 0,
                'active': True, 'step_active': True, 'measuring': True
            }
            self.pid_setpoint = self.TUNE_RELAY_HALF_PERIOD_RADIANS
            return False
        
        # Monitor system response during relay period
        if now - self.state['relay_time'] < self.TUNE_RELAY_FULL_PERIOD_US:
            # Track maximum overshoot during step response
            if self.state['step_active']:
                os = abs(err)
                if os > self.state['max_os']:
                    self.state['max_os'] = os
                if now - self.state['step_time'] >= self.TUNE_RELAY_HALF_PERIOD_US:
                    self.state['step_active'] = False
            
            # Pure relative oscillation detection: detect zero crossings
            if self.state['measuring']:
                # Create boolean conditions exactly like C code
                is_zero_crossing = (self.state['prev_err'] * err) <= 0.0
                relative_change = abs(err - self.state['prev_err']) / (abs(self.state['prev_err']) + self.FLT_EPSILON)
                is_significant_change = relative_change > self.TUNE_RELATIVE_CHANGE_THRESHOLD
                
                # Detect when error changes sign with significant relative change
                if is_zero_crossing and is_significant_change:
                    # System has oscillated enough - consider it settled after one full cycle
                    self.state['zero_crossings'] += 1
                    if self.state['zero_crossings'] >= 2:  # At least one full oscillation cycle
                        self.state['settle_time'] = now
                        self.state['measuring'] = False
                
                self.state['prev_err'] = err
            return False
        
        # Relay cycle complete - calculate performance metrics
        current_os = self.state['max_os']
        if self.state['measuring']:
            settle_time = self.TUNE_RELAY_FULL_PERIOD_US
        else:
            settle_time = float(self.state['settle_time'] - self.state['step_time'])
        
        # Calculate hypervolume performance metric - exactly like C code
        hv = ((1.0 - min(settle_time / self.TUNE_RELAY_FULL_PERIOD_US, 1.0)) *
              (1.0 - min(current_os / self.TUNE_RELAY_FULL_PERIOD_RADIANS, 1.0)))
        
        # Non-responsive detection: insufficient oscillation (hypervolume < 10%)
        if hv < self.TUNE_HYPERVOLUME_CONVERGENCE:
            self.state['settle_time'] = self.state['step_time'] + self.TUNE_RELAY_FULL_PERIOD_US
            self.state['measuring'] = False
        
        # Prepare for next relay cycle - toggle excitation direction
        self.state['step_active'] = True
        self.state['step_time'] = now
        self.state['max_os'] = 0.0
        self.state['measuring'] = True
        self.pid_setpoint = -self.pid_setpoint
        self.state['relay_time'] = now
        self.state['prev_err'] = err
        self.state['zero_crossings'] = 0  # Reset oscillation counter for next cycle
        
        # Hypervolume convergence check - pure relative improvement
        if (self.state['hv_prev'] == 0.0 or 
            (abs(hv - self.state['hv_prev']) / self.state['hv_prev'] >= self.TUNE_HYPERVOLUME_CONVERGENCE)):
            gain_idx = self.stages[self.state['stage']]['gain_idx']
            inc = self.stages[self.state['stage']]['inc']
            self.pid_gains[gain_idx] += inc
            self.state['hv_prev'] = hv
            return False
        
        # Current stage complete - progress to next PID component
        self.pid_setpoint = 0.0  # Return system to center
        
        # Check if all stages are complete
        if self.state['stage'] >= 2:
            self.state['active'] = False
            return True
        
        # Initialize next tuning stage with initial gain increment
        self.state['stage'] += 1
        gain_idx = self.stages[self.state['stage']]['gain_idx']
        inc = self.stages[self.state['stage']]['inc']
        self.pid_gains[gain_idx] += inc
        self.state['hv_prev'] = 0.0
        return False
    
    def update(self):
        """Update physics and tuning"""
        self.sim_time += self.dt
        self.us_time = int(self.sim_time * 1e6)
        
        # Get control input
        control = self.pid_control()
        
        # Update system dynamics
        self.system_dynamics(control)
        
        # Run tuning algorithm only if still active
        tuning_complete = False
        if self.state['active']:
            tuning_complete = self.pid_tune_step()
            if tuning_complete:
                print(f"Tuning complete! Final gains: P={self.pid_gains[0]:.3f}, I={self.pid_gains[1]:.3f}, D={self.pid_gains[2]:.3f}")
        else:
            # Tuning complete - maintain balance at center
            self.pid_setpoint = 0.0
        
        # Update mass position for visualization
        self.mass_x = self.center_x + self.position * self.max_position
        
        # Add to trail
        self.trail_positions.append((self.mass_x, self.HEIGHT // 2))
        if len(self.trail_positions) > 100:
            self.trail_positions.pop(0)
        
        return tuning_complete
    
    def draw(self):
        """Draw everything"""
        self.screen.fill(self.BG_COLOR)
        
        # Draw grid
        for x in range(0, self.WIDTH, 50):
            pygame.draw.line(self.screen, self.GRID_COLOR, (x, 0), (x, self.HEIGHT), 1)
        for y in range(0, self.HEIGHT, 50):
            pygame.draw.line(self.screen, self.GRID_COLOR, (0, y), (self.WIDTH, y), 1)
        
        # Draw center axis
        pygame.draw.line(self.screen, self.AXIS_COLOR, (self.center_x, 50), (self.center_x, self.HEIGHT - 50), 3)
        
        # Draw balance point
        pygame.draw.circle(self.screen, self.BALANCE_POINT_COLOR, (self.center_x, self.HEIGHT // 2), 8)
        pygame.draw.circle(self.screen, self.BALANCE_POINT_COLOR, (self.center_x, self.HEIGHT // 2), 15, 2)
        
        # Draw setpoint indicator
        setpoint_x = self.center_x + self.pid_setpoint * self.max_position
        pygame.draw.line(self.screen, self.SETPOINT_COLOR, 
                        (setpoint_x, self.HEIGHT // 2 - 40), 
                        (setpoint_x, self.HEIGHT // 2 + 40), 2)
        
        # Draw position trail
        for i, (trail_x, trail_y) in enumerate(self.trail_positions):
            alpha = i / len(self.trail_positions)
            color = (int(0 * alpha), int(200 * alpha), int(255 * alpha))
            pygame.draw.circle(self.screen, color, (int(trail_x), int(trail_y)), 2)
        
        # Draw mass
        pygame.draw.circle(self.screen, self.MASS_COLOR, (int(self.mass_x), self.HEIGHT // 2), self.mass_radius)
        pygame.draw.circle(self.screen, (255, 255, 255), (int(self.mass_x), self.HEIGHT // 2), self.mass_radius, 2)
        
        # Draw info panel
        self.draw_info_panel()
        
        pygame.display.flip()
    
    def draw_info_panel(self):
        """Draw detailed information"""
        # PID gains
        gains_text = f"PID Gains: P={self.pid_gains[0]:.3f}  I={self.pid_gains[1]:.4f}  D={self.pid_gains[2]:.3f}"
        gains_surface = self.font.render(gains_text, True, self.TEXT_COLOR)
        self.screen.blit(gains_surface, (20, 20))
        
        # Status
        if not self.state['active']:
            # Tuning complete - show success message
            status_text = "TUNING COMPLETE!"
            status_surface = self.large_font.render(status_text, True, self.COMPLETE_COLOR)
            text_width = status_surface.get_width()
            self.screen.blit(status_surface, (self.WIDTH // 2 - text_width // 2, 30))
            
            # Final performance
            perf_text = f"Final gains provide stable balance"
            perf_surface = self.font.render(perf_text, True, self.COMPLETE_COLOR)
            self.screen.blit(perf_surface, (20, 60))
        else:
            # Still tuning - show current stage
            stage_names = ["P-tuning", "D-tuning", "I-tuning"]
            current_stage = stage_names[min(self.state['stage'], 2)]
            status_text = f"Status: {current_stage}"
            status_surface = self.font.render(status_text, True, self.TEXT_COLOR)
            self.screen.blit(status_surface, (20, 50))
            
            # Hypervolume info
            hv_text = f"Hypervolume: {self.state['hv_prev']:.3f}  OS: {self.state['max_os']:.3f}"
            hv_surface = self.font.render(hv_text, True, self.TEXT_COLOR)
            self.screen.blit(hv_surface, (20, 80))
            
            # Oscillation info
            osc_text = f"Zero Crossings: {self.state['zero_crossings']}"
            osc_surface = self.font.render(osc_text, True, self.TEXT_COLOR)
            self.screen.blit(osc_surface, (20, 110))
            
            # Current error for debugging
            current_error = self.position - self.pid_setpoint
            err_text = f"Current Error: {current_error:.3f}"
            err_surface = self.font.render(err_text, True, self.TEXT_COLOR)
            self.screen.blit(err_surface, (20, 140))
        
        # System state
        state_text = f"Position: {self.position:.3f}  Setpoint: {self.pid_setpoint:.3f}"
        state_surface = self.font.render(state_text, True, self.TEXT_COLOR)
        self.screen.blit(state_surface, (20, 170 if self.state['active'] else 140))
        
        # Algorithm info
        if self.state['active']:
            instructions = [
                "HYPERVOLUME TUNING:",
                "• Zero-crossing + 10% relative change",
                "• Settling: 2+ significant crossings", 
                "• Progress: HV improvement ≥10%",
                "• Non-responsive: HV <10%",
                "• Stages: P → D → I"
            ]
        else:
            instructions = [
                "TUNING COMPLETE!",
                "• System balancing with final gains",
                f"• P={self.pid_gains[0]:.3f}, I={self.pid_gains[1]:.4f}, D={self.pid_gains[2]:.3f}",
                "• Press R to restart tuning",
                "• Press ESC to exit"
            ]
        
        for i, instruction in enumerate(instructions):
            inst_surface = self.small_font.render(instruction, True, self.TEXT_COLOR)
            self.screen.blit(inst_surface, (self.WIDTH - 300, 20 + i * 25))
    
    def run(self):
        """Main game loop"""
        clock = pygame.time.Clock()
        running = True
        
        print("Running Pure Relative Oscillation Detection PID Tuning")
        print("Using EXACT same algorithm as C code")
        print("Zero-crossing detection with 10% relative change threshold")
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        # Reset and start over
                        self.__init__()
            
            self.update()
            self.draw()
            clock.tick(120)
        
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    visualizer = PIDBalanceVisualizer()
    visualizer.run()