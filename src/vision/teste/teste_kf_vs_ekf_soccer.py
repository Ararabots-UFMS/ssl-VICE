"""
Testes específicos para aplicações em futebol de robôs.

Este módulo contém testes mais realistas para o contexto de Small Size League (SSL),
incluindo simulações de movimentos de bola, robôs e mudanças bruscas de trajetória.
"""

import pytest
import numpy as np
import time
from typing import List, Tuple
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from vision.kalman_filter import (
    KalmanFilterClass2D,
    ExtendedKalmanFilterClass2D,
    KalmanFilterClass1D,
    ExtendedKalmanFilterClass1D
)


class TestSoccerBall:
    """Testes com trajetória realista de bola de futebol."""
    
    @pytest.fixture
    def filters_2d(self):
        """Cria filtros otimizados para bola."""
        kf = KalmanFilterClass2D(x_sd=0.08, y_sd=0.08, u_x=0.0, u_y=0.0, sd_acceleration=0.5)
        ekf = ExtendedKalmanFilterClass2D(x_sd=0.08, y_sd=0.08, u_x=0.0, u_y=0.0,
                                          sd_acceleration=0.5, friction=0.08)
        return kf, ekf
    
    def generate_ball_trajectory(self, scenario: str) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Gera trajetória realista de bola.
        
        Cenários:
        - straight: movimento em linha reta
        - parabolic: movimento parabólico (chute)
        - bouncing: bola ricocheteando
        - spinning: bola com efeito
        """
        np.random.seed(42)
        dt = 0.016  # 60 FPS
        n_steps = 300
        
        true_trajectory = []
        measurements = []
        
        if scenario == "straight":
            # Movimento em linha reta com desaceleração
            for step in range(n_steps):
                t = step * dt
                # Velocidade inicial 3 m/s, desaceleração de 0.5 m/s²
                v0 = 3.0
                decel = 0.5
                
                x = v0 * t - 0.5 * decel * t**2
                y = 0.5
                vx = v0 - decel * t
                vy = 0.0
                
                if vx < 0:
                    vx = 0
                    x = v0**2 / (2 * decel)
                
                true_trajectory.append(np.array([x, y, vx, vy]))
                z = np.matrix([[x + np.random.normal(0, 0.03)],
                              [y + np.random.normal(0, 0.03)]])
                measurements.append(z)
        
        elif scenario == "parabolic":
            # Movimento parabólico (bola em voo após chute)
            for step in range(n_steps):
                t = step * dt
                # Velocidade inicial com componentes x, y
                vx0, vy0 = 2.0, 2.0
                # Gravidade reduzida (ambiente SSL)
                g = 1.5
                
                x = 0.0 + vx0 * t
                y = 1.0 + vy0 * t - 0.5 * g * t**2
                vx = vx0
                vy = vy0 - g * t
                
                if y < 0:  # Bola toca o chão
                    y = 0
                    vy = 0
                
                true_trajectory.append(np.array([x, y, vx, vy]))
                z = np.matrix([[x + np.random.normal(0, 0.04)],
                              [y + np.random.normal(0, 0.04)]])
                measurements.append(z)
        
        elif scenario == "bouncing":
            # Bola ricocheteando
            for step in range(n_steps):
                t = step * dt
                bounce_interval = 0.5  # Intervalo entre ricochetes
                bounce_cycle = t % bounce_interval
                
                vx0 = 1.5
                g = 2.0
                
                x = 0.0 + vx0 * t
                y = 0.2 * np.sin(t * 2 * np.pi / bounce_interval) * np.cos(bounce_cycle * np.pi / bounce_interval)
                vx = vx0
                vy = 0.2 * 2 * np.pi / bounce_interval * np.cos(t * 2 * np.pi / bounce_interval)
                
                if y < 0:
                    y = 0
                    vy = 0
                
                true_trajectory.append(np.array([x, y, vx, vy]))
                z = np.matrix([[x + np.random.normal(0, 0.04)],
                              [y + np.random.normal(0, 0.04)]])
                measurements.append(z)
        
        elif scenario == "spinning":
            # Bola com efeito (curva Magnus)
            for step in range(n_steps):
                t = step * dt
                v0 = 2.5
                omega = 2.0  # Velocidade angular
                # Força Magnus diminui com a distância
                lift = 0.5 * np.exp(-t)
                
                x = v0 * t
                y = 0.3 + lift * np.sin(omega * t)
                vx = v0
                vy = lift * omega * np.cos(omega * t)
                
                true_trajectory.append(np.array([x, y, vx, vy]))
                z = np.matrix([[x + np.random.normal(0, 0.04)],
                              [y + np.random.normal(0, 0.04)]])
                measurements.append(z)
        
        return true_trajectory, measurements
    
    def test_straight_line_trajectory(self, filters_2d):
        """Testa rastreamento de movimento em linha reta."""
        kf, ekf = filters_2d
        true_trajectory, measurements = self.generate_ball_trajectory("straight")
        dt = 0.016
        
        kf_errors = []
        ekf_errors = []
        
        for true_state, measurement in zip(true_trajectory, measurements):
            kf.predict(dt)
            kf.update(measurement)
            kf_state = np.array(kf.x).flatten()
            kf_error = np.linalg.norm(kf_state[:2] - true_state[:2])
            kf_errors.append(kf_error)
            
            ekf.predict(dt)
            ekf.update(measurement)
            ekf_state = np.array(ekf.x).flatten()
            ekf_error = np.linalg.norm(ekf_state[:2] - true_state[:2])
            ekf_errors.append(ekf_error)
        
        kf_mse = np.mean(np.array(kf_errors) ** 2)
        ekf_mse = np.mean(np.array(ekf_errors) ** 2)
        
        print(f"\n[BOLA] Movimento em Linha Reta:")
        print(f"  KF MSE:  {kf_mse:.6f}")
        print(f"  EKF MSE: {ekf_mse:.6f}")
        print(f"  Melhora: {(1 - ekf_mse/kf_mse)*100:.2f}%")
        
        assert kf_mse > 0 and ekf_mse > 0
    
    def test_parabolic_trajectory(self, filters_2d):
        """Testa rastreamento de movimento parabólico."""
        kf, ekf = filters_2d
        true_trajectory, measurements = self.generate_ball_trajectory("parabolic")
        dt = 0.016
        
        kf_errors = []
        ekf_errors = []
        
        for true_state, measurement in zip(true_trajectory, measurements):
            kf.predict(dt)
            kf.update(measurement)
            kf_state = np.array(kf.x).flatten()
            kf_error = np.linalg.norm(kf_state[:2] - true_state[:2])
            kf_errors.append(kf_error)
            
            ekf.predict(dt)
            ekf.update(measurement)
            ekf_state = np.array(ekf.x).flatten()
            ekf_error = np.linalg.norm(ekf_state[:2] - true_state[:2])
            ekf_errors.append(ekf_error)
        
        kf_mse = np.mean(np.array(kf_errors) ** 2)
        ekf_mse = np.mean(np.array(ekf_errors) ** 2)
        
        print(f"\n[BOLA] Movimento Parabólico:")
        print(f"  KF MSE:  {kf_mse:.6f}")
        print(f"  EKF MSE: {ekf_mse:.6f}")
        print(f"  Melhora: {(1 - ekf_mse/kf_mse)*100:.2f}%")
    
    def test_spinning_trajectory(self, filters_2d):
        """Testa rastreamento de bola com efeito (Magnus)."""
        kf, ekf = filters_2d
        true_trajectory, measurements = self.generate_ball_trajectory("spinning")
        dt = 0.016
        
        kf_errors = []
        ekf_errors = []
        
        for true_state, measurement in zip(true_trajectory, measurements):
            kf.predict(dt)
            kf.update(measurement)
            kf_state = np.array(kf.x).flatten()
            kf_error = np.linalg.norm(kf_state[:2] - true_state[:2])
            kf_errors.append(kf_error)
            
            ekf.predict(dt)
            ekf.update(measurement)
            ekf_state = np.array(ekf.x).flatten()
            ekf_error = np.linalg.norm(ekf_state[:2] - true_state[:2])
            ekf_errors.append(ekf_error)
        
        kf_mse = np.mean(np.array(kf_errors) ** 2)
        ekf_mse = np.mean(np.array(ekf_errors) ** 2)
        
        print(f"\n[BOLA] Movimento com Efeito (Magnus):")
        print(f"  KF MSE:  {kf_mse:.6f}")
        print(f"  EKF MSE: {ekf_mse:.6f}")
        print(f"  Melhora: {(1 - ekf_mse/kf_mse)*100:.2f}%")


class TestRobotMovement:
    """Testes com trajetória realista de robô."""
    
    @pytest.fixture
    def filters_2d(self):
        """Cria filtros otimizados para robô."""
        kf = KalmanFilterClass2D(x_sd=0.05, y_sd=0.05, u_x=0.0, u_y=0.0, sd_acceleration=2.0)
        ekf = ExtendedKalmanFilterClass2D(x_sd=0.05, y_sd=0.05, u_x=0.0, u_y=0.0,
                                          sd_acceleration=2.0, friction=0.03)
        return kf, ekf
    
    @pytest.fixture
    def filters_1d(self):
        """Cria filtros otimizados para orientação de robô."""
        kf = KalmanFilterClass1D(a_sd=0.05, u=0.0, sd_acceleration=5.0)
        ekf = ExtendedKalmanFilterClass1D(a_sd=0.05, u=0.0, sd_acceleration=5.0, friction=0.02)
        return kf, ekf
    
    def generate_robot_trajectory_2d(self) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Gera trajetória realista de robô em campo."""
        np.random.seed(42)
        dt = 0.016  # 60 FPS
        n_steps = 400
        
        true_trajectory = []
        measurements = []
        
        for step in range(n_steps):
            t = step * dt
            
            # Robô se movimenta em padrão retangular com aceleração/desaceleração
            cycle = 4.0  # Duração de um ciclo
            phase = (t % cycle) / cycle
            
            if phase < 0.25:  # Movimento para +X
                segment = phase * 4
                x = 0.5 + segment * 1.0
                y = 0.0
                vx = 1.0
                vy = 0.0
            elif phase < 0.5:  # Movimento para +Y
                segment = (phase - 0.25) * 4
                x = 1.5
                y = segment * 1.0
                vx = 0.0
                vy = 1.0
            elif phase < 0.75:  # Movimento para -X
                segment = (phase - 0.5) * 4
                x = 1.5 - segment * 1.0
                y = 1.0
                vx = -1.0
                vy = 0.0
            else:  # Movimento para -Y
                segment = (phase - 0.75) * 4
                x = 0.5
                y = 1.0 - segment * 1.0
                vx = 0.0
                vy = -1.0
            
            true_trajectory.append(np.array([x, y, vx, vy]))
            z = np.matrix([[x + np.random.normal(0, 0.02)],
                          [y + np.random.normal(0, 0.02)]])
            measurements.append(z)
        
        return true_trajectory, measurements
    
    def generate_robot_trajectory_1d(self) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Gera trajetória realista de orientação de robô."""
        np.random.seed(42)
        dt = 0.016
        n_steps = 400
        
        true_trajectory = []
        measurements = []
        
        for step in range(n_steps):
            t = step * dt
            
            # Robô rotaciona em padrão senoidal
            frequency = 0.5  # Hz
            amplitude = np.pi  # radianos
            
            theta = amplitude * np.sin(2 * np.pi * frequency * t)
            omega = amplitude * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * t)
            
            theta = (theta + np.pi) % (2 * np.pi) - np.pi
            
            true_trajectory.append(np.array([theta, omega]))
            z = np.matrix([[theta + np.random.normal(0, 0.03)]])
            measurements.append(z)
        
        return true_trajectory, measurements
    
    def test_robot_2d_movement(self, filters_2d):
        """Testa rastreamento de movimento 2D de robô."""
        kf, ekf = filters_2d
        true_trajectory, measurements = self.generate_robot_trajectory_2d()
        dt = 0.016
        
        kf_errors = []
        ekf_errors = []
        
        for true_state, measurement in zip(true_trajectory, measurements):
            kf.predict(dt)
            kf.update(measurement)
            kf_state = np.array(kf.x).flatten()
            kf_error = np.linalg.norm(kf_state[:2] - true_state[:2])
            kf_errors.append(kf_error)
            
            ekf.predict(dt)
            ekf.update(measurement)
            ekf_state = np.array(ekf.x).flatten()
            ekf_error = np.linalg.norm(ekf_state[:2] - true_state[:2])
            ekf_errors.append(ekf_error)
        
        kf_mse = np.mean(np.array(kf_errors) ** 2)
        ekf_mse = np.mean(np.array(ekf_errors) ** 2)
        
        print(f"\n[ROBÔ] Movimento 2D (Padrão Retangular):")
        print(f"  KF MSE:  {kf_mse:.6f}")
        print(f"  EKF MSE: {ekf_mse:.6f}")
        print(f"  Melhora: {(1 - ekf_mse/kf_mse)*100:.2f}%")
    
    def test_robot_1d_orientation(self, filters_1d):
        """Testa rastreamento de orientação de robô."""
        kf, ekf = filters_1d
        true_trajectory, measurements = self.generate_robot_trajectory_1d()
        dt = 0.016
        
        kf_errors = []
        ekf_errors = []
        
        for true_state, measurement in zip(true_trajectory, measurements):
            kf.predict(dt)
            kf.update(measurement)
            kf_state = np.array(kf.x).flatten()
            angle_diff = kf_state[0] - true_state[0]
            angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
            kf_error = abs(angle_diff)
            kf_errors.append(kf_error)
            
            ekf.predict(dt)
            ekf.update(measurement)
            ekf_state = np.array(ekf.x).flatten()
            angle_diff = ekf_state[0] - true_state[0]
            angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
            ekf_error = abs(angle_diff)
            ekf_errors.append(ekf_error)
        
        kf_mae = np.mean(kf_errors)
        ekf_mae = np.mean(ekf_errors)
        
        print(f"\n[ROBÔ] Orientação 1D (Padrão Senoidal):")
        print(f"  KF MAE:  {kf_mae:.6f} rad")
        print(f"  EKF MAE: {ekf_mae:.6f} rad")
        print(f"  Melhora: {(1 - ekf_mae/kf_mae)*100:.2f}%")


class TestPerformanceComparison:
    """Testes de desempenho em cenários realistas."""
    
    def test_real_time_feasibility(self):
        """Testa viabilidade em tempo real (deve rodar em menos de 16ms para 60FPS)."""
        kf = KalmanFilterClass2D()
        ekf = ExtendedKalmanFilterClass2D()
        dt = 0.016
        
        # 100 iterações (simulando 1600ms)
        n_iterations = 100
        
        # Teste KF
        start = time.perf_counter()
        for _ in range(n_iterations):
            kf.predict(dt)
            measurement = np.matrix([[1.0], [2.0]])
            kf.update(measurement)
        kf_time = (time.perf_counter() - start) / n_iterations * 1000  # ms
        
        # Teste EKF
        start = time.perf_counter()
        for _ in range(n_iterations):
            ekf.predict(dt)
            measurement = np.matrix([[1.0], [2.0]])
            ekf.update(measurement)
        ekf_time = (time.perf_counter() - start) / n_iterations * 1000
        
        print(f"\n[TEMPO REAL] Viabilidade 60 FPS (max 16.67ms por frame):")
        print(f"  KF 2D tempo por ciclo: {kf_time:.3f} ms (✓ viável)" if kf_time < 16.67 else f"  KF 2D tempo por ciclo: {kf_time:.3f} ms (✗ não viável)")
        print(f"  EKF 2D tempo por ciclo: {ekf_time:.3f} ms (✓ viável)" if ekf_time < 16.67 else f"  EKF 2D tempo por ciclo: {ekf_time:.3f} ms (✗ não viável)")
        
        assert kf_time < 16.67, "KF não viável em tempo real"
        assert ekf_time < 16.67, "EKF não viável em tempo real"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
