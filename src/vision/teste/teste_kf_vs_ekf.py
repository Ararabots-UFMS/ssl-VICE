"""
Testes unitários para comparar KalmanFilter (KF) com ExtendedKalmanFilter (EKF).

Este módulo avalia o desempenho de ambos os filtros em cenários 2D e 1D,
medindo métricas como erro de rastreamento, velocidade de convergência e tempo de processamento.
"""

import pytest
import numpy as np
import time
from typing import Tuple, Dict, List
import sys
from pathlib import Path

# Adicionar o caminho para importar os módulos
sys.path.insert(0, str(Path(__file__).parent.parent))

from vision.kalman_filter import (
    KalmanFilterClass2D,
    ExtendedKalmanFilterClass2D,
    KalmanFilterClass1D,
    ExtendedKalmanFilterClass1D
)


class TestKFvsEKF2D:
    """Testes para comparação de KF vs EKF em 2D."""
    
    @pytest.fixture
    def filters_2d(self):
        """Cria instâncias dos filtros 2D."""
        kf = KalmanFilterClass2D(x_sd=0.1, y_sd=0.1, u_x=0.0, u_y=0.0, sd_acceleration=1.0)
        ekf = ExtendedKalmanFilterClass2D(x_sd=0.1, y_sd=0.1, u_x=0.0, u_y=0.0, 
                                          sd_acceleration=1.0, friction=0.05)
        return kf, ekf
    
    @pytest.fixture
    def trajectory_2d(self) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Cria uma trajetória sintética 2D com movimento circular com atrito."""
        np.random.seed(42)
        dt = 0.016  # ~60 FPS
        n_steps = 500
        
        true_trajectory = []
        measurements = []
        
        # Simular movimento em círculo com velocidade decrescente (atrito)
        t = np.linspace(0, 10, n_steps)
        friction = 0.02
        
        for i, time_step in enumerate(t):
            # Velocidade angular decrescente
            omega = 2 * np.pi * 0.5 * np.exp(-friction * time_step)
            
            # Posição circular com raio decrescente
            radius = 2.0 * np.exp(-friction * time_step)
            x_true = radius * np.cos(omega * time_step)
            y_true = radius * np.sin(omega * time_step)
            
            # Velocidade
            vx_true = -radius * omega * np.sin(omega * time_step) - x_true * friction
            vy_true = radius * omega * np.cos(omega * time_step) - y_true * friction
            
            true_trajectory.append(np.array([x_true, y_true, vx_true, vy_true]))
            
            # Medir com ruído
            z = np.matrix([[x_true + np.random.normal(0, 0.05)],
                          [y_true + np.random.normal(0, 0.05)]])
            measurements.append(z)
        
        return true_trajectory, measurements
    
    def test_2d_tracking_accuracy(self, filters_2d, trajectory_2d):
        """Testa a acurácia de rastreamento 2D."""
        kf, ekf = filters_2d
        true_trajectory, measurements = trajectory_2d
        dt = 0.016
        
        kf_errors = []
        ekf_errors = []
        
        for true_state, measurement in zip(true_trajectory, measurements):
            # KF
            kf.predict(dt)
            kf.update(measurement)
            kf_state = np.array(kf.x).flatten()
            kf_error = np.linalg.norm(kf_state[:2] - true_state[:2])
            kf_errors.append(kf_error)
            
            # EKF
            ekf.predict(dt)
            ekf.update(measurement)
            ekf_state = np.array(ekf.x).flatten()
            ekf_error = np.linalg.norm(ekf_state[:2] - true_state[:2])
            ekf_errors.append(ekf_error)
        
        kf_mse = np.mean(np.array(kf_errors) ** 2)
        ekf_mse = np.mean(np.array(ekf_errors) ** 2)
        improvement = (1 - ekf_mse/kf_mse) * 100
        
        print(f"\n2D Tracking Accuracy:")
        print(f"  KF MSE (position):  {kf_mse:.6f}")
        print(f"  EKF MSE (position): {ekf_mse:.6f}")
        print(f"  Improvement:        {improvement:.2f}%")
        
        # Asserts significativos
        assert np.isfinite(kf_mse), f"KF MSE inválido: {kf_mse}"
        assert np.isfinite(ekf_mse), f"EKF MSE inválido: {ekf_mse}"
        assert kf_mse > 0, "KF MSE deve ser positivo"
        assert ekf_mse > 0, "EKF MSE deve ser positivo"
        # EKF não pode ser muito pior que KF
        tolerance_factor = 1.25
        assert ekf_mse <= kf_mse * tolerance_factor, (
            f"EKF degradou além do tolerado: KF={kf_mse:.6f}, EKF={ekf_mse:.6f}"
        )
        # Idealmente, EKF deve melhorar ou pelo menos não piorar
        assert improvement >= 0, (
            f"EKF não melhorou em relação ao KF: KF={kf_mse:.6f}, EKF={ekf_mse:.6f}"
        )
    
    def test_2d_velocity_tracking(self, filters_2d, trajectory_2d):
        """Testa a precisão de rastreamento de velocidade 2D."""
        kf, ekf = filters_2d
        true_trajectory, measurements = trajectory_2d
        dt = 0.016
        
        kf_vel_errors = []
        ekf_vel_errors = []
        
        for true_state, measurement in zip(true_trajectory, measurements):
            kf.predict(dt)
            kf.update(measurement)
            kf_state = np.array(kf.x).flatten()
            kf_vel_error = np.linalg.norm(kf_state[2:4] - true_state[2:4])
            kf_vel_errors.append(kf_vel_error)
            
            ekf.predict(dt)
            ekf.update(measurement)
            ekf_state = np.array(ekf.x).flatten()
            ekf_vel_error = np.linalg.norm(ekf_state[2:4] - true_state[2:4])
            ekf_vel_errors.append(ekf_vel_error)
        
        kf_vel_mse = np.mean(np.array(kf_vel_errors) ** 2)
        ekf_vel_mse = np.mean(np.array(ekf_vel_errors) ** 2)
        improvement = (1 - ekf_vel_mse/kf_vel_mse) * 100
        
        print(f"\n2D Velocity Tracking:")
        print(f"  KF MSE (velocity):  {kf_vel_mse:.6f}")
        print(f"  EKF MSE (velocity): {ekf_vel_mse:.6f}")
        print(f"  Improvement:        {improvement:.2f}%")
        
        # Asserts significativos
        assert np.isfinite(kf_vel_mse), f"KF MSE inválido: {kf_vel_mse}"
        assert np.isfinite(ekf_vel_mse), f"EKF MSE inválido: {ekf_vel_mse}"
        assert kf_vel_mse > 0, "KF MSE deve ser positivo"
        assert ekf_vel_mse > 0, "EKF MSE deve ser positivo"
        tolerance_factor = 1.25
        assert ekf_vel_mse <= kf_vel_mse * tolerance_factor, (
            f"EKF degradou além do tolerado: KF={kf_vel_mse:.6f}, EKF={ekf_vel_mse:.6f}"
        )
        assert improvement >= 0, (
            f"EKF não melhorou em relação ao KF: KF={kf_vel_mse:.6f}, EKF={ekf_vel_mse:.6f}"
        )
    
    def test_2d_convergence_speed(self, filters_2d, trajectory_2d):
        """Testa a velocidade de convergência 2D."""
        kf, ekf = filters_2d
        true_trajectory, measurements = trajectory_2d
        dt = 0.016
        
        convergence_steps = 50  # Primeiros 50 passos
        
        kf_errors = []
        ekf_errors = []
        
        for i in range(min(convergence_steps, len(measurements))):
            kf.predict(dt)
            kf.update(measurements[i])
            kf_state = np.array(kf.x).flatten()
            kf_error = np.linalg.norm(kf_state[:2] - true_trajectory[i][:2])
            kf_errors.append(kf_error)
            
            ekf.predict(dt)
            ekf.update(measurements[i])
            ekf_state = np.array(ekf.x).flatten()
            ekf_error = np.linalg.norm(ekf_state[:2] - true_trajectory[i][:2])
            ekf_errors.append(ekf_error)
        
        kf_initial_error = kf_errors[0]
        ekf_initial_error = ekf_errors[0]
        kf_final_error = kf_errors[-1]
        ekf_final_error = ekf_errors[-1]
        
        kf_convergence = (kf_initial_error - kf_final_error) / kf_initial_error * 100
        ekf_convergence = (ekf_initial_error - ekf_final_error) / ekf_initial_error * 100
        
        print(f"\n2D Convergence Speed (first {convergence_steps} steps):")
        print(f"  KF convergence:  {kf_convergence:.2f}%")
        print(f"  EKF convergence: {ekf_convergence:.2f}%")
        print(f"  KF initial error: {kf_initial_error:.6f}, final: {kf_final_error:.6f}")
        print(f"  EKF initial error: {ekf_initial_error:.6f}, final: {ekf_final_error:.6f}")
        
        # Asserts significativos
        assert np.isfinite(kf_initial_error) and np.isfinite(kf_final_error), "KF erros inválidos"
        assert np.isfinite(ekf_initial_error) and np.isfinite(ekf_final_error), "EKF erros inválidos"
        assert kf_initial_error > 0 and ekf_initial_error > 0, "Erro inicial deve ser positivo"
        assert kf_final_error >= 0 and ekf_final_error >= 0, "Erro final deve ser não-negativo"
        
        # Ambos devem convergir (erro final menor que inicial)
        assert kf_final_error < kf_initial_error, "KF não convergiu"
        assert ekf_final_error < ekf_initial_error, "EKF não convergiu"
        
        # EKF não pode ser muito pior que KF
        tolerance_factor = 1.25
        assert ekf_final_error <= kf_final_error * tolerance_factor, (
            f"EKF convergiu pior além do tolerado: KF final={kf_final_error:.6f}, EKF final={ekf_final_error:.6f}"
        )

    def test_2d_processing_time(self, filters_2d, trajectory_2d):
        """Testa o tempo de processamento 2D."""
        kf, ekf = filters_2d
        _, measurements = trajectory_2d
        dt = 0.016
        
        # Teste do KF
        start = time.perf_counter()
        for measurement in measurements:
            kf.predict(dt)
            kf.update(measurement)
        kf_time = (time.perf_counter() - start) / len(measurements) * 1e6  # μs
        
        # Teste do EKF
        start = time.perf_counter()
        for measurement in measurements:
            ekf.predict(dt)
            ekf.update(measurement)
        ekf_time = (time.perf_counter() - start) / len(measurements) * 1e6
        
        print(f"\n2D Processing Time:")
        print(f"  KF time per step:  {kf_time:.3f} μs")
        print(f"  EKF time per step: {ekf_time:.3f} μs")
        print(f"  EKF overhead:      {(ekf_time/kf_time - 1)*100:.1f}%")
        
        # Asserts significativos
        assert np.isfinite(kf_time), f"Tempo inválido para KF: {kf_time}"
        assert np.isfinite(ekf_time), f"Tempo inválido para EKF: {ekf_time}"
        assert kf_time > 0, "Tempo do KF deve ser positivo"
        assert ekf_time > 0, "Tempo do EKF deve ser positivo"
        
        # EKF pode ser mais lento, mas não absurdamente (ex: não mais que 10x o KF)
        max_factor = 10
        assert ekf_time <= kf_time * max_factor, (
            f"EKF muito mais lento que KF: KF={kf_time:.3f} μs, EKF={ekf_time:.3f} μs"
        )

    def test_2d_noise_robustness(self, filters_2d):
        """Testa robustez ao ruído em 2D."""
        kf, ekf = filters_2d
        dt = 0.016
        
        # Criar trajetória reta simples
        true_pos = np.array([0.0, 0.0])
        true_vel = np.array([1.0, 0.5])
        
        noise_levels = [0.01, 0.05, 0.1, 0.2]
        results = {}
        
        for noise_level in noise_levels:
            kf_errors = []
            ekf_errors = []
            
            for step in range(100):
                # Trajetória real
                true_pos_step = true_pos + true_vel * step * dt
                
                # Medição com ruído
                z = np.matrix([[true_pos_step[0] + np.random.normal(0, noise_level)],
                            [true_pos_step[1] + np.random.normal(0, noise_level)]])
                
                kf.predict(dt)
                kf.update(z)
                kf_state = np.array(kf.x).flatten()
                kf_error = np.linalg.norm(kf_state[:2] - true_pos_step)
                kf_errors.append(kf_error)
                
                ekf.predict(dt)
                ekf.update(z)
                ekf_state = np.array(ekf.x).flatten()
                ekf_error = np.linalg.norm(ekf_state[:2] - true_pos_step)
                ekf_errors.append(ekf_error)
            
            results[noise_level] = {
                'kf_mse': np.mean(np.array(kf_errors) ** 2),
                'ekf_mse': np.mean(np.array(ekf_errors) ** 2)
            }
        
        print(f"\n2D Noise Robustness:")
        kf_mse_values = []
        ekf_mse_values = []
        tolerance_factor = 1.25
        
        for noise_level, res in results.items():
            kf_mse = res['kf_mse']
            ekf_mse = res['ekf_mse']
            improvement = (1 - ekf_mse / kf_mse) * 100
            
            print(f"  Noise {noise_level}: KF MSE={kf_mse:.6f}, "
                f"EKF MSE={ekf_mse:.6f}, improvement={improvement:.2f}%")
            
            # Asserts significativos
            assert np.isfinite(kf_mse), f"KF MSE inválido para ruído {noise_level}: {kf_mse}"
            assert np.isfinite(ekf_mse), f"EKF MSE inválido para ruído {noise_level}: {ekf_mse}"
            assert kf_mse >= 0, f"KF MSE negativo para ruído {noise_level}: {kf_mse}"
            assert ekf_mse >= 0, f"EKF MSE negativo para ruído {noise_level}: {ekf_mse}"
            assert ekf_mse <= kf_mse * tolerance_factor, (
                f"EKF degradou além do tolerado para ruído {noise_level}: "
                f"KF MSE={kf_mse:.6f}, EKF MSE={ekf_mse:.6f}"
            )
            
            kf_mse_values.append(kf_mse)
            ekf_mse_values.append(ekf_mse)
        
        # Verificação média
        assert np.mean(ekf_mse_values) <= np.mean(kf_mse_values) * tolerance_factor, (
            f"EKF degradou além do tolerado na média: "
            f"KF MSE médio={np.mean(kf_mse_values):.6f}, "
            f"EKF MSE médio={np.mean(ekf_mse_values):.6f}"
        )


class TestKFvsEKF1D:
    """Testes para comparação de KF vs EKF em 1D (ângulo)."""
    
    @pytest.fixture
    def filters_1d(self):
        """Cria instâncias dos filtros 1D."""
        kf = KalmanFilterClass1D(a_sd=0.1, u=0.0, sd_acceleration=1.0)
        ekf = ExtendedKalmanFilterClass1D(a_sd=0.1, u=0.0, 
                                          sd_acceleration=1.0, friction=0.05)
        return kf, ekf
    
    @pytest.fixture
    def trajectory_1d(self) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Cria uma trajetória sintética 1D com rotação com atrito."""
        np.random.seed(42)
        dt = 0.016
        n_steps = 500
        
        true_trajectory = []
        measurements = []
        
        t = np.linspace(0, 10, n_steps)
        friction = 0.02
        
        for time_step in t:
            # Ângulo com velocidade angular decrescente
            omega = np.pi * np.exp(-friction * time_step)
            theta = omega * time_step / np.pi
            
            # Velocidade angular
            omega_actual = omega - theta * friction
            
            # Wrap angle
            theta = (theta + np.pi) % (2 * np.pi) - np.pi
            
            true_trajectory.append(np.array([theta, omega_actual]))
            
            # Medir com ruído
            z = np.matrix([[theta + np.random.normal(0, 0.05)]])
            measurements.append(z)
        
        return true_trajectory, measurements
    
    def test_1d_angle_tracking(self, filters_1d, trajectory_1d):
        """Testa o rastreamento de ângulo 1D."""
        kf, ekf = filters_1d
        true_trajectory, measurements = trajectory_1d
        dt = 0.016
        
        kf_errors = []
        ekf_errors = []
        
        for true_state, measurement in zip(true_trajectory, measurements):
            kf.predict(dt)
            kf.update(measurement)
            kf_state = np.array(kf.x).flatten()
            angle_diff = (kf_state[0] - true_state[0] + np.pi) % (2 * np.pi) - np.pi
            kf_errors.append(abs(angle_diff))
            
            ekf.predict(dt)
            ekf.update(measurement)
            ekf_state = np.array(ekf.x).flatten()
            angle_diff = (ekf_state[0] - true_state[0] + np.pi) % (2 * np.pi) - np.pi
            ekf_errors.append(abs(angle_diff))
        
        kf_mae = np.mean(kf_errors)
        ekf_mae = np.mean(ekf_errors)
        improvement = (1 - ekf_mae/kf_mae) * 100
        
        print(f"\n1D Angle Tracking:")
        print(f"  KF MAE (angle):  {kf_mae:.6f} rad")
        print(f"  EKF MAE (angle): {ekf_mae:.6f} rad")
        print(f"  Improvement:     {improvement:.2f}%")
        
        # Asserts significativos
        assert np.isfinite(kf_mae), f"KF MAE inválido: {kf_mae}"
        assert np.isfinite(ekf_mae), f"EKF MAE inválido: {ekf_mae}"
        assert kf_mae > 0, "KF MAE deve ser positivo"
        assert ekf_mae > 0, "EKF MAE deve ser positivo"
        tolerance_factor = 1.25
        assert ekf_mae <= kf_mae * tolerance_factor, (
            f"EKF degradou além do tolerado: KF={kf_mae:.6f}, EKF={ekf_mae:.6f}"
        )
        assert improvement >= 0, (
            f"EKF não melhorou em relação ao KF: KF={kf_mae:.6f}, EKF={ekf_mae:.6f}"
        )

    def test_1d_angular_velocity_tracking(self, filters_1d, trajectory_1d):
        """Testa o rastreamento de velocidade angular 1D."""
        kf, ekf = filters_1d
        true_trajectory, measurements = trajectory_1d
        dt = 0.016
        
        kf_vel_errors = []
        ekf_vel_errors = []
        
        for true_state, measurement in zip(true_trajectory, measurements):
            kf.predict(dt)
            kf.update(measurement)
            kf_state = np.array(kf.x).flatten()
            kf_vel_errors.append(abs(kf_state[1] - true_state[1]))
            
            ekf.predict(dt)
            ekf.update(measurement)
            ekf_state = np.array(ekf.x).flatten()
            ekf_vel_errors.append(abs(ekf_state[1] - true_state[1]))
        
        kf_vel_mae = np.mean(kf_vel_errors)
        ekf_vel_mae = np.mean(ekf_vel_errors)
        improvement = (1 - ekf_vel_mae/kf_vel_mae) * 100
        
        print(f"\n1D Angular Velocity Tracking:")
        print(f"  KF MAE (omega):  {kf_vel_mae:.6f} rad/s")
        print(f"  EKF MAE (omega): {ekf_vel_mae:.6f} rad/s")
        print(f"  Improvement:     {improvement:.2f}%")
        
        # Asserts significativos
        assert np.isfinite(kf_vel_mae), f"KF MAE inválido: {kf_vel_mae}"
        assert np.isfinite(ekf_vel_mae), f"EKF MAE inválido: {ekf_vel_mae}"
        assert kf_vel_mae > 0, "KF MAE deve ser positivo"
        assert ekf_vel_mae > 0, "EKF MAE deve ser positivo"
        tolerance_factor = 1.25
        assert ekf_vel_mae <= kf_vel_mae * tolerance_factor, (
            f"EKF degradou além do tolerado: KF={kf_vel_mae:.6f}, EKF={ekf_vel_mae:.6f}"
        )
        assert improvement >= 0, (
            f"EKF não melhorou em relação ao KF: KF={kf_vel_mae:.6f}, EKF={ekf_vel_mae:.6f}"
        )
    
    def test_1d_processing_time(self, filters_1d, trajectory_1d):
        """Testa o tempo de processamento 1D."""
        kf, ekf = filters_1d
        _, measurements = trajectory_1d
        dt = 0.016
        
        # KF
        start = time.perf_counter()
        for measurement in measurements:
            kf.predict(dt)
            kf.update(measurement)
        kf_time = (time.perf_counter() - start) / len(measurements) * 1e6
        
        # EKF
        start = time.perf_counter()
        for measurement in measurements:
            ekf.predict(dt)
            ekf.update(measurement)
        ekf_time = (time.perf_counter() - start) / len(measurements) * 1e6
        
        print(f"\n1D Processing Time:")
        print(f"  KF time per step:  {kf_time:.3f} μs")
        print(f"  EKF time per step: {ekf_time:.3f} μs")
        print(f"  EKF overhead:      {(ekf_time/kf_time - 1)*100:.1f}%")
        
        # Asserts significativos
        assert np.isfinite(kf_time), f"Tempo inválido para KF: {kf_time}"
        assert np.isfinite(ekf_time), f"Tempo inválido para EKF: {ekf_time}"
        assert kf_time > 0, "Tempo do KF deve ser positivo"
        assert ekf_time > 0, "Tempo do EKF deve ser positivo"
        max_factor = 10
        assert ekf_time <= kf_time * max_factor, (
            f"EKF muito mais lento que KF: KF={kf_time:.3f} μs, EKF={ekf_time:.3f} μs"
        )


class TestParameterComparison:
    """Testes de sensibilidade a parâmetros."""
    
    def test_friction_impact_2d(self):
        """Testa o impacto do parâmetro friction no EKF 2D."""
        print(f"\n2D Friction Impact:")
        friction_values = [0.0, 0.02, 0.05, 0.1, 0.2]
        
        np.random.seed(42)
        dt = 0.016
        n_steps = 200
        
        for friction in friction_values:
            ekf = ExtendedKalmanFilterClass2D(x_sd=0.1, y_sd=0.1, friction=friction)
            
            errors = []
            for step in range(n_steps):
                t = step * dt
                radius = 2.0 * np.exp(-friction * t)
                x_true = radius * np.cos(2 * np.pi * t)
                y_true = radius * np.sin(2 * np.pi * t)
                
                z = np.matrix([[x_true + np.random.normal(0, 0.05)],
                            [y_true + np.random.normal(0, 0.05)]])
                
                ekf.predict(dt)
                ekf.update(z)
                ekf_state = np.array(ekf.x).flatten()
                error = np.linalg.norm(ekf_state[:2] - np.array([x_true, y_true]))
                errors.append(error)
            
            mse = np.mean(np.array(errors) ** 2)
            print(f"  Friction={friction}: MSE={mse:.6f}")
            
            # Asserts significativos
            assert np.isfinite(mse), f"MSE inválido para friction={friction}: {mse}"
            assert mse >= 0, f"MSE negativo para friction={friction}: {mse}"
            # Erro não deve explodir (limite arbitrário de segurança)
            assert mse < 10, f"MSE muito alto para friction={friction}: {mse}"
    
    def test_friction_impact_1d(self):
        """Testa o impacto do parâmetro friction no EKF 1D."""
        print(f"\n1D Friction Impact:")
        friction_values = [0.0, 0.02, 0.05, 0.1, 0.2]
        
        np.random.seed(42)
        dt = 0.016
        n_steps = 200
        
        for friction in friction_values:
            ekf = ExtendedKalmanFilterClass1D(a_sd=0.1, friction=friction)
            
            errors = []
            for step in range(n_steps):
                t = step * dt
                theta_true = np.pi * np.exp(-friction * t)
                
                z = np.matrix([[theta_true + np.random.normal(0, 0.05)]])
                
                ekf.predict(dt)
                ekf.update(z)
                ekf_state = np.array(ekf.x).flatten()
                angle_diff = (ekf_state[0] - theta_true + np.pi) % (2 * np.pi) - np.pi
                errors.append(abs(angle_diff))
            
            mae = np.mean(errors)
            print(f"  Friction={friction}: MAE={mae:.6f} rad")
            
            # Asserts significativos
            assert np.isfinite(mae), f"MAE inválido para friction={friction}: {mae}"
            assert mae >= 0, f"MAE negativo para friction={friction}: {mae}"
            # Erro não deve explodir (limite arbitrário de segurança)
            assert mae < 5, f"MAE muito alto para friction={friction}: {mae}"


def test_summary():
    """Resumo da suíte de testes KF vs EKF."""
    print("\n" + "="*70)
    print("TESTE COMPARATIVO: KALMAN FILTER vs EXTENDED KALMAN FILTER")
    print("="*70)
    
    # Verificação simples: garantir que as classes de teste existem
    assert 'TestKFvsEKF2D' in globals(), "Classe TestKFvsEKF2D não encontrada"
    assert 'TestKFvsEKF1D' in globals(), "Classe TestKFvsEKF1D não encontrada"
    assert 'TestParameterComparison' in globals(), "Classe TestParameterComparison não encontrada"


if __name__ == "__main__":
    # Executar com pytest para mais detalhes
    pytest.main([__file__, "-v", "-s"])
