"""
Benchmark de latência do CBF Filter.

Mede:
1. Tempo de Filter() isolado, em diferentes cenários (longe de
   restrições, perto de 1 restrição, perto de 2 restrições).
2. Comparação "com CBF" vs "sem CBF" (passthrough) no timer_callback
   simulado, pra medir o overhead real introduzido no ciclo de controle.

Rodar com: python3 benchmark_cbf_latency.py
"""
import sys
import os
import time
import statistics

from asif_filter import AsifFilter
from new_movement.entities.States import State, Vector2D
import cvxpy as cp


N_WARMUP = 200
N_ITERS = 5000


def make_filter():
    """Instancia AsifFilter pulando __init__ (sem precisar de rclpy real)."""
    node = AsifFilter.__new__(AsifFilter)

    node.gamma_field = 4.0
    node.gamma_prohibited = 4.0
    node.gamma_robot = 4.0
    node.d_min = 0.20
    node.robot_margin = 0.10
    node.rho = 100.0

    node.field_half_length = 4.5
    node.field_half_width = 3.0
    node.prohibited_zones = [(-4.5, -3.5, -1.0, 1.0)]

    n_field = 4
    n_prohibited = len(node.prohibited_zones)
    node._n_constraints = n_field + n_prohibited

    node._u_des_param = cp.Parameter(2)
    node._A_param = cp.Parameter((node._n_constraints, 2))
    node._B_param = cp.Parameter(node._n_constraints)

    node._u_var = cp.Variable(2)
    node._delta_var = cp.Variable(node._n_constraints, nonneg=True)

    constraints = [node._A_param @ node._u_var <= node._B_param + node._delta_var]
    objective = cp.Minimize(
        cp.sum_squares(node._u_var - node._u_des_param)
        + node.rho * cp.sum_squares(node._delta_var)
    )
    node._qp = cp.Problem(objective, constraints)

    class _Logger:
        def warn(self, msg): pass
        def info(self, msg): pass
        def error(self, msg): pass
    node.get_logger = lambda: _Logger()

    return node


def measure(fn, n_iters=N_ITERS, n_warmup=N_WARMUP):
    """Roda fn() n_iters vezes e retorna a lista de tempos em milissegundos."""
    for _ in range(n_warmup):
        fn()

    times_ms = []
    for _ in range(n_iters):
        t0 = time.perf_counter()
        fn()
        t1 = time.perf_counter()
        times_ms.append((t1 - t0) * 1000.0)
    return times_ms


def report(label, times_ms):
    times_sorted = sorted(times_ms)
    n = len(times_sorted)
    mean = statistics.mean(times_ms)
    median = statistics.median(times_ms)
    p95 = times_sorted[int(n * 0.95)]
    p99 = times_sorted[int(n * 0.99)]
    worst = times_sorted[-1]

    print(f"\n--- {label} ---")
    print(f"  n         : {n}")
    print(f"  média     : {mean:.4f} ms")
    print(f"  mediana   : {median:.4f} ms")
    print(f"  p95       : {p95:.4f} ms")
    print(f"  p99       : {p99:.4f} ms")
    print(f"  pior caso : {worst:.4f} ms")
    return {"mean": mean, "median": median, "p95": p95, "p99": p99, "worst": worst}


def main():
    node = make_filter()

    scenarios = {
        "longe_de_restricoes": (State(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0)), Vector2D(1.0, 1.0)),
        "perto_1_borda": (State(Vector2D(4.35, 0.0), Vector2D(0.0, 0.0)), Vector2D(2.0, 0.0)),
        "perto_2_bordas": (State(Vector2D(4.35, 2.85), Vector2D(0.0, 0.0)), Vector2D(2.0, 2.0)),
        "dentro_area_proibida": (State(Vector2D(-4.0, 0.0), Vector2D(0.0, 0.0)), Vector2D(1.0, 0.0)),
    }

    print("=" * 60)
    print("BENCHMARK 1: Filter() isolado, por cenário de posição")
    print("=" * 60)

    results = {}
    for name, (cur_state, u_des) in scenarios.items():
        fn = lambda cs=cur_state, ud=u_des: node.Filter(cs, ud)
        times = measure(fn)
        results[name] = report(name, times)

    print("\n" + "=" * 60)
    print("BENCHMARK 2: Ciclo de controle - COM CBF vs SEM CBF (passthrough)")
    print("=" * 60)

    cur_state, u_des = scenarios["perto_1_borda"]

    def com_cbf():
        # Simula o que o timer_callback faz por robô: chama Filter()
        safe = node.Filter(cur_state, u_des)
        return safe

    def sem_cbf():
        # Passthrough: sem CBF, comando desejado vai direto
        return u_des

    times_com = measure(com_cbf)
    times_sem = measure(sem_cbf)

    r_com = report("COM CBF", times_com)
    r_sem = report("SEM CBF (passthrough)", times_sem)

    overhead_ms = r_com["mean"] - r_sem["mean"]
    print(f"\n>>> Overhead médio introduzido pelo CBF: {overhead_ms:.4f} ms por robô")
    print(f">>> Para 6 robôs em campo: ~{overhead_ms * 6:.4f} ms por ciclo")
    print(f">>> Orçamento do ciclo a 100Hz: 10 ms totais")


if __name__ == "__main__":
    main()
