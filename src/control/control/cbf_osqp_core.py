import numpy as np
import scipy.sparse as sp
import osqp

class CBFOsqpCore:
    def __init__(self, gamma_field, gamma_prohibited, robot_margin, rho, field_half_length,
                 field_half_width, prohibited_zones):

        self.gamma_field = gamma_field
        self.gamma_prohibited = gamma_prohibited
        self.robot_margin = robot_margin
        self.rho = rho
        self.field_half_length = field_half_length
        self.field_half_width = field_half_width
        self.prohibited_zones = prohibited_zones

        self.n_field = 4
        self.n_prohibited = len(prohibited_zones)
        self.n = self.n_field +self.n_prohibited
        self.nx = 2 + self.n
        self.n_rows = 2 * self.n

        self._build_qp()

    def _build_qp(self):
        n, nx = self.n, self.nx

        P_diag = np.array([2.0, 2.0] + [2.0 * self.rho] * n)
        self.P = sp.diags(P_diag, format="csc")

        self.q = np.zeros(nx)

        rows, cols, data = [], [], []

        #Field Borders: [+x, -x, +y, -y]
        field_coefs = [(0, 1.0), (0, -1.0), (1, 1.0), (1, -1.0)]
        self._field_row_idx = list(range(self.n_field))
        for i, (col, val) in enumerate(field_coefs):
            rows.append(i); cols.append(col); data.append(val)

            rows.append(i); cols.append(2 + i); data.append(-1.0)

        self._prohibited_row_idx = list(range(self.n_field, self.n))
        self._prohibited_A_slot = {} #index array 'data'

        for k, row in enumerate(self._prohibited_row_idx):
            idx0 = len(data)
            rows.append(row); cols.append(0); data.append(1e-12)
            idx1 = len(data)
            rows.append(row); cols.append(1); data.append(1e-12)
            self._prohibited_A_slot[row] = (idx0, idx1)

            rows.append(row); cols.append(2 + row); data.append(-1.0)

        for i in range(n):
            rows.append(n + i); cols.append(2 + i); data.append(1.0)

        A_row = np.array(rows)
        A_col = np.array(cols)
        A_data_unsorted = np.array(data, dtype=float)

        self.A = sp.csc_matrix(
            (A_data_unsorted, (A_row, A_col)),
            shape=(self.n_rows, nx),
        )

        self._A_data = self.A.data

        self._reindex_after_csc()

        self.l = np.hstack([np.full(n, -np.inf), np.zeros(n)])
        self.u = np.hstack([np.zeros(n), np.full(n, np.inf)])

        self.m = osqp.OSQP()
        self.m.setup(self.P, self.q, self.A, self.l, self.u, warm_start=True, verbose=False)

    def _reindex_after_csc(self):
        self._pos_in_data = {}

        for col in range(self.A.shape[1]):
            start, end = self.A.indptr[col], self.A.indptr[col + 1]
            for ptr in range(start, end):
                row = self.A.indices[ptr]
                self._pos_in_data[(row, col)] = ptr

    def _field_b(self, px, py):
        L, W, m, g = self.field_half_length, self.field_half_width, self.robot_margin, self.gamma_field
        return [
            g * ((L - m) - px),
            g * (px + (L - m)),
            g * ((W - m) - py),
            g * (py + (W - m)),
        ]

    def _prohibited_row(self, px, py, zone):
        g, m = self.gamma_prohibited, self.robot_margin
        x_min, x_max, y_min, y_max = zone
        cx = min(max(px, x_min), x_max)
        cy = min(max(py, y_min), y_max)
        dx, dy = px - cx, py - cy
        dist = (dx**2 + dy**2) ** 0.5

        if dist < 1e-6:
            return 0.0, 0.0, 1e6

        h = dist - m
        gx, gy = -dx / dist, -dy / dist

        return gx, gy, g * h

    def solve(self, px, py, udx, udy):
        n = self.n
        b = self._field_b(px, py)

        for zone in self.prohibited_zones:
            gx, gy, bz = self._prohibited_row(px, py, zone)
            b.append(bz)
            row = self._prohibited_row_idx[self.prohibited_zones.index(zone)]
            idx0 = self._pos_in_data[(row, 0)]
            idx1 = self._pos_in_data[(row, 1)]

            self._A_data[idx0] = gx if gx != 0.0 else 1e-12
            self._A_data[idx1] = gy if gy != 0.0 else 1e-12

        u_new = np.hstack([np.array(b), np.full(n, np.inf)])
        q_new = np.array([-2.0 * udx, -2.0 * udy] + [0.0] * n)

        self.m.update(q=q_new, u=u_new, Ax=self._A_data)
        result = self.m.solve()

        if result.info.status not in ("solved", "solved inaccurate"):
            return 0.0, 0.0
        return float(result.x[0]), float(result.x[1])
