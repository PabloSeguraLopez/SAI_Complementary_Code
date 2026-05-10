import math
from utils import wrap_pi, clamp


class VFHPlanner:
    """
    Vector Field Histogram planner.
    Implementación cercana a Borenstein & Koren (1991).
    """

    def __init__(self, config):
        self.hist_bins = config["hist_bins"]
        self.alpha = 2.0 * math.pi / self.hist_bins

        self.a = config.get("vfh_a", 1.0)
        self.b = config.get("vfh_b", 1.0 / config["influence_distance"])

        self.threshold = config["threshold"]
        self.l = config.get("smooth_window", 5)
        self.s_max = config.get("s_max", 18)

        self.trap_detected = False
        # +1 = lado angular positivo desde k_targ
        # -1 = lado angular negativo desde k_targ
        # Ojo: esto es "lado en el histograma", no lo etiqueto como
        # izquierda/derecha física para no mezclar convenciones.
        self.diversion_mode = 0

    def reset(self):
        self.trap_detected = False
        self.diversion_mode = 0

    def build_histogram(self, position, yaw, measurements):
        hist = [0.0] * self.hist_bins

        for measurement in measurements:
            if len(measurement) == 3:
                angle, dist, c = measurement
            else:
                angle, dist = measurement
                c = 1.0

            beta = wrap_pi(angle)
            k = int((beta + math.pi) / (2.0 * math.pi) * self.hist_bins) % self.hist_bins
            m = (c ** 2) * max(0.0, self.a - self.b * dist)
            hist[k] += m

        return hist

    def smooth_histogram(self, hist):
        smoothed = [0.0] * self.hist_bins
        l = self.l
        denominator = float(2 * l + 1)

        for k in range(self.hist_bins):
            acc = 0.0
            for i in range(-l, l + 1):
                kk = (k + i) % self.hist_bins
                weight = (l + 1) - abs(i)
                acc += hist[kk] * weight
            smoothed[k] = acc / denominator

        return smoothed

    def bin_to_angle(self, k):
        return wrap_pi((k / self.hist_bins) * 2.0 * math.pi - math.pi)

    def angle_to_bin(self, angle):
        k = int((wrap_pi(angle) + math.pi) / (2.0 * math.pi) * self.hist_bins)
        return k % self.hist_bins

    def _circular_distance_bins(self, a, b):
        d = abs(a - b)
        return min(d, self.hist_bins - d)

    def _signed_offset(self, k_ref, k_candidate):
        """
        Devuelve el desplazamiento angular discreto más corto de k_ref a
        k_candidate en [-n/2, n/2]. Signo positivo = sentido angular positivo
        del histograma; negativo = sentido angular negativo.
        """
        n = self.hist_bins
        raw = (k_candidate - k_ref) % n
        if raw > n // 2:
            raw -= n
        return raw

    def _side_of(self, k_ref, k_candidate):
        """
        Lado de k_candidate respecto a k_ref:
          +1 -> lado angular positivo
          -1 -> lado angular negativo
           0 -> mismo sector
        En empate exacto (solo posible con n par y oposición exacta), usa +1.
        """
        off = self._signed_offset(k_ref, k_candidate)
        if off == 0:
            return 0
        return +1 if off >= 0 else -1

    def _find_valleys(self, free):
        n = self.hist_bins
        start_search = None
        for i in range(n):
            if not free[i]:
                start_search = i
                break

        if start_search is None:
            return [(0, n - 1)]

        valleys = []
        in_valley = False
        k_start = 0

        for step in range(n):
            k = (start_search + step) % n
            if free[k]:
                if not in_valley:
                    k_start = k
                    in_valley = True
            else:
                if in_valley:
                    k_end = (k - 1) % n
                    valleys.append((k_start, k_end))
                    in_valley = False

        if in_valley:
            k_end = (start_search - 1) % n
            valleys.append((k_start, k_end))

        return valleys

    def _valley_width(self, k_start, k_end):
        return (k_end - k_start) % self.hist_bins + 1

    def _iter_valley_bins(self, k_start, k_end):
        width = self._valley_width(k_start, k_end)
        for i in range(width):
            yield (k_start + i) % self.hist_bins

    def _nearest_bin_in_valley(self, k_targ, k_start, k_end, side=None):
        """
        Devuelve el sector del valley más cercano a k_targ.

        side:
          None -> cualquier lado
          +1   -> solo bins en el lado angular positivo de k_targ
          -1   -> solo bins en el lado angular negativo de k_targ

        Desempate:
          1) menor distancia circular
          2) si hay side, preferir ese lado
          3) si no hay side, preferir lado positivo para hacer el resultado
             determinista
        """
        best_k = None
        best_d = float("inf")
        best_side_rank = float("inf")

        for k in self._iter_valley_bins(k_start, k_end):
            s = self._side_of(k_targ, k)

            if side in (+1, -1) and s != side:
                continue

            d = self._circular_distance_bins(k_targ, k)
            side_rank = 0 if s == +1 else 1  # desempate determinista

            if d < best_d or (d == best_d and side_rank < best_side_rank):
                best_d = d
                best_side_rank = side_rank
                best_k = k

        return best_k

    def _closest_valley_to_target(self, k_targ, valleys, side=None):
        """
        Selecciona el valley cuyo sector libre más cercano a k_targ esté más próximo.
        Esto es más fiel que comparar solo los bordes.
        """
        best_valley = None
        best_kn = None
        best_dist = float("inf")
        best_side_rank = float("inf")

        for (k_start, k_end) in valleys:
            kn = self._nearest_bin_in_valley(k_targ, k_start, k_end, side=side)
            if kn is None:
                continue

            d = self._circular_distance_bins(k_targ, kn)
            s = self._side_of(k_targ, kn)
            side_rank = 0 if s == +1 else 1

            if d < best_dist or (d == best_dist and side_rank < best_side_rank):
                best_dist = d
                best_side_rank = side_rank
                best_valley = (k_start, k_end)
                best_kn = kn

        return best_valley, best_kn

    def _opposite_border(self, k_start, k_end, k_n):
        """
        En un valley estrecho, k_f es el borde opuesto real a k_n.
        """
        if k_n == k_start:
            return k_end
        if k_n == k_end:
            return k_start

        # Si k_n no es exactamente un borde por discretización o empate,
        # usar el borde más alejado de k_n a lo largo del valley.
        d_start = (k_n - k_start) % self.hist_bins
        d_end = (k_end - k_n) % self.hist_bins
        return k_start if d_start > d_end else k_end

    def _interior_step_from_border(self, k_start, k_end, k_border):
        """
        Devuelve la dirección (+1 o -1) que entra hacia el interior del valley
        partiendo de un borde real.
        """
        if k_border == k_start:
            return +1
        if k_border == k_end:
            return -1

        # Caso raro: k_border no coincide exactamente con un borde.
        # Elegimos el sentido que permanezca dentro del valley.
        valley = set(self._iter_valley_bins(k_start, k_end))
        if ((k_border + 1) % self.hist_bins) in valley:
            return +1
        return -1

    def _find_kn_with_monitor(self, k_targ, k_start, k_end, side=None):
        """
        Path monitor según Sec. 4.5:
        buscar k_n al lado indicado de k_targ y, si no aparece dentro de
        180°/alpha sectores, declarar trap.
        """
        n_trap = int(math.pi / self.alpha)

        valley = set(self._iter_valley_bins(k_start, k_end))

        if side in (+1, -1):
            for step in range(1, n_trap + 1):
                k = (k_targ + side * step) % self.hist_bins
                if k in valley:
                    return k
            return None

        # Sin diversion mode todavía: probar ambos lados por cercanía.
        for step in range(1, n_trap + 1):
            kp = (k_targ + step) % self.hist_bins
            km = (k_targ - step) % self.hist_bins

            if kp in valley:
                return kp
            if km in valley:
                return km

        return None

    def _circular_midpoint(self, k_a, k_b):
        arc_cw = (k_b - k_a) % self.hist_bins
        return (k_a + arc_cw // 2) % self.hist_bins

    def select_heading(self, yaw, goal_heading, histogram):
        """
        Selección de steering direction más cercana al paper.
        """
        self.trap_detected = False

        free = [h < self.threshold for h in histogram]
        k_targ = self.angle_to_bin(goal_heading)

        # Si el objetivo ya cae en sector libre, ir directo y limpiar monitor.
        if free[k_targ]:
            self.diversion_mode = 0
            return self.bin_to_angle(k_targ)

        valleys = self._find_valleys(free)
        if not valleys:
            self.trap_detected = True
            return yaw

        # 1) Elegir el valley más cercano a k_targ, restringiendo por el
        # diversion mode solo si ya existe.
        preferred_side = self.diversion_mode if self.diversion_mode != 0 else None
        best_valley, _ = self._closest_valley_to_target(
            k_targ, valleys, side=preferred_side
        )

        # Si por la restricción lateral no hay ninguno, relajar solo la
        # elección del valley; el trap lo decide luego la búsqueda de k_n.
        if best_valley is None:
            best_valley, _ = self._closest_valley_to_target(
                k_targ, valleys, side=None
            )

        if best_valley is None:
            self.trap_detected = True
            return yaw

        k_start, k_end = best_valley

        # 2) Buscar k_n conforme al path monitor.
        k_n = self._find_kn_with_monitor(
            k_targ,
            k_start,
            k_end,
            side=preferred_side
        )

        if k_n is None:
            self.trap_detected = True
            return yaw

        # 3) Registrar diversion mode la primera vez.
        if self.diversion_mode == 0:
            side = self._side_of(k_targ, k_n)
            self.diversion_mode = +1 if side >= 0 else -1

        # 4) Determinar k_f según valley ancho/estrecho.
        valley_width = self._valley_width(k_start, k_end)

        if valley_width > self.s_max:
            # Avanzar hacia el interior del valley desde k_n.
            interior_step = self._interior_step_from_border(k_start, k_end, k_n)
            k_f = (k_n + interior_step * self.s_max) % self.hist_bins
        else:
            # Usar el borde opuesto real, no diversion_mode.
            k_f = self._opposite_border(k_start, k_end, k_n)

        # 5) Dirección final = centro entre k_n y k_f
        k_selected = self._circular_midpoint(k_n, k_f)
        return self.bin_to_angle(k_selected)

    def compute_speed(self, dist_to_goal, desired_heading, current_yaw,
                      histogram, config, omega_actual=0.0):
        v_max = config["v_max"]
        v_min = config["v_min"]
        h_m = config["h_m"]
        omega_max = config["yaw_rate_max"]

        yaw_rate_cmd = clamp(
            wrap_pi(desired_heading - current_yaw),
            -omega_max,
            omega_max,
        )

        sector_current = self.angle_to_bin(current_yaw)
        h_c_prime = histogram[sector_current]
        h_c_double_prime = min(h_c_prime, h_m)
        v_prime = v_max * (1.0 - h_c_double_prime / h_m)

        omega_ratio = abs(omega_actual) / omega_max if omega_max > 0.0 else 0.0
        v = v_prime * (1.0 - omega_ratio) + v_min
        v = max(v, v_min)

        return v, yaw_rate_cmd